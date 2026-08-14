package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.OptionalDouble;

/**
 * TrajectorySolver handles 2D ballistic kinematics for fixed-flywheel hood aiming.
 * Centralizes velocity conversions, minimum velocity calculations, closed-form
 * launch angle quadratic solving, domain boundary checks, and inertia-preserving
 * flywheel/hood trajectory planning.
 */
public class TrajectorySolver {

    // Physical Hood Limits (Radians)
    public static final double MIN_HOOD_ANGLE_RAD = Units.degreesToRadians(15.0);
    public static final double MAX_HOOD_ANGLE_RAD = Units.degreesToRadians(75.0);
    public static final double DEFAULT_HOOD_ANGLE_RAD = Units.degreesToRadians(45.0);

    // Scoring Distance Limits (Meters)
    public static final double MIN_DISTANCE_METERS = 1.5;
    public static final double MAX_DISTANCE_METERS = 7.0;

    // Flywheel RPM Limits
    public static final double MIN_FLYWHEEL_RPM = 500.0;
    public static final double MAX_FLYWHEEL_RPM = 6000.0;
    public static final double DEFAULT_FLYWHEEL_RPM = Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;

    /**
     * Data record representing trajectory computation output.
     */
    public record TrajectoryResult(
        double desiredHoodAngleRad,
        double targetFlywheelRPM,
        boolean isValid,
        boolean flywheelAdjusted,
        boolean isUnreachable
    ) {}

    /**
     * Converts exit velocity v0 (m/s) to flywheel RPM.
     * RPM = (60 * v0) / (pi * D * C_comp)
     */
    public static double exitVelocityToRPM(double v0) {
        if (v0 <= 0.0 || Double.isNaN(v0) || Double.isInfinite(v0)) {
            return 0.0;
        }
        double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
        double compressionRatio = Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
        return (60.0 * v0) / (Math.PI * wheelDiameterMeters * compressionRatio);
    }

    /**
     * Converts flywheel RPM to exit velocity v0 (m/s).
     * v0 = (RPM * pi * D * C_comp) / 60
     */
    public static double rpmToExitVelocity(double rpm) {
        if (rpm <= 0.0 || Double.isNaN(rpm) || Double.isInfinite(rpm)) {
            return 0.0;
        }
        double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
        double compressionRatio = Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
        return (rpm * Math.PI * wheelDiameterMeters * compressionRatio) / 60.0;
    }

    /**
     * Calculates absolute minimum exit velocity required to hit target height h at distance d.
     * v_min = sqrt(g * (h + sqrt(d^2 + h^2)))
     */
    public static double calculateMinimumVelocity(double distanceMeters, double targetHeightMeters) {
        if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
            Double.isNaN(targetHeightMeters) || Double.isInfinite(targetHeightMeters)) {
            return 0.0;
        }
        double g = Constants.Shooter.kGRAVITATIONAL_CONSTANT;
        double d = Math.max(0.0, distanceMeters);
        double h = targetHeightMeters;
        double radicand = d * d + h * h;
        double term = h + Math.sqrt(radicand);
        if (term < 0.0) {
            return 0.0;
        }
        double vMin = Math.sqrt(g * term);
        return (Double.isNaN(vMin) || Double.isInfinite(vMin)) ? 0.0 : vMin;
    }

    /**
     * Solves closed-form launch angle theta for a fixed exit velocity v0:
     * theta = atan((v0^2 - sqrt(v0^4 - g*(g*d^2 + 2*h*v0^2))) / (g*d))
     *
     * @return OptionalDouble containing launch angle in radians, or empty if complex root or out of bounds.
     */
    public static OptionalDouble solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters) {
        if (exitVelocityMetersPerSec <= 0.0 || distanceMeters <= 0.0 ||
            Double.isNaN(exitVelocityMetersPerSec) || Double.isNaN(distanceMeters) ||
            Double.isNaN(targetHeightMeters) || Double.isInfinite(exitVelocityMetersPerSec) ||
            Double.isInfinite(distanceMeters) || Double.isInfinite(targetHeightMeters)) {
            return OptionalDouble.empty();
        }

        double v0 = exitVelocityMetersPerSec;
        double d = distanceMeters;
        double h = targetHeightMeters;
        double g = Constants.Shooter.kGRAVITATIONAL_CONSTANT;

        double v0Sq = v0 * v0;
        double v0Quad = v0Sq * v0Sq;
        double radicand = v0Quad - g * (g * d * d + 2.0 * h * v0Sq);

        // Complex roots condition (v0 < v_min)
        if (radicand < 0.0) {
            return OptionalDouble.empty();
        }

        double tanTheta = (v0Sq - Math.sqrt(radicand)) / (g * d);
        double theta = Math.atan(tanTheta);

        // Domain & safety validation checks
        if (Double.isNaN(theta) || Double.isInfinite(theta)) {
            return OptionalDouble.empty();
        }

        // Guard condition: d * tan(theta) > h
        if (d * tanTheta <= h) {
            return OptionalDouble.empty();
        }

        if (theta <= 0.0 || theta >= Math.PI / 2.0) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(theta);
    }

    /**
     * Calculates trajectory setpoints maintaining current flywheel speed when possible (hood-first).
     *
     * @param currentFlywheelRPM Current flywheel RPM of shooter.
     * @param distanceMeters Distance to target hub in meters.
     * @return TrajectoryResult record containing hood angle, target RPM, and status flags.
     */
    public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters) {
        double targetHeightMeters = Units.inchesToMeters(Constants.Hood.ScoreHeight);

        // 1. Input Validation Guard (Distance & Flywheel RPM NaN/Infinity/Bounds)
        if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
            Double.isNaN(currentFlywheelRPM) || Double.isInfinite(currentFlywheelRPM) ||
            distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS) {
            return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true);
        }

        // 2. Attempt Hood-First Solve at current flywheel RPM
        if (currentFlywheelRPM > 0.0 && !Double.isNaN(currentFlywheelRPM) && !Double.isInfinite(currentFlywheelRPM)) {
            double v0Current = rpmToExitVelocity(currentFlywheelRPM);
            OptionalDouble angleOpt = solveLaunchAngle(v0Current, distanceMeters, targetHeightMeters);

            if (angleOpt.isPresent()) {
                double theta = angleOpt.getAsDouble();
                if (theta >= MIN_HOOD_ANGLE_RAD && theta <= MAX_HOOD_ANGLE_RAD) {
                    // SUCCESS: Hood angle accommodates distance at fixed current RPM!
                    return new TrajectoryResult(theta, currentFlywheelRPM, true, false, false);
                }
            }
        }

        // 3. Flywheel Speed Adjustment Needed
        double optAngle = (Math.PI / 4.0) + 0.5 * Math.atan2(targetHeightMeters, distanceMeters);
        double cosOpt = Math.cos(optAngle);
        double denom = 2.0 * (distanceMeters * Math.tan(optAngle) - targetHeightMeters);

        if (denom <= 0.001 || Math.abs(cosOpt) < 0.001) {
            return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, true, true);
        }

        double g = Constants.Shooter.kGRAVITATIONAL_CONSTANT;
        double v0Required = (1.0 / cosOpt) * Math.sqrt((g * distanceMeters * distanceMeters) / denom) * 1.001;
        double targetRPM = Math.max(MIN_FLYWHEEL_RPM, Math.min(MAX_FLYWHEEL_RPM, exitVelocityToRPM(v0Required)));

        double v0Adjusted = rpmToExitVelocity(targetRPM);
        OptionalDouble adjustedAngleOpt = solveLaunchAngle(v0Adjusted, distanceMeters, targetHeightMeters);

        if (adjustedAngleOpt.isPresent()) {
            double thetaAdj = adjustedAngleOpt.getAsDouble();
            if (thetaAdj >= MIN_HOOD_ANGLE_RAD && thetaAdj <= MAX_HOOD_ANGLE_RAD) {
                return new TrajectoryResult(thetaAdj, targetRPM, true, true, false);
            }
        }

        // 4. Target Unreachable Fallback
        return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, true, true);
    }
}
