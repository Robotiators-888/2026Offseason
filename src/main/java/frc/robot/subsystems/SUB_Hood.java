package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOHardware;
import frc.robot.subsystems.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.hood.HoodIOSim;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the adjustable shooter hood angle mechanism with AdvantageKit IO abstraction.
 */
public class SUB_Hood extends SubsystemBase {
        private static SUB_Hood INSTANCE = null;
        private final HoodIO io;
        private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

        private double desiredAngle = 0;

        public static SUB_Hood getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Hood();
                }
                return INSTANCE;
        }

        public SUB_Hood(HoodIO io) {
                this.io = io;
                INSTANCE = this;
        }

        public SUB_Hood() {
                this(createIO());
        }

        private static HoodIO createIO() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                return new HoodIOHardware();
                        case SIM:
                                return new HoodIOSim();
                        case REPLAY:
                        default:
                                return new HoodIO() {};
                }
        }

        public void setPosition(final double angle) {
                desiredAngle = Math.max(
                    Constants.Hood.kMinAngle, Math.min(Constants.Hood.kMaxAngle, 90 - angle));
                io.setPositionDegrees(desiredAngle);
        }

        public double getPosition() {
                return inputs.positionRotations;
        }

        public double getPositionDegrees() {
                return inputs.positionDegrees;
        }

        public static double findoptimalangle(final double distance) {
                double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
                return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
        }

        public void resetSafe() {
                setPosition(90);
        }

        public void resetEncoder() {
                io.resetPosition(0.0);
        }

        public void set(double speed) {
                io.setVoltage(speed * 12.0);
        }

        public void stop() {
                io.stop();
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs("Hood", inputs);

                SmartDashboard.putNumber("Hood/Desired Angle", desiredAngle);
                SmartDashboard.putNumber("Hood/Position", getPosition());
                SmartDashboard.putNumber("Hood/PositionDegrees", inputs.positionDegrees);
                SmartDashboard.putNumber("Hood/Stator Current", inputs.statorCurrentAmps);
                SmartDashboard.putNumber("Hood/Supply Current", inputs.supplyCurrentAmps);
                SmartDashboard.putNumber("Hood/Motor Voltage", inputs.appliedVolts);
                SmartDashboard.putNumber("Hood/Velocity", inputs.velocityDegreesPerSec);
                SmartDashboard.putBoolean("Hood/AtDesiredAngle", atDesiredAngle());
        }

        public boolean atDesiredAngle() {
                return Math.abs(getPosition() - (desiredAngle / 360.0))
                    < Constants.Hood.kHoodTolerance;
        }

        public static double calculateLaunchAngle(
            double distanceMeters, double exitVelocityMps, boolean highArc) {
                double deltaHeightMeters = Units.inchesToMeters(Constants.Hood.ScoreHeight);
                return calculateLaunchAngle(distanceMeters, deltaHeightMeters, exitVelocityMps, highArc);
        }

        public static double calculateLaunchAngle(double distanceMeters, double deltaHeightMeters,
            double exitVelocityMps, boolean highArc) {
                double v2 = exitVelocityMps * exitVelocityMps;
                double v4 = v2 * v2;
                double x = distanceMeters;
                double y = deltaHeightMeters;

                double discriminant = v4
                    - Constants.Shooter.kGRAVITATIONAL_CONSTANT
                        * (Constants.Shooter.kGRAVITATIONAL_CONSTANT * x * x + 2 * y * v2);
                if (discriminant < 0) {
                        return 0;
                }

                double sqrtDisc = Math.sqrt(discriminant);
                double sign = highArc ? 1.0 : -1.0;
                double tanTheta =
                    (v2 + (sign * sqrtDisc)) / (Constants.Shooter.kGRAVITATIONAL_CONSTANT * x);

                return Math.atan(tanTheta);
        }
}
