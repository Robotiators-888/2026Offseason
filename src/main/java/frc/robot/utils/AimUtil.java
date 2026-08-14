package frc.robot.utils;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Pure, stateless aiming math shared by the aim and shuttle commands. Everything here is a plain
 * function of its arguments so it can be unit tested without hardware.
 */
public final class AimUtil {
    private AimUtil() {}

    /**
     * Offset from the hub center to the shuttle aim point: back toward our own end of the field,
     * and toward the sideline the robot is currently closest to, so shuttled fuel piles up out of
     * traffic on our side.
     *
     * <p>The blue alliance wall is at X = 0, so "toward our own end" is −X for blue and +X for
     * red. The near sideline for a robot below the field centerline is the Y = 0 wall, i.e. a
     * negative Y offset.
     *
     * @param isRed true when on the red alliance
     * @param sidelineSign −1 to lob toward the Y = 0 sideline, +1 toward the far one; see
     *     {@link #sidelineSign}
     * @param offsetMeters magnitude of the offset along each axis
     */
    public static Translation2d shuttleOffset (final boolean isRed, final int sidelineSign,
            final double offsetMeters) {
        final double x = isRed ? offsetMeters : -offsetMeters;
        final double y = sidelineSign * offsetMeters;
        return new Translation2d(x, y);
    }

    /**
     * Which sideline to lob toward, held with hysteresis about the field centerline so a robot
     * hovering there does not throw the aim point two offsets across the field every loop.
     *
     * @param robotY robot Y position in meters
     * @param fieldWidth field width in meters
     * @param previousSign the last returned sign, or 0 on the first call to pick the nearest
     * @param hysteresisMeters how far past the centerline the robot must travel to flip
     * @return −1 for the Y = 0 sideline, +1 for the far sideline
     */
    public static int sidelineSign (final double robotY, final double fieldWidth,
            final int previousSign, final double hysteresisMeters) {
        final double half = fieldWidth / 2.0;
        if (previousSign == 0 || Math.abs(robotY - half) > hysteresisMeters) {
            return robotY < half ? -1 : 1;
        }
        return previousSign;
    }

    /**
     * Heading tolerance that scales with range: the angle subtended by the hub opening's
     * effective half-width at this distance, times a safety factor, clamped to sane bounds. A
     * flat tolerance lets the lateral miss grow linearly with range — a flat 5 degrees is
     * 0.47 m sideways at 5.4 m.
     *
     * @param distanceMeters range to the target
     * @param aimHalfWidthMeters effective half-width of the opening (opening half-width minus
     *     ball radius)
     * @param safetyFactor fraction of the geometric tolerance actually accepted
     * @return tolerance in radians, clamped to [2°, 10°]
     */
    public static double alignedToleranceRads (final double distanceMeters,
            final double aimHalfWidthMeters, final double safetyFactor) {
        return MathUtil.clamp(
            Math.atan(aimHalfWidthMeters / distanceMeters) * safetyFactor,
            Units.degreesToRadians(2), Units.degreesToRadians(10));
    }

    /**
     * Holds a setpoint until the newly computed one drifts far enough to matter, so a closed-loop
     * "at setpoint" check can actually latch instead of chasing measurement jitter.
     *
     * @return {@code currentSetpoint} while {@code requiredSetpoint} is within {@code deadband}
     *     of it, otherwise {@code requiredSetpoint}
     */
    public static double latchedSetpoint (final double currentSetpoint,
            final double requiredSetpoint, final double deadband) {
        return Math.abs(requiredSetpoint - currentSetpoint) <= deadband
            ? currentSetpoint
            : requiredSetpoint;
    }

    /**
     * Turns the heading controller's output into a rotational rate command.
     *
     * <p>The controller already produces radians per second, so it is clamped rather than scaled
     * by the max rate. Inside the heading tolerance the output is zeroed so the robot can settle.
     * The stiction feedforward ramps in over {@code rampWidthRads} past the tolerance instead of
     * switching on — the old step created a limit cycle at the boundary, with the robot slamming
     * to a stop and immediately kicking back out.
     *
     * @param omegaSpeed heading controller output, radians per second
     * @param thetaErrorRads absolute heading error
     * @param maxAngularRate output clamp, radians per second
     * @param headingTolRads error inside which the output is zero
     * @param stictionRadsPerSec full-strength stiction feedforward
     * @param rampWidthRads width over which the stiction term fades in past the tolerance
     */
    public static double rotationalRate (final double omegaSpeed, final double thetaErrorRads,
            final double maxAngularRate, final double headingTolRads,
            final double stictionRadsPerSec, final double rampWidthRads) {
        final double clamped = MathUtil.clamp(omegaSpeed, -maxAngularRate, maxAngularRate);
        if (thetaErrorRads <= headingTolRads) {
            return 0.0;
        }
        final double ramp = Math.min(1.0, (thetaErrorRads - headingTolRads) / rampWidthRads);
        return clamped + Math.copySign(stictionRadsPerSec * ramp, clamped);
    }

    /** Fixed-point convergence threshold for {@link #virtualTarget}, in meters. */
    private static final double kVirtualTargetToleranceMeters = 0.01;

    /** Iteration cap for {@link #virtualTarget} — the map stops contracting near max chassis speed. */
    private static final int kVirtualTargetMaxIterations = 10;

    /**
     * Motion-compensated aim point: the goal shifted opposite the robot's field-relative velocity
     * by the shot's time of flight, iterated to a fixed point because the TOF itself depends on
     * the distance to the shifted target. Each iteration only shrinks the error by roughly
     * {@code |v| * dTOF/dDist}, so this runs until the point stops moving (1 cm) rather than a
     * token iteration or two — at 2 m/s a fixed two iterations could still be half a meter off.
     * With zero velocity the result is the goal itself, immediately.
     *
     * @param goal the static target
     * @param shooterPosition field position of the shooter
     * @param vxMetersPerSec field-relative X velocity
     * @param vyMetersPerSec field-relative Y velocity
     * @param tofForDistance time-of-flight model: seconds as a function of range in meters
     */
    public static Translation2d virtualTarget (final Translation2d goal,
            final Translation2d shooterPosition, final double vxMetersPerSec,
            final double vyMetersPerSec, final DoubleUnaryOperator tofForDistance) {
        Translation2d virtual = goal;
        for (int i = 0; i < kVirtualTargetMaxIterations; i++) {
            final double tof = tofForDistance.applyAsDouble(shooterPosition.getDistance(virtual));
            final Translation2d next = new Translation2d(
                goal.getX() - vxMetersPerSec * tof,
                goal.getY() - vyMetersPerSec * tof);
            final double moved = next.getDistance(virtual);
            virtual = next;
            if (moved < kVirtualTargetToleranceMeters) {
                break;
            }
        }
        return virtual;
    }
}
