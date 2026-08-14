package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.OptionalDouble;
import frc.robot.Constants;
import frc.robot.utils.Alert;

/**
 * Shooter hood.
 *
 * <p>Every angle crossing this class's boundary is a <b>mechanism angle in radians</b>. The TalonFX
 * is configured with a SensorToMechanismRatio so its integrated encoder reports mechanism rotations,
 * and the conversion to radians happens here and nowhere else. Callers never see rotations.
 */
public class SUB_Hood extends SubsystemBase {
    /**
     * Slack on the trajectory quadratic's discriminant, in meters squared. Absorbs rounding at the
     * maximum-range boundary where the two arcs coincide.
     */
    private static final double kDiscriminantTolerance = 1e-9;

    private static SUB_Hood INSTANCE = null;
    private final TalonFX hood;
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    /** Last angle commanded, in mechanism radians, for telemetry and atAngle(). */
    private double desiredAngleRads = Constants.Hood.kHOOD_ANGLE_AT_ZERO_RADS;

    /**
     * The normal soft limits, and a homing variant with the reverse limit dropped. The retracted
     * hard stop sits AT the reverse soft limit (encoder zero), so a homing crawl with the limit
     * enabled is halted by the controller before it can ever reach the stop — the exact failure
     * the homing routine used to time out on, every time.
     */
    private final SoftwareLimitSwitchConfigs normalSoftLimits = new SoftwareLimitSwitchConfigs();
    private final SoftwareLimitSwitchConfigs homingSoftLimits = new SoftwareLimitSwitchConfigs();

    public static SUB_Hood getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Hood();
        }
        return INSTANCE;
    }

    private SUB_Hood () {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(25)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(7)
                .withSupplyCurrentLowerLimit(5)
                .withSupplyCurrentLowerTime(.5));

        // Report mechanism rotations rather than motor rotations.
        config.Feedback.SensorToMechanismRatio = Constants.Hood.kHOOD_GEAR_RATIO;

        // Position loop runs on the motor controller, so a slow RIO loop cannot destabilise it.
        config.Slot0.kP = Constants.Hood.kHOOD_kP;
        config.Slot0.kI = Constants.Hood.kHOOD_kI;
        config.Slot0.kD = Constants.Hood.kHOOD_kD;

        // The hood has hard stops and no limit switch, so the soft limits are the only thing
        // standing between a bad setpoint and a stalled motor.
        normalSoftLimits.ForwardSoftLimitEnable = true;
        normalSoftLimits.ForwardSoftLimitThreshold =
            angleToRotations(Constants.Hood.kHOOD_MAX_ANGLE_RADS);
        normalSoftLimits.ReverseSoftLimitEnable = true;
        normalSoftLimits.ReverseSoftLimitThreshold =
            angleToRotations(Constants.Hood.kHOOD_MIN_ANGLE_RADS);

        homingSoftLimits.ForwardSoftLimitEnable = normalSoftLimits.ForwardSoftLimitEnable;
        homingSoftLimits.ForwardSoftLimitThreshold = normalSoftLimits.ForwardSoftLimitThreshold;
        homingSoftLimits.ReverseSoftLimitEnable = false;
        homingSoftLimits.ReverseSoftLimitThreshold = normalSoftLimits.ReverseSoftLimitThreshold;

        config.SoftwareLimitSwitch = normalSoftLimits;

        hood = new TalonFX(Constants.Hood.kHOOD_CAN_ID);
        hood.getConfigurator().apply(config);
    }

    /** Swaps the soft-limit profile without blocking; used only around the homing crawl. */
    private void setReverseSoftLimitEnabled (final boolean enabled) {
        hood.getConfigurator().apply(enabled ? normalSoftLimits : homingSoftLimits, 0.0);
    }

    /** Converts a mechanism angle in radians to the encoder's rotation reading. */
    private static double angleToRotations (final double angleRads) {
        return Units.radiansToRotations(angleRads - Constants.Hood.kHOOD_ANGLE_AT_ZERO_RADS);
    }

    /** Converts an encoder rotation reading to a mechanism angle in radians. */
    private static double rotationsToAngle (final double rotations) {
        return Units.rotationsToRadians(rotations) + Constants.Hood.kHOOD_ANGLE_AT_ZERO_RADS;
    }

    /**
     * Drives the hood to a mechanism angle, clamped to the hood's travel.
     *
     * @param angleRads Target mechanism angle in radians
     */
    public void setAngle (final double angleRads) {
        desiredAngleRads = MathUtil.clamp(
            angleRads, Constants.Hood.kHOOD_MIN_ANGLE_RADS, Constants.Hood.kHOOD_MAX_ANGLE_RADS);
        hood.setControl(positionRequest.withPosition(angleToRotations(desiredAngleRads)));
    }

    /** @return Current mechanism angle in radians */
    public double getAngle () {
        return rotationsToAngle(hood.getPosition().getValueAsDouble());
    }

    /** @return true if the hood is within a degree of its commanded angle */
    public boolean atAngle () {
        return Math.abs(getAngle() - desiredAngleRads) < Units.degreesToRadians(1.0);
    }

    /**
     * Optimal launch angle for a given range: the classic minimum-launch-speed solution,
     * {@code pi/4 + atan(h/d)/2}, which also guarantees {@code d*tan(theta) > h} so the companion
     * velocity solve in {@link SUB_Shooter#findoptimalRPM} never takes a root of a negative number.
     *
     * @param distance Horizontal range to the target, in meters
     * @return Launch angle in radians
     */
    public static double findoptimalangle(final double distance) {
        double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
    }

    /**
     * Hood angle that lands a shot at {@code distance} when the flywheel is held at
     * {@code launchSpeed}, in m/s.
     *
     * <p>This is the inverse of the usual solve. Substituting {@code sec^2 = 1 + tan^2} into the
     * trajectory equation turns it into a quadratic in {@code tan(theta)}:
     *
     * <pre>k*tan^2(theta) - d*tan(theta) + (h + k) = 0,  where k = g*d^2 / (2*v^2)</pre>
     *
     * <p>Its two roots are the flat and lobbed arcs that both reach the same point. The steeper one
     * is returned: it drops the ball into the goal at a better entry angle, and it continues the
     * hood in the direction it already sweeps as range closes. The roots merge at the maximum range
     * the speed can reach, beyond which the discriminant goes negative and there is no solution.
     *
     * @param distance Horizontal range in meters
     * @param launchSpeed Ball exit speed in m/s
     * @return Launch angle in radians, or empty if the speed cannot reach that range
     */
    public static OptionalDouble angleForLaunchSpeed (final double distance, final double launchSpeed) {
        final double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        final double k = (Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance)
            / (2 * launchSpeed * launchSpeed);

        double discriminant = distance * distance - 4 * k * (height + k);

        // At exactly the maximum range the two arcs merge and the discriminant is mathematically
        // zero, so rounding can land it just either side. Treat a hair below zero as the grazing
        // case rather than declaring the shot impossible.
        if (discriminant < -kDiscriminantTolerance) {
            return OptionalDouble.empty();
        }
        discriminant = Math.max(discriminant, 0);

        final double steeperRoot = (distance + Math.sqrt(discriminant)) / (2 * k);
        return OptionalDouble.of(Math.atan(steeperRoot));
    }

    /**
     * Hood angle for a shot at {@code distance} with the flywheel held at {@code rpm}.
     *
     * <p>The RPM is converted to a launch speed by scaling the minimum-energy speed for this range
     * by how far the commanded RPM exceeds the tuned table value. Ratio, not absolute: the
     * absolute RPM-to-speed transfer is unreliable, but ratios taken near a calibrated point are
     * sound.
     *
     * @return Launch angle in radians, or empty if that RPM cannot reach that range
     */
    public static OptionalDouble angleForRPM (final double distance, final double rpm, final double tunedRpm) {
        if (tunedRpm <= 0 || rpm < tunedRpm) {
            // Below the tuned value there is no angle at all — the table already sits at the
            // minimum-energy angle, which is the cheapest way to reach this range.
            return OptionalDouble.empty();
        }
        final double launchSpeed = SUB_Shooter.minLaunchSpeed(distance) * (rpm / tunedRpm);
        return angleForLaunchSpeed(distance, launchSpeed);
    }

    /** Retracts the hood to its stowed angle using the position loop and soft limits. */
    public void stow () {
        setAngle(Constants.Hood.kHOOD_MIN_ANGLE_RADS);
    }

    /** @return Stator current, used by the homing routine to detect the hard stop */
    public double getStatorCurrent () {
        return hood.getStatorCurrent().getValueAsDouble();
    }

    /** @return true if the hood is pushing hard enough to be sitting on its hard stop */
    public boolean isStalled () {
        return Math.abs(getStatorCurrent()) > Constants.Hood.kHOOD_STALL_AMPS;
    }

    /** Drives the hood open-loop toward its retracted stop. Used only by the homing command. */
    public void driveTowardStop () {
        hood.set(Constants.Hood.kHOOD_HOMING_OUTPUT);
    }

    /**
     * Genuinely safe homing: crawl toward the retracted stop at a low open-loop output, watch the
     * stator current, and zero the encoder once the hood has been pushing against the stop for long
     * enough to rule out a startup spike. Bounded by a timeout so a dead current signal cannot leave
     * the motor stalled indefinitely.
     *
     * <p>The reverse soft limit is dropped for the duration of the crawl — the retracted stop sits
     * at the soft limit, so leaving it enabled halts the crawl at encoder zero before the stop and
     * the routine can only ever time out. And a timeout must NOT re-zero: whatever position the
     * hood happened to be sitting at would silently become the new zero, which is worse than no
     * home at all.
     *
     * <p>Run it once on startup, or bind it to a button — it is the only thing that makes the
     * encoder's zero mean anything.
     */
    public Command homeCommand () {
        final Debouncer stallDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);
        final boolean[] stalled = {false};
        return startEnd(this::driveTowardStop, this::stop)
            .beforeStarting(() -> {
                stalled[0] = false;
                stallDebouncer.calculate(false);
                setReverseSoftLimitEnabled(false);
            })
            .until(() -> stalled[0] = stallDebouncer.calculate(isStalled()))
            .withTimeout(Constants.Hood.kHOOD_HOMING_TIMEOUT_SECONDS)
            .finallyDo(() -> setReverseSoftLimitEnabled(true))
            .andThen(Commands.either(
                runOnce(this::resetEncoder),
                runOnce(() -> Alert.registerWarning("Hood homing timed out — encoder NOT zeroed")),
                () -> stalled[0]))
            .withName("HomeHood");
    }

    public void stop () {
        hood.stopMotor();
    }

    /** Declares the current position to be the retracted hard stop. */
    public void resetEncoder () {
        hood.setPosition(angleToRotations(Constants.Hood.kHOOD_ANGLE_AT_ZERO_RADS));
        desiredAngleRads = Constants.Hood.kHOOD_ANGLE_AT_ZERO_RADS;
    }

    public void set (double speed) {
        hood.set(speed);
    }

    /** Loop counter used to decimate the diagnostic telemetry below to ~4 Hz. */
    private int telemetryTick = 0;

    @Override
    public void periodic () {
        // Fire-gate signals at full rate; slow diagnostics at ~4 Hz.
        SmartDashboard.putNumber("Hood/Angle (Deg)", Units.radiansToDegrees(getAngle()));
        SmartDashboard.putNumber("Hood/Desired Angle (Deg)", Units.radiansToDegrees(desiredAngleRads));
        SmartDashboard.putBoolean("Hood/At Angle", atAngle());
        SmartDashboard.putNumber("Hood/Position (Rotations)", hood.getPosition().getValueAsDouble());
        // Stator current at full rate too — it is the homing routine's stall signal.
        SmartDashboard.putNumber("Hood/Stator Current", hood.getStatorCurrent().getValueAsDouble());
        if (telemetryTick++ % 12 == 0) {
            SmartDashboard.putNumber("Hood/Supply Current", hood.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Hood/Supply Voltage", hood.getSupplyVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Hood/Motor Voltage", hood.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Hood/Torque Current", hood.getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Hood/Device Temp", hood.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Hood/Processor Temp", hood.getProcessorTemp().getValueAsDouble());
            SmartDashboard.putNumber("Hood/Velocity", hood.getVelocity().getValueAsDouble());
        }
        Alert.alertKraken(hood);
    }
}
