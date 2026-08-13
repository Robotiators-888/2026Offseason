package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private static SUB_Hood INSTANCE = null;
    private final TalonFX hood;
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    /** Last angle commanded, in mechanism radians, for telemetry and atAngle(). */
    private double desiredAngleRads = Constants.Hood.kHOOD_ANGLE_AT_ZERO_RADS;

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
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            angleToRotations(Constants.Hood.kHOOD_MAX_ANGLE_RADS);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
            angleToRotations(Constants.Hood.kHOOD_MIN_ANGLE_RADS);

        hood = new TalonFX(Constants.Hood.kHOOD_CAN_ID);
        hood.getConfigurator().apply(config);
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
     * <p>This is what {@code resetSafe()} always claimed to be. Run it once on startup, or bind it
     * to a button — it is the only thing that makes the encoder's zero mean anything.
     */
    public Command homeCommand () {
        final Debouncer stallDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);
        return startEnd(this::driveTowardStop, this::stop)
            .beforeStarting(() -> stallDebouncer.calculate(false))
            .until(() -> stallDebouncer.calculate(isStalled()))
            .withTimeout(Constants.Hood.kHOOD_HOMING_TIMEOUT_SECONDS)
            .andThen(runOnce(this::resetEncoder))
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

    @Override
    public void periodic () {
        SmartDashboard.putNumber("Hood/Angle (Deg)", Units.radiansToDegrees(getAngle()));
        SmartDashboard.putNumber("Hood/Desired Angle (Deg)", Units.radiansToDegrees(desiredAngleRads));
        SmartDashboard.putBoolean("Hood/At Angle", atAngle());
        SmartDashboard.putNumber("Hood/Position (Rotations)", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Stator Current", hood.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Supply Current", hood.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Supply Voltage", hood.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Motor Voltage", hood.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Torque Current", hood.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Device Temp", hood.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Processor Temp", hood.getProcessorTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Velocity", hood.getVelocity().getValueAsDouble());
        Alert.alertKraken(hood);
    }
}
