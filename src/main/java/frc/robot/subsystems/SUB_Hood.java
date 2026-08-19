package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

/**
 * Subsystem controlling the adjustable shooter hood angle mechanism.
 *
 * <p>Hardware: Single CTRE TalonFX motor on CAN ID 47 ({@link Constants.Hood#kHOOD_CAN_ID})
 * with stator current limits (25A) and supply current limits (7A/5A).
 */
public class SUB_Hood extends SubsystemBase {
        private static SUB_Hood INSTANCE = null;
        private final TalonFX hood;

        private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0).withSlot(0);

        /**
         * Singleton pattern provider for the hood subsystem.
         *
         * @return The single instance of {@link SUB_Hood}.
         */
        public static SUB_Hood getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Hood();
                }
                return INSTANCE;
        }

        /**
         * Private constructor initializing the TalonFX motor controller with current limits.
         */
        private SUB_Hood() {
                final TalonFXConfiguration config =
                    new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(25)
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(7)
                            .withSupplyCurrentLowerLimit(5)
                            .withSupplyCurrentLowerTime(.5));
                config.Slot0
                        .withKS(0)
                        .withKV(0)
                        .withKA(1)
                        .withKP(1)
                        .withKI(0)
                        .withKD(0);
                hood = new TalonFX(Constants.Hood.kHOOD_CAN_ID);
                hood.getConfigurator().apply(config);
        }

        /**
         * Drives the hood motor to a specified target position using PID control.
         *
         * @param angle Target position in motor rotations.
         */
        public void setPosition(final double angle) {
                hood.setControl(positionRequest.withPosition(Degrees.of(angle)));
        }

        /**
         * Returns the current hood motor position in rotations.
         *
         * @return Current position in motor rotations.
         */
        public double getPosition() {
                return hood.getPosition().getValueAsDouble();
        }

        /**
         * Calculates optimal hood angle in radians based on target distance and target height.
         *
         * @param distance Horizontal distance to target in meters.
         * @return Calculated optimal launch angle in radians.
         */
        public static double findoptimalangle(final double distance) {
                double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
                return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
        }

        /**
         * Resets the hood towards position 0 safely.
         */
        public void resetSafe() {
                setPosition(0);
        }

        /**
         * Resets the motor encoder position reading to zero.
         */
        public void resetEncoder() {
                hood.setPosition(0);
        }

        /**
         * Sets raw duty cycle power output to the hood motor (-1.0 to 1.0).
         *
         * @param speed Duty cycle speed percentage.
         */
        public void set(double speed) {
                hood.set(speed);
        }

        /**
         * Subsystem periodic loop (20ms). Telemeters position, current, voltage, temperature,
         * and velocity to SmartDashboard, and checks Kraken motor health status via Alert class.
         */
        @Override
        public void periodic() {
                SmartDashboard.putNumber("Hood/Position", getPosition());
                SmartDashboard.putNumber(
                    "Hood/Stator Current", hood.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber(
                    "Hood/Supply Current", hood.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber(
                    "Hood/Supply Voltage", hood.getSupplyVoltage().getValueAsDouble());
                SmartDashboard.putNumber(
                    "Hood/Motor Voltage", hood.getMotorVoltage().getValueAsDouble());
                SmartDashboard.putNumber(
                    "Hood/Torque Current", hood.getTorqueCurrent().getValueAsDouble());
                SmartDashboard.putNumber(
                    "Hood/Device Temp", hood.getDeviceTemp().getValueAsDouble());
                SmartDashboard.putNumber(
                    "Hood/Processor Temp", hood.getProcessorTemp().getValueAsDouble());
                SmartDashboard.putNumber("Hood/Velocity", hood.getVelocity().getValueAsDouble());
                Alert.alertKraken(hood);
        }

        public boolean atDesiredAngle() {
                return Math.abs(getPosition() - Constants.Hood.kHOOD_PID_CONTROLLER.getSetpoint()) < 0.05;
        }
}
