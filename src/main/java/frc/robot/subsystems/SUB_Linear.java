package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

/**
 * Subsystem controlling linear intake deployment.
 *
 * <p>Hardware: Single REV SPARK Max motor controller driving a 23:1 cycloidal gearbox
 * on CAN ID 31 ({@link Constants.Linear#kLINEAR_MOTOR_CANID}) with a 35A smart current limit.
 */
public class SUB_Linear extends SubsystemBase {
        /** Subsystem extension state flag. */
        public static boolean extended;

        /** SPARK Max motor controller driving linear deploy mechanism. */
        // 16:1 gear ratio
        private final SparkMax linear;

        private static SUB_Linear INSTANCE = null;

        /**
         * Singleton provider for the linear intake deploy subsystem.
         *
         * @return Single instance of {@link SUB_Linear}.
         */
        public static SUB_Linear getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Linear();
                }
                return INSTANCE;
        }

        /**
         * Private constructor initializing SPARK Max controller and applying hardware settings.
         */
        private SUB_Linear() {
                // Defines motors with IDs and motor type
                linear = new SparkMax(Constants.Linear.kLINEAR_MOTOR_CANID, MotorType.kBrushless);
                configureMotors();
        }

        /**
         * Configures current limits (35A stall, 5A free) and motor direction.
         */
        @SuppressWarnings("removal")
        private void configureMotors() {
                // Creates config for motor and encoder (23:1 cycloidal gearbox)
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit(35, 5); // Sets stall limit in amps
                config.inverted(true);
                linear.configure(config, SparkMax.ResetMode.kResetSafeParameters,
                    SparkMax.PersistMode.kPersistParameters);
        }

        /**
         * Subsystem periodic loop (20ms). Telemeters encoder position, current, bus voltage,
         * and motor temperature to SmartDashboard, checking for REV hardware faults.
         */
        @Override
        public void periodic() {
                // Telemetry logging for dashboard
                SmartDashboard.putNumber(
                    "Linear/Linear Encoder Pos", linear.getEncoder().getPosition());
                SmartDashboard.putNumber("Linear/Linear Output Current", linear.getOutputCurrent());
                SmartDashboard.putNumber("Linear/Linear Bus Voltage", linear.getBusVoltage());
                SmartDashboard.putNumber("Linear/Linear Motor Temp", linear.getMotorTemperature());

                Alert.alertNeoFaults(linear);
                Alert.alertNeoWarnings(linear);
        }

        /**
         * Returns whether the linear mechanism is extended.
         *
         * @return True if extended, false otherwise.
         */
        public boolean isExtended() {
                return extended;
        }

        /**
         * Drives linear mechanism toward extended forward setpoint using specified PIDController.
         *
         * @param controller Positional PIDController for motion calculations.
         */
        public void forward(final PIDController controller) {
                linear.set(controller.calculate(
                    linear.getEncoder().getPosition(), Constants.Linear.kLINEAR_FORWARD_SETPOINT));
        }

        /**
         * Drives linear mechanism toward retracted backward setpoint using specified PIDController.
         *
         * @param controller Positional PIDController for motion calculations.
         */
        public void backward(final PIDController controller) {
                linear.set(controller.calculate(
                    linear.getEncoder().getPosition(), Constants.Linear.kLINEAR_BACKWARD_SETPOINT));
        }

        /**
         * Checks whether the linear position is within tolerance (3 degrees) of forward setpoint.
         *
         * @return True if at forward setpoint position, false otherwise.
         */
        public boolean isForward() {
                return Math.abs(linear.getEncoder().getPosition()
                           - Constants.Linear.kLINEAR_FORWARD_SETPOINT)
                    < 3.0;
        }

        /**
         * Checks whether the linear position is within tolerance (3 degrees) of backward setpoint.
         *
         * @return True if at backward setpoint position, false otherwise.
         */
        public boolean isBackward() {
                return Math.abs(linear.getEncoder().getPosition()
                           - Constants.Linear.kLINEAR_BACKWARD_SETPOINT)
                    < 3.0;
        }

        /**
         * Sets open-loop percent output speed to linear motor (-1.0 to 1.0).
         *
         * @param speed Percent output duty cycle.
         */
        public void set(double speed) {
                linear.set(speed);
        }
}
