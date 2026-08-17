package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

/**
 * Subsystem controlling the ground intake roller mechanism.
 *
 * <p>Hardware: Dual CTRE TalonFX motors on CAN IDs 30 and 31 ({@link Constants.Roller#kINTAKE_LEFTMOTOR_CANID}
 * and {@link Constants.Roller#kINTAKE_RIGHTMOTOR_CANID}) in opposed leader-follower configuration with FOC enabled.
 */
public class SUB_Roller extends SubsystemBase {
        /** Left roller TalonFX motor controller (Leader). */
        private final TalonFX LeftRollerMotor;

        /** Right roller TalonFX motor controller (Follower). */
        private final TalonFX RightRollerMotor;

        /** Voltage output control request object with Field Oriented Control (FOC) enabled. */
        private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

        /** Duty cycle percent output control request object with Field Oriented Control (FOC) enabled. */
        private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);

        private static SUB_Roller INSTANCE = null;

        /**
         * Singleton pattern provider for the roller subsystem.
         *
         * @return Single instance of {@link SUB_Roller}.
         */
        public static SUB_Roller getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Roller();
                }
                return INSTANCE;
        }

        /**
         * Private constructor initializing TalonFX motors and applying configuration.
         */
        private SUB_Roller() {
                // Defines motor with ID from Constants
                LeftRollerMotor = new TalonFX(Constants.Roller.kINTAKE_LEFTMOTOR_CANID);
                RightRollerMotor = new TalonFX(Constants.Roller.kINTAKE_RIGHTMOTOR_CANID);
                configureMotors();
        }

        /**
         * Configures current limits (30A supply limit, 15A lower limit) and sets follower mode.
         */
        private void configureMotors() {
                // Configure TalonFX motor controller with current limits and inversion
                TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                talonConfig.CurrentLimits.SupplyCurrentLimit = 30;
                talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 15;
                talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
                talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                LeftRollerMotor.getConfigurator().apply(talonConfig);
                RightRollerMotor.getConfigurator().apply(talonConfig);

                RightRollerMotor.setControl(
                    new Follower(LeftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        /**
         * Sets target output voltage for the roller motors.
         *
         * @param speed Target output in volts.
         */
        public void setVolts(double speed) {
                LeftRollerMotor.setControl(voltageRequest.withOutput(speed));
        }

        /**
         * Sets target duty cycle percent output for the roller motor (-1.0 to 1.0).
         *
         * @param speed Target percent duty cycle.
         */
        public void set(double speed) {
                LeftRollerMotor.setControl(dutyCycleRequest.withOutput(speed));
        }

        /**
         * Calculates average velocity of left and right roller motors in RPM.
         *
         * @return Average rotational velocity in RPM.
         */
        public double rollerRPM() {
                return (LeftRollerMotor.getVelocity().getValue().baseUnitMagnitude()
                           + RightRollerMotor.getVelocity().getValue().baseUnitMagnitude())
                    / 2;
        }

        /**
         * Subsystem periodic loop (20ms). Telemeters roller RPM, positions, current draws, voltages,
         * and device temperatures for both Kraken motors to SmartDashboard, monitoring motor health.
         */
        @Override
        public void periodic() {
                // Telemetry logging for dashboard
                SmartDashboard.putNumber("Roller/Roller Average RPM", rollerRPM());
                SmartDashboard.putNumber("Roller/Left Roller Encoder Pos",
                    LeftRollerMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Encoder Pos",
                    RightRollerMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Left Roller Stator Current",
                    LeftRollerMotor.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Stator Current",
                    RightRollerMotor.getStatorCurrent().getValueAsDouble());

                SmartDashboard.putNumber("Roller/Left Roller Supply Current",
                    LeftRollerMotor.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Supply Current",
                    RightRollerMotor.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Left Roller Torque Current",
                    LeftRollerMotor.getTorqueCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Torque Current",
                    RightRollerMotor.getTorqueCurrent().getValueAsDouble());

                SmartDashboard.putNumber("Roller/Left Roller Supply Voltage",
                    LeftRollerMotor.getSupplyVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Supply Voltage",
                    RightRollerMotor.getSupplyVoltage().getValueAsDouble());

                SmartDashboard.putNumber("Roller/Left Roller Motor Voltage",
                    LeftRollerMotor.getMotorVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Motor Voltage",
                    RightRollerMotor.getMotorVoltage().getValueAsDouble());

                SmartDashboard.putNumber("Roller/Left Roller Device Temp",
                    LeftRollerMotor.getDeviceTemp().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Device Temp",
                    RightRollerMotor.getDeviceTemp().getValueAsDouble());

                SmartDashboard.putNumber("Roller/Left Roller Processor Temp",
                    LeftRollerMotor.getProcessorTemp().getValueAsDouble());
                SmartDashboard.putNumber("Roller/Right Roller Processor Temp",
                    RightRollerMotor.getProcessorTemp().getValueAsDouble());

                Alert.alertKraken(LeftRollerMotor);
                Alert.alertKraken(RightRollerMotor);
        }
}
