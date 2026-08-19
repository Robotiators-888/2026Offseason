package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

/**
 * Subsystem controlling the metering feed wheel system feeding game pieces into shooter flywheels.
 *
 * <p>Hardware: Dual CTRE TalonFX motors on CAN IDs 45 and 46 ({@link Constants.Metering#kMETERING_MOTOR_CAN_ID}
 * and {@link Constants.Metering#kMETERING_MOTOR_FOLLOWER_CAN_ID}) configured in leader-follower mode with stator
 * current limits (120A) and supply limits (60A/25A).
 */
public class SUB_Metering extends SubsystemBase {
        /** Leader TalonFX motor controller for metering wheel. */
        private final TalonFX metering;

        /** Follower TalonFX motor controller for metering wheel. */
        private final TalonFX meteringFollower;

        private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

        private static SUB_Metering INSTANCE = null;

        /**
         * Singleton pattern provider for the metering subsystem.
         *
         * @return Single instance of {@link SUB_Metering}.
         */
        public static SUB_Metering getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Metering();
                }
                return INSTANCE;
        }

        /**
         * Private constructor initializing leader and follower TalonFX motor controllers, applying current limits,
         * and establishing follower relationships.
         */
        private SUB_Metering() {
                metering = new TalonFX(Constants.Metering.kMETERING_MOTOR_CAN_ID);
                meteringFollower = new TalonFX(Constants.Metering.kMETERING_MOTOR_FOLLOWER_CAN_ID);
                final TalonFXConfiguration config =
                    new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(120)
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(60)
                            .withSupplyCurrentLowerLimit(25)
                            .withSupplyCurrentLowerTime(.5));
                config.Slot0
                        .withKS(0)
                        .withKV(0)
                        .withKA(1)
                        .withKP(1)
                        .withKI(0)
                        .withKD(0);
                metering.getConfigurator().apply(config);
                meteringFollower.getConfigurator().apply(config);
                meteringFollower.setControl(
                    new Follower(metering.getDeviceID(), MotorAlignmentValue.Aligned));
        }

        /**
         * Sets open-loop percent output speed for the metering leader motor (-1.0 to 1.0).
         *
         * @param speed Percent duty cycle power.
         */
        @Deprecated
        public void set(final double speed) {
                metering.set(speed);
        }

        public void setRPM (final double rpm) {
            metering.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }

        /**
         * Subsystem periodic loop (20ms). Telemeters position, current draw, supply/motor voltage, torque current,
         * device temperature, and velocity for both leader and follower Kraken motors to SmartDashboard.
         */
        @Override
        public void periodic() {
                SmartDashboard.putNumber(
                    "Metering/Leader Position", metering.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Position",
                    meteringFollower.getPosition().getValueAsDouble());

                SmartDashboard.putNumber("Metering/Leader Stator Current",
                    metering.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Stator Current",
                    meteringFollower.getStatorCurrent().getValueAsDouble());

                SmartDashboard.putNumber("Metering/Leader Supply Current",
                    metering.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Supply Current",
                    meteringFollower.getSupplyCurrent().getValueAsDouble());

                SmartDashboard.putNumber("Metering/Leader Supply Voltage",
                    metering.getSupplyVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Supply Voltage",
                    meteringFollower.getSupplyVoltage().getValueAsDouble());

                SmartDashboard.putNumber(
                    "Metering/Leader Motor Voltage", metering.getMotorVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Motor Voltage",
                    meteringFollower.getMotorVoltage().getValueAsDouble());

                SmartDashboard.putNumber("Metering/Leader Torque Current",
                    metering.getTorqueCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Torque Current",
                    meteringFollower.getTorqueCurrent().getValueAsDouble());

                SmartDashboard.putNumber(
                    "Metering/Leader Device Temp", metering.getDeviceTemp().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Device Temp",
                    meteringFollower.getDeviceTemp().getValueAsDouble());

                SmartDashboard.putNumber("Metering/Leader Processor Temp",
                    metering.getProcessorTemp().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Processor Temp",
                    meteringFollower.getProcessorTemp().getValueAsDouble());

                SmartDashboard.putNumber(
                    "Metering/Leader Velocity", metering.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("Metering/Follower Velocity",
                    meteringFollower.getVelocity().getValueAsDouble());

                Alert.alertKraken(metering);
                Alert.alertKraken(meteringFollower);
        }
}
