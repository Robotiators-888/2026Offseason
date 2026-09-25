package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.utils.PhoenixUtil;

/**
 * Physical hardware implementation of ShooterIO using CTRE Phoenix 6 TalonFX controllers.
 */
public class ShooterIOHardware implements ShooterIO {
        private final TalonFX shooterLeader;
        private final TalonFX shooterFollower;

        private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
        private final VoltageOut voltageRequest = new VoltageOut(0);

        public ShooterIOHardware() {
                shooterLeader = new TalonFX(Constants.Shooter.kSHOOTER_LEADER_MOTOR_CANID);
                shooterFollower = new TalonFX(Constants.Shooter.kSHOOTER_FOLLOWER_MOTOR_CANID);

                TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
                shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                shooterConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.kStatorCurrentLimit;
                shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                shooterConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.kSupplyCurrentLimit;
                shooterConfig.CurrentLimits.SupplyCurrentLowerLimit =
                    Constants.Shooter.kSupplyCurrentLowerLimit;
                shooterConfig.CurrentLimits.SupplyCurrentLowerTime =
                    Constants.Shooter.kSupplyCurrentLowerTime;
                shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
                shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV;
                shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
                shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP;
                shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
                shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD;
                shooterConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.kGearRatio;

                PhoenixUtil.tryUntilOk(5, () -> shooterLeader.getConfigurator().apply(shooterConfig));
                PhoenixUtil.tryUntilOk(5, () -> shooterFollower.getConfigurator().apply(shooterConfig));

                shooterLeader.getTorqueCurrent().setUpdateFrequency(Hertz.of(100));
                shooterFollower.setControl(
                    new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
        }

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
                inputs.leaderVelocityRPM = shooterLeader.getVelocity().getValue().in(RPM);
                inputs.followerVelocityRPM = shooterFollower.getVelocity().getValue().in(RPM);

                inputs.leaderAppliedVolts = shooterLeader.getMotorVoltage().getValueAsDouble();
                inputs.followerAppliedVolts = shooterFollower.getMotorVoltage().getValueAsDouble();

                inputs.leaderSupplyCurrentAmps = shooterLeader.getSupplyCurrent().getValueAsDouble();
                inputs.followerSupplyCurrentAmps = shooterFollower.getSupplyCurrent().getValueAsDouble();

                inputs.leaderStatorCurrentAmps = shooterLeader.getStatorCurrent().getValueAsDouble();
                inputs.followerStatorCurrentAmps = shooterFollower.getStatorCurrent().getValueAsDouble();

                inputs.leaderTempCelsius = shooterLeader.getDeviceTemp().getValueAsDouble();
                inputs.followerTempCelsius = shooterFollower.getDeviceTemp().getValueAsDouble();
        }

        @Override
        public void setVelocity(double targetRPM) {
                shooterLeader.setControl(velocityRequest.withVelocity(RPM.of(targetRPM)));
        }

        @Override
        public void setVoltage(double volts) {
                shooterLeader.setControl(voltageRequest.withOutput(volts));
        }

        @Override
        public void stop() {
                shooterLeader.setControl(voltageRequest.withOutput(0));
        }
}
