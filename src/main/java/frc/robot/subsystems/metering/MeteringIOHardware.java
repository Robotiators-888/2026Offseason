package frc.robot.subsystems.metering;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.utils.Alert;
import frc.robot.utils.PhoenixUtil;

/**
 * Physical hardware implementation of MeteringIO using dual CTRE TalonFX motors.
 */
public class MeteringIOHardware implements MeteringIO {
        private final TalonFX leader = new TalonFX(Constants.Metering.kMETERING_MOTOR_CAN_ID);
        private final TalonFX follower = new TalonFX(Constants.Metering.kMETERING_MOTOR_FOLLOWER_CAN_ID);

        private final VelocityTorqueCurrentFOC velocityRequest =
            new VelocityTorqueCurrentFOC(0).withSlot(0);
        private final CoastOut coastRequest = new CoastOut();
        private final VoltageOut voltageRequest = new VoltageOut(0);

        public MeteringIOHardware() {
                final TalonFXConfiguration config = new TalonFXConfiguration().withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Constants.Metering.kStatorCurrentLimit)
                        .withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Constants.Metering.kSupplyCurrentLimit)
                        .withSupplyCurrentLowerLimit(Constants.Metering.kSupplyCurrentLowerLimit)
                        .withSupplyCurrentLowerTime(Constants.Metering.kSupplyCurrentLowerTime));
                config.Slot0.withKS(Constants.Metering.kS)
                    .withKV(Constants.Metering.kV)
                    .withKA(Constants.Metering.kA)
                    .withKP(Constants.Metering.kP)
                    .withKI(Constants.Metering.kI)
                    .withKD(Constants.Metering.kD);
                config.Feedback.SensorToMechanismRatio = Constants.Metering.kGearRatio;

                PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config));
                PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));

                leader.getTorqueCurrent().setUpdateFrequency(Hertz.of(100));
                follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
        }

        @Override
        public void updateInputs(MeteringIOInputs inputs) {
                inputs.leaderVelocityRPM = leader.getVelocity().getValue().in(RPM);
                inputs.followerVelocityRPM = follower.getVelocity().getValue().in(RPM);
                inputs.leaderPositionRotations = leader.getPosition().getValueAsDouble();
                inputs.followerPositionRotations = follower.getPosition().getValueAsDouble();
                inputs.leaderAppliedVolts = leader.getMotorVoltage().getValueAsDouble();
                inputs.followerAppliedVolts = follower.getMotorVoltage().getValueAsDouble();
                inputs.leaderSupplyCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
                inputs.followerSupplyCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
                inputs.leaderStatorCurrentAmps = leader.getStatorCurrent().getValueAsDouble();
                inputs.followerStatorCurrentAmps = follower.getStatorCurrent().getValueAsDouble();
                inputs.leaderTempCelsius = leader.getDeviceTemp().getValueAsDouble();
                inputs.followerTempCelsius = follower.getDeviceTemp().getValueAsDouble();

                Alert.alertKraken(leader);
                Alert.alertKraken(follower);
        }

        @Override
        public void setRPM(double rpm) {
                leader.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }

        @Override
        public void setVoltage(double volts) {
                leader.setControl(voltageRequest.withOutput(volts));
        }

        @Override
        public void stop() {
                leader.setControl(coastRequest);
        }
}
