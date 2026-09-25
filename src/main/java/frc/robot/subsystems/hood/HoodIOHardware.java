package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.utils.PhoenixUtil;

/**
 * Physical hardware implementation of HoodIO using a CTRE TalonFX motor controller.
 */
public class HoodIOHardware implements HoodIO {
        private final TalonFX hood = new TalonFX(Constants.Hood.kHOOD_CAN_ID);
        private final PositionTorqueCurrentFOC positionRequest =
            new PositionTorqueCurrentFOC(0).withSlot(0);
        private final VoltageOut voltageRequest = new VoltageOut(0);

        public HoodIOHardware() {
                final TalonFXConfiguration config =
                    new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(Constants.Hood.kStatorCurrentLimit)
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(Constants.Hood.kSupplyCurrentLimit)
                            .withSupplyCurrentLowerLimit(Constants.Hood.kSupplyCurrentLowerLimit)
                            .withSupplyCurrentLowerTime(Constants.Hood.kSupplyCurrentLowerTime));
                config.Slot0.withKS(Constants.Hood.kS)
                    .withKV(Constants.Hood.kV)
                    .withKA(Constants.Hood.kA)
                    .withKP(Constants.Hood.kP)
                    .withKI(Constants.Hood.kI)
                    .withKD(Constants.Hood.kD)
                    .withKG(Constants.Hood.kG);
                config.Feedback.SensorToMechanismRatio = Constants.Hood.kGearRatio;

                PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(config));
        }

        @Override
        public void updateInputs(HoodIOInputs inputs) {
                inputs.positionRotations = hood.getPosition().getValueAsDouble();
                inputs.positionDegrees = hood.getPosition().getValue().in(Degrees);
                inputs.velocityDegreesPerSec = hood.getVelocity().getValue().in(DegreesPerSecond);
                inputs.appliedVolts = hood.getMotorVoltage().getValueAsDouble();
                inputs.supplyCurrentAmps = hood.getSupplyCurrent().getValueAsDouble();
                inputs.statorCurrentAmps = hood.getStatorCurrent().getValueAsDouble();
                inputs.tempCelsius = hood.getDeviceTemp().getValueAsDouble();
        }

        @Override
        public void setPositionDegrees(double degrees) {
                hood.setControl(positionRequest.withPosition(Degrees.of(degrees)));
        }

        @Override
        public void setVoltage(double volts) {
                hood.setControl(voltageRequest.withOutput(volts));
        }

        @Override
        public void resetPosition(double positionRotations) {
                hood.setPosition(positionRotations);
        }

        @Override
        public void stop() {
                hood.setControl(voltageRequest.withOutput(0));
        }
}
