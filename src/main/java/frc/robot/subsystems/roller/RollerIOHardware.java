package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.utils.PhoenixUtil;

/**
 * Physical hardware implementation of RollerIO using dual TalonFX motors.
 */
public class RollerIOHardware implements RollerIO {
        private final TalonFX leftRollerMotor;
        private final TalonFX rightRollerMotor;

        private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
        private final VelocityTorqueCurrentFOC velocityRequest =
            new VelocityTorqueCurrentFOC(0).withSlot(0);
        private final CoastOut coast = new CoastOut();

        public RollerIOHardware() {
                leftRollerMotor = new TalonFX(Constants.Roller.kINTAKE_LEFTMOTOR_CANID);
                rightRollerMotor = new TalonFX(Constants.Roller.kINTAKE_RIGHTMOTOR_CANID);

                final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
                talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                talonConfig.CurrentLimits.SupplyCurrentLimit = Constants.Roller.kSupplyCurrentLimit;
                talonConfig.CurrentLimits.SupplyCurrentLowerLimit =
                    Constants.Roller.kSupplyCurrentLowerLimit;
                talonConfig.CurrentLimits.SupplyCurrentLowerTime =
                    Constants.Roller.kSupplyCurrentLowerTime;
                talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                talonConfig.Slot0.withKS(Constants.Roller.kS)
                    .withKV(Constants.Roller.kV)
                    .withKA(Constants.Roller.kA)
                    .withKP(Constants.Roller.kP)
                    .withKI(Constants.Roller.kI)
                    .withKD(Constants.Roller.kD);
                talonConfig.Feedback.SensorToMechanismRatio = Constants.Roller.kGearRatio;

                PhoenixUtil.tryUntilOk(5, () -> leftRollerMotor.getConfigurator().apply(talonConfig));
                PhoenixUtil.tryUntilOk(5, () -> rightRollerMotor.getConfigurator().apply(talonConfig));

                leftRollerMotor.getTorqueCurrent().setUpdateFrequency(Hertz.of(100));
                rightRollerMotor.setControl(
                    new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        @Override
        public void updateInputs(RollerIOInputs inputs) {
                inputs.leftVelocityRPM = leftRollerMotor.getVelocity().getValue().in(RPM);
                inputs.rightVelocityRPM = rightRollerMotor.getVelocity().getValue().in(RPM);

                inputs.leftAppliedVolts = leftRollerMotor.getMotorVoltage().getValueAsDouble();
                inputs.rightAppliedVolts = rightRollerMotor.getMotorVoltage().getValueAsDouble();

                inputs.leftSupplyCurrentAmps = leftRollerMotor.getSupplyCurrent().getValueAsDouble();
                inputs.rightSupplyCurrentAmps = rightRollerMotor.getSupplyCurrent().getValueAsDouble();

                inputs.leftStatorCurrentAmps = leftRollerMotor.getStatorCurrent().getValueAsDouble();
                inputs.rightStatorCurrentAmps = rightRollerMotor.getStatorCurrent().getValueAsDouble();

                inputs.leftTempCelsius = leftRollerMotor.getDeviceTemp().getValueAsDouble();
                inputs.rightTempCelsius = rightRollerMotor.getDeviceTemp().getValueAsDouble();
        }

        @Override
        public void setRPM(double rpm) {
                leftRollerMotor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        }

        @Override
        public void setVoltage(double volts) {
                leftRollerMotor.setControl(voltageRequest.withOutput(volts));
        }

        @Override
        public void stop() {
                leftRollerMotor.setControl(coast);
        }
}
