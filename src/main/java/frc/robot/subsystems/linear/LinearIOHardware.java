package frc.robot.subsystems.linear;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.utils.SparkUtil;

/**
 * Physical hardware implementation of LinearIO using REV SPARK MAX.
 */
public class LinearIOHardware implements LinearIO {
        private final SparkMax linear;

        public LinearIOHardware() {
                linear = new SparkMax(Constants.Linear.kLINEAR_MOTOR_CANID, MotorType.kBrushless);

                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit(Constants.Linear.kStallLimit, Constants.Linear.kFreeLimit);
                config.inverted(true);

                SparkUtil.tryUntilOk(linear, 5, () -> linear.configure(
                    config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
        }

        @Override
        public void updateInputs(LinearIOInputs inputs) {
                inputs.positionRotations = linear.getEncoder().getPosition();
                inputs.velocityRPM = linear.getEncoder().getVelocity();
                inputs.appliedVolts = linear.getAppliedOutput() * linear.getBusVoltage();
                inputs.currentAmps = linear.getOutputCurrent();
                inputs.busVolts = linear.getBusVoltage();
                inputs.tempCelsius = linear.getMotorTemperature();
        }

        @Override
        public void setVoltage(double volts) {
                linear.setVoltage(volts);
        }

        @Override
        public void setDutyCycle(double speed) {
                linear.set(speed);
        }

        @Override
        public void stop() {
                linear.set(0);
        }
}
