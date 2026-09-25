package frc.robot.subsystems.index;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.utils.SparkUtil;

/**
 * Physical hardware implementation of IndexIO using dual REV SPARK MAX controllers.
 */
public class IndexIOHardware implements IndexIO {
        private final SparkMax leftIndexer;
        private final SparkMax rightIndexer;

        public IndexIOHardware() {
                leftIndexer = new SparkMax(Constants.Index.kINDEX_MOTOR_CANID, MotorType.kBrushless);
                rightIndexer =
                    new SparkMax(Constants.Index.kINDEX_FOLLOWER_WHEEL_CANID, MotorType.kBrushless);

                SparkMaxConfig rightConfig = new SparkMaxConfig();
                rightConfig.smartCurrentLimit(Constants.Index.kSmartCurrentLimit);
                rightConfig.inverted(true);
                rightConfig.signals.appliedOutputPeriodMs(10);

                SparkMaxConfig leftConfig = new SparkMaxConfig();
                leftConfig.smartCurrentLimit(Constants.Index.kSmartCurrentLimit);
                leftConfig.follow(rightIndexer, true);

                SparkUtil.tryUntilOk(rightIndexer, 5, () -> rightIndexer.configure(
                    rightConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
                SparkUtil.tryUntilOk(leftIndexer, 5, () -> leftIndexer.configure(
                    leftConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters));
        }

        @Override
        public void updateInputs(IndexIOInputs inputs) {
                inputs.rightVelocityRPM = rightIndexer.getEncoder().getVelocity();
                inputs.leftVelocityRPM = leftIndexer.getEncoder().getVelocity();

                inputs.rightAppliedVolts = rightIndexer.getAppliedOutput() * rightIndexer.getBusVoltage();
                inputs.leftAppliedVolts = leftIndexer.getAppliedOutput() * leftIndexer.getBusVoltage();

                inputs.rightCurrentAmps = rightIndexer.getOutputCurrent();
                inputs.leftCurrentAmps = leftIndexer.getOutputCurrent();

                inputs.rightBusVolts = rightIndexer.getBusVoltage();
                inputs.leftBusVolts = leftIndexer.getBusVoltage();

                inputs.rightTempCelsius = rightIndexer.getMotorTemperature();
                inputs.leftTempCelsius = leftIndexer.getMotorTemperature();
        }

        @Override
        public void setVoltage(double volts) {
                rightIndexer.setVoltage(volts);
        }

        @Override
        public void setDutyCycle(double speed) {
                rightIndexer.set(speed);
        }

        @Override
        public void stop() {
                rightIndexer.set(0);
        }
}
