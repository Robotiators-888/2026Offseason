package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit hardware abstraction interface for the spindexer feed mechanism.
 */
public interface IndexIO {
        @AutoLog
        public static class IndexIOInputs {
                public double rightVelocityRPM = 0.0;
                public double leftVelocityRPM = 0.0;
                public double rightAppliedVolts = 0.0;
                public double leftAppliedVolts = 0.0;
                public double rightCurrentAmps = 0.0;
                public double leftCurrentAmps = 0.0;
                public double rightBusVolts = 0.0;
                public double leftBusVolts = 0.0;
                public double rightTempCelsius = 0.0;
                public double leftTempCelsius = 0.0;
        }

        default void updateInputs(IndexIOInputs inputs) {}
        default void setVoltage(double volts) {}
        default void setDutyCycle(double speed) {}
        default void stop() {}
}
