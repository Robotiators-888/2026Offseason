package frc.robot.subsystems.linear;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit hardware abstraction interface for the linear intake deploy mechanism.
 */
public interface LinearIO {
        @AutoLog
        public static class LinearIOInputs {
                public double positionRotations = 0.0;
                public double velocityRPM = 0.0;
                public double appliedVolts = 0.0;
                public double currentAmps = 0.0;
                public double busVolts = 0.0;
                public double tempCelsius = 0.0;
        }

        default void updateInputs(LinearIOInputs inputs) {}
        default void setVoltage(double volts) {}
        default void setDutyCycle(double speed) {}
        default void stop() {}
}
