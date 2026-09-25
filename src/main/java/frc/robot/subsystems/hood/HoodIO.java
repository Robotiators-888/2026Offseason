package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit hardware abstraction interface for the adjustable hood subsystem.
 */
public interface HoodIO {
        @AutoLog
        public static class HoodIOInputs {
                public double positionRotations = 0.0;
                public double positionDegrees = 0.0;
                public double velocityDegreesPerSec = 0.0;
                public double appliedVolts = 0.0;
                public double supplyCurrentAmps = 0.0;
                public double statorCurrentAmps = 0.0;
                public double tempCelsius = 0.0;
        }

        default void updateInputs(HoodIOInputs inputs) {}
        default void setPositionDegrees(double degrees) {}
        default void setVoltage(double volts) {}
        default void resetPosition(double positionRotations) {}
        default void stop() {}
}
