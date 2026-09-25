package frc.robot.subsystems.metering;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the dual-motor metering system.
 */
public interface MeteringIO {
        @AutoLog
        public static class MeteringIOInputs {
                public double leaderVelocityRPM = 0.0;
                public double followerVelocityRPM = 0.0;
                public double leaderPositionRotations = 0.0;
                public double followerPositionRotations = 0.0;
                public double leaderAppliedVolts = 0.0;
                public double followerAppliedVolts = 0.0;
                public double leaderSupplyCurrentAmps = 0.0;
                public double followerSupplyCurrentAmps = 0.0;
                public double leaderStatorCurrentAmps = 0.0;
                public double followerStatorCurrentAmps = 0.0;
                public double leaderTempCelsius = 0.0;
                public double followerTempCelsius = 0.0;
        }

        public default void updateInputs(MeteringIOInputs inputs) {}

        public default void setRPM(double rpm) {}

        public default void setVoltage(double volts) {}

        public default void stop() {}
}
