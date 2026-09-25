package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit hardware abstraction interface for the flywheel shooter subsystem.
 */
public interface ShooterIO {
        @AutoLog
        public static class ShooterIOInputs {
                public double leaderVelocityRPM = 0.0;
                public double followerVelocityRPM = 0.0;
                public double leaderAppliedVolts = 0.0;
                public double followerAppliedVolts = 0.0;
                public double leaderSupplyCurrentAmps = 0.0;
                public double followerSupplyCurrentAmps = 0.0;
                public double leaderStatorCurrentAmps = 0.0;
                public double followerStatorCurrentAmps = 0.0;
                public double leaderTempCelsius = 0.0;
                public double followerTempCelsius = 0.0;
        }

        default void updateInputs(ShooterIOInputs inputs) {}
        default void setVelocity(double targetRPM) {}
        default void setVoltage(double volts) {}
        default void stop() {}
}
