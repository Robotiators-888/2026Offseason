package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

/**
 * AdvantageKit hardware abstraction interface for the intake roller subsystem.
 */
public interface RollerIO {
        @AutoLog
        public static class RollerIOInputs {
                public double leftVelocityRPM = 0.0;
                public double rightVelocityRPM = 0.0;
                public double leftAppliedVolts = 0.0;
                public double rightAppliedVolts = 0.0;
                public double leftSupplyCurrentAmps = 0.0;
                public double rightSupplyCurrentAmps = 0.0;
                public double leftStatorCurrentAmps = 0.0;
                public double rightStatorCurrentAmps = 0.0;
                public double leftTempCelsius = 0.0;
                public double rightTempCelsius = 0.0;
        }

        default void updateInputs(RollerIOInputs inputs) {}
        default void setRPM(double rpm) {}
        default void setVoltage(double volts) {}
        default void stop() {}
}
