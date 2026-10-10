package frc.robot.commands.Motor;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SparkTuneableMotor implements TuneableMotor {

    SparkBase motor;
    SparkBaseConfig config;
    public SparkTuneableMotor(SparkBase motor, SparkBaseConfig config, int currentLimit) {
        this.motor = motor;
        this.config = config;
        this.config.smartCurrentLimit(currentLimit);
        motor.configure(config, SparkMax.ResetMode.kResetSafeParameters,
                    SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void set(double dutycycle) {
        motor.set(dutycycle);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }
}
