package frc.robot.commands.Motor;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class TalonTuneableMotor implements TuneableMotor {

    TalonFX motor;
    TalonFXConfiguration config;
    public TalonTuneableMotor(TalonFX motor, TalonFXConfiguration config, double SupplyCurrentLimit, double StatorCurrentLimit) {
        this.motor = motor;
        this.config = config;
        this.config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(StatorCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SupplyCurrentLimit));
        motor.getConfigurator().apply(config);
    }

    @Override
    public void set(double dutycycle) {
        motor.setControl(new DutyCycleOut(dutycycle));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }
}
