package frc.robot.commands.Motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * CTRE {@link TalonFX} implementation of the {@link TuneableMotor} interface for Phoenix 6.
 *
 * <p>Wraps a physical TalonFX motor controller (such as a Kraken X60 or Falcon 500) to safely
 * perform voltage-based feedforward and feedback tuning.
 *
 * <p>For mechanisms with multiple motors geared together, pass the <b>leader</b> motor into this wrapper.
 * As long as follower motors are set to follow the leader via Phoenix 6
 * (e.g., {@code follower.setControl(new Follower(leader.getDeviceID(), opposeMaster))}),
 * they will automatically mirror all voltage commands applied during tuning.
 */
public class TalonTuneableMotor implements TuneableMotor {

    /** The underlying physical CTRE {@link TalonFX} controller. */
    private final TalonFX motor;

    /** Configuration applied to the TalonFX, including current limits. */
    private final TalonFXConfiguration config;

    /**
     * Constructs a new {@link TalonTuneableMotor} with specified hardware protection limits.
     *
     * @param motor The target {@link TalonFX} device (or leader motor of a geared group).
     * @param config The base {@link TalonFXConfiguration} profile for this mechanism.
     * @param SupplyCurrentLimit Breaker/battery supply current threshold in Amperes (A) (e.g. 40.0A or 60.0A).
     * @param StatorCurrentLimit Motor winding/stator current threshold in Amperes (A) (e.g. 80.0A) to prevent motor burn-out.
     */
    public TalonTuneableMotor(TalonFX motor, TalonFXConfiguration config, double SupplyCurrentLimit, double StatorCurrentLimit) {
        this.motor = motor;
        this.config = config;
        this.config.withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(StatorCurrentLimit)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SupplyCurrentLimit));
        motor.getConfigurator().apply(config);
    }

    /**
     * Sets the motor output as an uncompensated duty cycle percentage.
     *
     * @param dutycycle Fractional power output in the range [-1.0, 1.0].
     */
    @Override
    public void set(double dutycycle) {
        motor.setControl(new DutyCycleOut(dutycycle));
    }

    /**
     * Sets the motor output voltage using a WPILib {@link Voltage} measure.
     *
     * @param voltage Desired output voltage (e.g., {@code Volts.of(6.0)}).
     */
    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    /**
     * Sets the motor output voltage in Volts.
     *
     * @param voltage Desired output in Volts (V), typically in the range [-12.0 V, 12.0 V].
     */
    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    /**
     * Retrieves the current rotational velocity of the motor shaft.
     *
     * @return Angular velocity as a WPILib {@link AngularVelocity} measure (in Rotations/s or RPM).
     */
    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }
}
