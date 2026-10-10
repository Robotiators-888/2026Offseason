package frc.robot.commands.Motor;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * REV Robotics {@link SparkBase} (SparkMax / SparkFlex) implementation of the {@link TuneableMotor} interface.
 *
 * <p>Wraps a physical NEO or Vortex brushless motor using the modern REVLib configuration API
 * to safely execute automated feedforward and feedback velocity tuning.
 *
 * <p>For mechanisms with multiple motors geared together, pass the <b>leader</b> Spark into this wrapper.
 * Follower Sparks configured with {@code follower.follow(leader)} will automatically mirror
 * all voltage commands issued during tuning.
 */
public class SparkTuneableMotor implements TuneableMotor {

    /** The underlying physical REV {@link SparkBase} controller (SparkMax or SparkFlex). */
    private final SparkBase motor;

    /** Configuration applied to the controller. */
    private final SparkBaseConfig config;

    /**
     * Constructs a new {@link SparkTuneableMotor} with a specified smart current limit.
     *
     * @param motor The target {@link SparkBase} device (or leader motor of a geared group).
     * @param config The {@link SparkBaseConfig} profile containing motor settings and sensor setups.
     * @param currentLimit Smart current limit threshold in Amperes (A) (e.g., 40 A for NEO, 60 A for Vortex) to prevent breaker trips.
     */
    public SparkTuneableMotor(SparkBase motor, SparkBaseConfig config, int currentLimit) {
        this.motor = motor;
        this.config = config;
        this.config.smartCurrentLimit(currentLimit);
        motor.configure(config, SparkMax.ResetMode.kResetSafeParameters,
                    SparkMax.PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor output as an uncompensated duty cycle percentage.
     *
     * @param dutycycle Fractional power output in the range [-1.0, 1.0].
     */
    @Override
    public void set(double dutycycle) {
        motor.set(dutycycle);
    }

    /**
     * Sets the motor output voltage using a WPILib {@link Voltage} measure.
     *
     * @param voltage Desired output voltage (e.g., {@code Volts.of(6.0)}).
     */
    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Sets the motor output voltage in Volts.
     *
     * @param voltage Desired output in Volts (V), typically in the range [-12.0 V, 12.0 V].
     */
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Retrieves the rotational velocity from the built-in hall encoder.
     *
     * @return Angular velocity as a WPILib {@link AngularVelocity} measure (converted from encoder RPM).
     */
    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }
}
