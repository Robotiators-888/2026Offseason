package frc.robot.commands.Motor;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Universal hardware-abstraction interface for motors tuned by {@link frc.robot.commands.CMD_VelocityTuner}.
 *
 * <p>Provides a standardized control and feedback API across different motor controllers
 * (e.g., CTRE {@link com.ctre.phoenix6.hardware.TalonFX}, REV {@link com.revrobotics.spark.SparkBase},
 * and physics simulation).
 */
public interface TuneableMotor {

    /**
     * Sets the motor output as an uncompensated duty cycle percentage.
     *
     * @param dutycycle Normalized fractional output in the range [-1.0, 1.0] (where 1.0 is full forward).
     */
    public void set(double dutycycle);

    /**
     * Sets the motor output using a strongly-typed WPILib {@link Voltage} measure.
     *
     * @param voltage Desired output voltage (e.g., {@code Volts.of(6.0)}).
     */
    public void setVoltage(Voltage voltage);

    /**
     * Sets the motor output voltage in Volts.
     *
     * @param voltage Desired output in Volts (V), typically in the range [-12.0 V, 12.0 V].
     */
    public void setVoltage(double voltage);

    /**
     * Retrieves the live rotational velocity of the motor rotor or mechanism shaft.
     *
     * @return Current rotational speed as a WPILib {@link AngularVelocity} measure
     *         (queryable via {@code in(RPM)} or {@code in(RadiansPerSecond)}).
     */
    public AngularVelocity getVelocity();
}
