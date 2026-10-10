package frc.robot.commands.Motor;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TuneableMotor {
    public void set(double dutycycle);
    public void setVoltage(Voltage voltage);
    public void setVoltage(double voltage);
    public AngularVelocity getVelocity();
}
