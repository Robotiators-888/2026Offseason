package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Hood extends SubsystemBase {
    private static SUB_Hood INSTANCE = null;
    private final TalonFX hood;

    public static SUB_Hood getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Hood();
        }
        return INSTANCE;
    }

    private SUB_Hood () {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(25)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(7)
                .withSupplyCurrentLowerLimit(5)
                .withSupplyCurrentLowerTime(.5));
        hood = new TalonFX(Constants.Hood.kHOOD_CAN_ID);
        hood.getConfigurator().apply(config);
    }

    public void setToPosition (double angle) {
        hood.set(Constants.Hood.kHOOD_PID_CONTROLLER.calculate(hood.getPosition().getValueAsDouble(), angle));
    }

    public double getPosition () {
        return hood.getPosition().getValueAsDouble();
    }

    public static double findoptimalangle(final double distance) {
        double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
    }

    // Reset the hood to the origional position while including checks for motor stalling
    public void resetSafe () {
        setToPosition(0); // TODO: Actually make this safe maybe by using a high I PID and detected when the current gets high
    }

    public void resetEncoder () {
        hood.setPosition(0);
    }

    public void set (double speed) {
        hood.set(speed);
    }

    @Override
    public void periodic () {
        SmartDashboard.putNumber("Hood/Position", getPosition());
        SmartDashboard.putNumber("Hood/Stator Current", hood.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Supply Current", hood.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Supply Voltage", hood.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Motor Voltage", hood.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Torque Current", hood.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Device Temp", hood.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Processor Temp", hood.getProcessorTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Velocity", hood.getVelocity().getValueAsDouble());
        Alert.alertKraken(hood);
    }
}
