package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
                .withStatorCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(25)
                .withSupplyCurrentLowerLimit(15)
                .withSupplyCurrentLowerTime(.5));
        hood = new TalonFX(Constants.Hood.KHOOD_CAN_ID);
        hood.getConfigurator().apply(config);
    }
    public void setToPosition (double angle) {
        hood.set(Constants.Hood.kHOOD_PID_CONTROLLER.calculate(hood.getPosition().getValueAsDouble(), angle));
    }
    public void setToDistance (double position) {
        setToPosition(Constants.Hood.kHOOD_MAP.get(position));
    }
    public double getPosition () {
        return hood.getPosition().getValueAsDouble();
    }
    @Override
    public void periodic () {
        SmartDashboard.putNumber("Hood/Position", getPosition());
    }
}
