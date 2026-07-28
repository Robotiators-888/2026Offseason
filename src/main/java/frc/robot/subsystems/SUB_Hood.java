package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;

public class SUB_Hood {
    private static SUB_Hood INSTANCE = null;
    private TalonFX hood;
    private PIDController pidController = new PIDController(4, 0.5, 0.025);
    private InterpolatingDoubleTreeMap hoodMap;
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
        hood = new TalonFX(Constants.Hood.kHoodCanId);
        hood.getConfigurator().apply(config);
    }
    public void setToPosition (double angle) {
        hood.set(pidController.calculate(hood.getPosition().getValueAsDouble(), angle));
    }
    public void setToDistance (double position) {
        
    }
}
