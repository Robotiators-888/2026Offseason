package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;

public class SUB_Metering {
    private final TalonFX motorOne;
    private final TalonFX motorTwo;
    private static SUB_Metering INSTANCE = null;
    public static SUB_Metering getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Metering();
        }
        return INSTANCE;
    }
    private SUB_Metering() {
        motorOne = new TalonFX(Constants.Metering.kMETERING_MOTOR_ONE_CAN_ID);
        motorTwo = new TalonFX(Constants.Metering.kMETERING_MOTOR_TWO_CAN_ID);
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(.5));
        motorOne.getConfigurator().apply(config);
        motorTwo.getConfigurator().apply(config);
        motorTwo.setControl(new Follower(motorOne.getDeviceID(), MotorAlignmentValue.Aligned));
    }
    public void set (double speed) {
        motorOne.set(speed);
    }
}
