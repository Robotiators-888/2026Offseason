package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Metering extends SubsystemBase {
    private final TalonFX metering;
    private final TalonFX meteringFollower;
    private static SUB_Metering INSTANCE = null;
    public static SUB_Metering getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Metering();
        }
        return INSTANCE;
    }
    private SUB_Metering() {
        metering = new TalonFX(Constants.Metering.kMETERING_MOTOR_CAN_ID);
        meteringFollower = new TalonFX(Constants.Metering.kMETERING_MOTOR_FOLLOWER_CAN_ID);
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(.5));
        metering.getConfigurator().apply(config);
        meteringFollower.getConfigurator().apply(config);
        meteringFollower.setControl(new Follower(metering.getDeviceID(), MotorAlignmentValue.Aligned));
    }
    public void set (double speed) {
        metering.set(speed);
    }
}
