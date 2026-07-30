package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

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

    @Override
    public void periodic () {
        SmartDashboard.putNumber("Hood/Leader Position", metering.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Position", meteringFollower.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Stator Current", metering.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Stator Current", meteringFollower.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Supply Current", metering.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Supply Current", meteringFollower.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Supply Voltage", metering.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Supply Voltage", meteringFollower.getSupplyVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Motor Voltage", metering.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Motor Voltage", meteringFollower.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Torque Current", metering.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Torque Current", meteringFollower.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Device Temp", metering.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Device Temp", meteringFollower.getDeviceTemp().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Processor Temp", metering.getProcessorTemp().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Processor Temp", meteringFollower.getProcessorTemp().getValueAsDouble());

        SmartDashboard.putNumber("Hood/Leader Velocity", metering.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Follower Velocity", meteringFollower.getVelocity().getValueAsDouble());
        // TODO: Implement rpm stuff

        Alert.alertKraken(metering);
        Alert.alertKraken(meteringFollower);
    }
}
