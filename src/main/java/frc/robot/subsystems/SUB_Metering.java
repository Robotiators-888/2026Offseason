package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Metering extends SubsystemBase {
    private final TalonFX metering;
    private final TalonFX meteringFollower;
    private static SUB_Metering INSTANCE = null;

    /** See the identical check in SUB_Shooter — a wrong MotorAlignmentValue shows up as this. */
    private final Debouncer followerOpposedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);

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
                .withStatorCurrentLimit(120)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLowerLimit(25)
                .withSupplyCurrentLowerTime(.5));
        metering.getConfigurator().apply(config);
        meteringFollower.getConfigurator().apply(config);
        meteringFollower.setControl(new Follower(metering.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void set (final double speed) {
        metering.set(speed);
    }

    /** Loop counter used to decimate the diagnostic telemetry below to ~4 Hz. */
    private int telemetryTick = 0;

    @Override
    public void periodic () {
        // Velocity is read every loop for the follower-alignment check; everything else is
        // slow-moving diagnostics published at ~4 Hz. These keys used to sit under "Hood/" —
        // a copy-paste from SUB_Hood that filed the metering stage's entire diagnostic surface
        // under the wrong mechanism.
        final double leaderRPS = metering.getVelocity().getValueAsDouble();
        final double followerRPS = meteringFollower.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Metering/Leader Velocity", leaderRPS);
        SmartDashboard.putNumber("Metering/Follower Velocity", followerRPS);

        if (followerOpposedDebouncer.calculate(
                Math.abs(leaderRPS) > 5 && Math.abs(followerRPS) > 5
                    && Math.signum(followerRPS) != Math.signum(leaderRPS))) {
            Alert.registerError("Metering follower (46) opposing leader — MotorAlignmentValue likely wrong");
        }

        if (telemetryTick++ % 12 == 0) {
            SmartDashboard.putNumber("Metering/Leader Position", metering.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Position", meteringFollower.getPosition().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Stator Current", metering.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Stator Current", meteringFollower.getStatorCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Supply Current", metering.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Supply Current", meteringFollower.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Supply Voltage", metering.getSupplyVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Supply Voltage", meteringFollower.getSupplyVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Motor Voltage", metering.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Motor Voltage", meteringFollower.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Torque Current", metering.getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Torque Current", meteringFollower.getTorqueCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Device Temp", metering.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Device Temp", meteringFollower.getDeviceTemp().getValueAsDouble());

            SmartDashboard.putNumber("Metering/Leader Processor Temp", metering.getProcessorTemp().getValueAsDouble());
            SmartDashboard.putNumber("Metering/Follower Processor Temp", meteringFollower.getProcessorTemp().getValueAsDouble());
        }

        Alert.alertKraken(metering);
        Alert.alertKraken(meteringFollower);
    }
}
