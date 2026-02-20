package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Intake extends SubsystemBase {
    public static boolean extended;
    private TalonFX intake;
    private SparkMax arm;
    private SparkMax armFollower;
    private SparkLimitSwitch forwardLimit;
    private SparkLimitSwitch reverseLimit;
    private static SUB_Intake INSTANCE = null;
    public static SUB_Intake getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Intake();
        } 
        return INSTANCE;
    }

    private SUB_Intake () {
        intake = new TalonFX(Constants.Intake.kINTAKE_MOTOR_CANID);
        arm = new SparkMax(Constants.Intake.kARM_MOTOR_CANID, MotorType.kBrushless);
        armFollower = new SparkMax(Constants.Intake.kARM_FOLLOWER_MOTOR_CANID, MotorType.kBrushless);
        forwardLimit = arm.getForwardLimitSwitch();
        reverseLimit = arm.getReverseLimitSwitch();
        configureMotors();
    }

    private void configureMotors(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.limitSwitch
            .forwardLimitSwitchEnabled(true)
            .forwardLimitSwitchType(Type.kNormallyOpen)//TODO: Test if normally open or normally closed, we want it to be normally open so that if the switch breaks it will just not trigger instead of always triggering and breaking the code
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen);//TODO: Test if normally open or normally closed, we want it to be normally open so that if the switch breaks it will just not trigger instead of always triggering and breaking the code
        config.encoder.positionConversionFactor(360.0 / 23); // Converts rotations to degrees, Thrifty bot cycloial gearbox 23:1
        config.encoder.velocityConversionFactor((360.0 / 23) / 60.0); // Converts RPM to deg/sec
        config.smartCurrentLimit(35);
        config.inverted(true);
        arm.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(arm, true);
        followerConfig.smartCurrentLimit(35);
        armFollower.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 35;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intake.getConfigurator().apply(talonConfig);
    }

    public boolean isForwardPressed() {
        return forwardLimit.isPressed();
    }

    public boolean isReversePressed() {
        return reverseLimit.isPressed();
    }

    public void set(double speed){
        intake.set(speed);
    }

    public double intakeRPM(){
        return intake.getVelocity().getValue().baseUnitMagnitude();
    }

    public void periodic() {
        if (isForwardPressed()) {
            arm.getEncoder().setPosition(0);
        }
        SmartDashboard.putNumber("intakeRPM", intakeRPM());
        SmartDashboard.putBoolean("Arm Forward Limit", isForwardPressed());
        SmartDashboard.putBoolean("Arm Reverse Limit", isReversePressed());
        SmartDashboard.putNumber("Arm Encoder Pos", arm.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Arm Output Amps", arm.getOutputCurrent());
    }

    public void setArm (double speed) {
        arm.set(speed);
    }

    public Command retractArm() {
        return Commands.run(() -> setArm(Constants.Intake.kINTAKE_ARM_MOTOR_SPEED), this)
            .until(this::isReversePressed) // Stop command when switch is hit
            .finallyDo(() -> {
                setArm(0);
                // HOMING: Reset encoder to 0 once we hit the back limit
                arm.getEncoder().setPosition(0);
            });
    }

    public Command extendArm() {
        return Commands.run(() -> setArm(-Constants.Intake.kINTAKE_ARM_MOTOR_SPEED), this)
            .until(this::isForwardPressed) // Stop command when switch is hit
            .finallyDo(() -> setArm(0));
    }

    public boolean isExtended() {
        return extended;
    }
}
