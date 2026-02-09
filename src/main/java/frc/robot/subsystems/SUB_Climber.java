package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Climber extends SubsystemBase {
    private SparkMax climberWheels;
    private SparkMax climberArm;
    private RelativeEncoder climberArmEncoder;

    private static SUB_Climber INSTANCE = null;
    public static SUB_Climber getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Climber();
        }
    
        return INSTANCE;
    }

    private SUB_Climber (){
        climberWheels = new SparkMax(Constants.Climber.kCLIMBER_MOTOR_CANID, SparkMax.MotorType.kBrushless);
        climberArm = new SparkMax(Constants.Climber.kCLIMBER_ARM_MOTOR_CANID, SparkMax.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.alternateEncoder
            .countsPerRevolution(8192)
            .positionConversionFactor(360.0); // Degrees
        climberArm.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        climberArmEncoder = climberArm.getAlternateEncoder();
    }

    public void setClimberWheels(double speed){
        climberWheels.set(speed);
    }

    public void setClimberArm(double speed){
        climberArm.set(speed);
    }

    /*
     * Returns the position of the climber arm in degrees, where 0 degrees is the starting position, it is 360 degrees
     */
    public double getClimberArmPosition(){ 
        return climberArmEncoder.getPosition();
    }

    public void periodic() {
        SmartDashboard.putNumber("ClimberArmPosition", getClimberArmPosition());
    }

    public void resetClimberArmEncoder() {
        climberArmEncoder.setPosition(0);
    }

    public boolean isClimberArmAtPosition(double targetPosition, double tolerance) {
        return Math.abs(getClimberArmPosition() - targetPosition) <= tolerance;
    }

    public void setClimberArmToPosition(double targetPosition, double speed) {
        if (getClimberArmPosition() < targetPosition) {
            setClimberArm(speed); // Move up
        } else if (getClimberArmPosition() > targetPosition) {
            setClimberArm(-speed); // Move down
        } else {
            setClimberArm(0); // Stop
        }
    }

    public void climb() {
        setClimberWheels(Constants.Climber.kCLIMBER_MOTOR_SPEED);
    }

    public void unClimb() {
        setClimberWheels(-Constants.Climber.kCLIMBER_MOTOR_SPEED);
    }

    public void stopClimb() {
        setClimberWheels(0);
    }
}
