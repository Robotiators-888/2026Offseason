package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Climber extends SubsystemBase {
    private SparkMax climberMotor;
    private RelativeEncoder climbEncoder;

    private static SUB_Climber INSTANCE = null;
    public static SUB_Climber getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Climber();
        }
    
        return INSTANCE;
    }

    private SUB_Climber () {
        climberMotor = new SparkMax(Constants.Climber.kCLIMBER_MOTOR_CANID, SparkMax.MotorType.kBrushless);
        // climberArm = new SparkMax(Constants.Climber.kCLIMBER_PIVOT_CANID, SparkMax.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
            .countsPerRevolution(42)
            .positionConversionFactor(360.0); // Degrees
        config.smartCurrentLimit(35);
        climbEncoder = climberMotor.getEncoder();
        climberMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void setClimber(double speed){
        climberMotor.set(speed);
    }

    public boolean hasReachedSetPoint (boolean isExtending) {
        return (isExtending) ? climbEncoder.getPosition() == Constants.Climber.kCLIMBER_SETPOINT : climbEncoder.getPosition() == 0;
    }

    // public void setClimberArm(double speed){
    //     climberArm.set(speed);
    // }

    /*
     * Returns the position of the climber arm in degrees, where 0 degrees is the starting position, it is 360 degrees
     */
    // public double getClimberArmPosition(){ 
    //     return climberArmEncoder.getPosition();
    // }

    // public void periodic() {
    //     SmartDashboard.putNumber("ClimberArmPosition", getClimberArmPosition());
    // }

    // public void resetClimberArmEncoder() {
    //     climberArmEncoder.setPosition(0);
    // }

    // public boolean isClimberArmAtPosition(double targetPosition) {
    //     return Math.abs(getClimberArmPosition() - targetPosition) <= Constants.Climber.kCLIMBER_PIVOT_TOLERANCE;
    // }

    // public void setClimberArmToPosition(double targetPosition) {
    //     if (getClimberArmPosition() < targetPosition) {
    //         setClimberArm(Constants.Climber.kCLIMBER_PIVOT_SPEED); // Move up
    //     } else if (getClimberArmPosition() > targetPosition) {
    //         setClimberArm(-Constants.Climber.kCLIMBER_PIVOT_SPEED); // Move down
    //     } else {
    //         setClimberArm(0); // Stop
    //     }
    // }

    public void climb() {
        setClimber(-Constants.Climber.kCLIMBER_MOTOR_SPEED); // Yeah its backwards for some reason
    }

    public void unClimb() {
        setClimber(Constants.Climber.kCLIMBER_MOTOR_SPEED);
    }

    public void stopClimb() {
        setClimber(0);
    }
}
