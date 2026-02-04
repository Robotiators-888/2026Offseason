package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class SUB_Intake extends SubsystemBase {
    private SparkMax intake;
    private static SUB_Intake INSTANCE = null;
    public static SUB_Intake getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Intake();
        } 
        return INSTANCE;
    }

    private SUB_Intake (){
        intake = new SparkMax(Constants.Intake.kINTAKE_MOTOR_CANID, MotorType.kBrushless);
    }
    public void set(double speed){
        intake.set(speed);
    }
    public double intakeRPM(){
        return intake.getEncoder().getVelocity();
    }
    public void periodic() {
      SmartDashboard.putNumber("intakeRPM", intakeRPM());
    }
}
