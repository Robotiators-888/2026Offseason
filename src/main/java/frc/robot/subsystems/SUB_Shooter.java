package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;
    private SparkMax flyWheel;
    private double desiredSpeed = 0;
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }
    private SUB_Shooter(){
        flyWheel = new SparkMax(Constants.Shooter.kSHOOTER_FLYWHEEL_MOTOR_CANID, MotorType.kBrushless);
    }
    public void set(double speed){
        flyWheel.set(speed);
        desiredSpeed = speed;
    }
    public double flywheelRPM(){
        return flyWheel.getEncoder().getVelocity();
    }
  
    public boolean atdesiredRPM() {
        return flywheelRPM()>= desiredSpeed;
    }
    public void periodic() {
      SmartDashboard.putNumber("FlywheelRPM", flywheelRPM());
   }
}