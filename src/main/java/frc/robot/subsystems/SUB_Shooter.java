package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;
    private TalonFX flyWheel1;
    private TalonFX flyWheel2;
    private double desiredSpeed = 0;
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }
    private SUB_Shooter(){
        flyWheel1 = new TalonFX(Constants.Shooter.kSHOOTER_FLYWHEEL1_MOTOR_CANID); // Might need a CANBus
        flyWheel2 = new TalonFX(Constants.Shooter.kSHOOTER_FLYWHEEL2_MOTOR_CANID);
    }
    public void set(double speed){
        flyWheel1.set(speed);
        flyWheel2.set(speed);
        desiredSpeed = speed;
    }
    // public double flywheelRPM() {
    //     return (flyWheel1.getEncoder().getVelocity()+flyWheel2.getEncoder().getVelocity())/2;
    // }
  
    // public boolean atdesiredRPM() {
    //     return flywheelRPM()>= desiredSpeed;
    // }

//     public void periodic() {
//       SmartDashboard.putNumber("FlywheelRPM", flywheelRPM());
//    }
}