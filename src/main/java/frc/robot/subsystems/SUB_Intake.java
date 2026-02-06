package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SUB_Intake extends SubsystemBase {
    private TalonFX intake;
    private SparkMax arm;
    private static SUB_Intake INSTANCE = null;
    public static SUB_Intake getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Intake();
        } 
        return INSTANCE;
    }

    private SUB_Intake (){
        intake = new TalonFX(Constants.Intake.kINTAKE_MOTOR_CANID);
        arm = new SparkMax(Constants.Intake.kARM_MOTOR_CANID, MotorType.kBrushless);
    }

    public void set(double speed){
        intake.set(speed);
    }

    public double intakeRPM(){
        return intake.getVelocity().getValue().baseUnitMagnitude();
    }

    public void periodic() {
      SmartDashboard.putNumber("intakeRPM", intakeRPM());
    }

    public void setArm (double speed) {
        arm.set(speed);
    }
}
