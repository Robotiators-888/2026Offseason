package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Index extends SubsystemBase {
    // Needs to be a neo vortex
    private SparkFlex index;
    private static SUB_Index INSTANCE = null;
    public static SUB_Index getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Index();
        }
        return INSTANCE;
    }
    private SUB_Index () {
        index = new SparkFlex(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
    }
    public void set(double speed){
        index.set(speed);
    }
    public double intakeRPM(){
        return index.getEncoder().getVelocity();
    }
    public void periodic() {
      SmartDashboard.putNumber("indexRPM", intakeRPM());
    }
}
