package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Index extends SubsystemBase {
    // Needs to be a neo vortex? IDK
    private SparkMax index;
    private SparkMax meteringWheel;
    private static SUB_Index INSTANCE = null;
    public static SUB_Index getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Index();
        }
        return INSTANCE;
    }
    private SUB_Index () {
        index = new SparkMax(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
        meteringWheel = new SparkMax(Constants.Shooter.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35);
        index.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        meteringWheel.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }
    public void set(double speed){
        index.set(speed);
    }
    public double intakeRPM(){
        return index.getEncoder().getVelocity()/3; // Gear Ratio
    }

    public void setMeteringSpeed(double speed) {
        meteringWheel.set(speed);
    }

    public void periodic() {
      SmartDashboard.putNumber("indexRPM", intakeRPM());
    }
}
