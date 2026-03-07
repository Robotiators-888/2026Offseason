package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Index extends SubsystemBase {
    // Needs to be a neo vortex? IDK
    private SparkMax index;
    private SparkMax meteringWheel;
    private SparkClosedLoopController meteringController;
    
    private double targetMeteringRPM = 0;

    private static SUB_Index INSTANCE = null;
    public static SUB_Index getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Index();
        }
        return INSTANCE;
    }
    
    private SUB_Index () {
        index = new SparkMax(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
        meteringWheel = new SparkMax(Constants.Index.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        SparkMaxConfig indexConfig = new SparkMaxConfig();
        indexConfig.smartCurrentLimit(60);
        indexConfig.inverted(true);
        index.configure(indexConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        SparkMaxConfig meteringConfig = new SparkMaxConfig();
        meteringConfig.smartCurrentLimit(60);
        double kP = 0.00005; // Super Aggressive P to get metering wheel to speed FAST
        double kI = 0.0;
        double kD = 0.0; 
        double kFF = 0.0021; // NEO Nominal RPM at 12V is ~5676. Max Wheel RPM is 5676. Since we measure the wheel, not the flywheel reduction: 1/5676 = 0.000176
        meteringConfig.closedLoop.pid(kP, kI, kD);
        meteringConfig.closedLoop.velocityFF(kFF);
        meteringConfig.encoder.uvwMeasurementPeriod(8); 
        meteringConfig.encoder.uvwAverageDepth(2);
        meteringWheel.configure(meteringConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        meteringController = meteringWheel.getClosedLoopController();
    }
    
    public void set(double speed){
        index.set(speed);
    }
    
    public double intakeRPM(){
        return index.getEncoder().getVelocity();
    }
    
    public double intakeMeteringRPM(){
        return meteringWheel.getEncoder().getVelocity(); 
    }

    public void setMeteringSpeed(double speed) {
        targetMeteringRPM = -1;
        meteringWheel.set(speed);
    }
    public void setVolts(double volts) {
        index.setVoltage(volts);
    }
    public void setMeteringVolts(double volts) {
        targetMeteringRPM = -1;
        meteringWheel.setVoltage(volts);
    }
    public void setMeteringRPM(double wheelRPM) {
        targetMeteringRPM = wheelRPM;
        meteringController.setReference(wheelRPM, ControlType.kVelocity);
    }
    
    public void stopMetering() {
        targetMeteringRPM = 0;
        meteringWheel.set(0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Index/indexRPM", intakeRPM());
        SmartDashboard.putNumber("Index/meteringRPM", intakeMeteringRPM());
        SmartDashboard.putNumber("Index/meteringTargetRPM", targetMeteringRPM);
        SmartDashboard.putNumber("Index/meteringCurrent", meteringWheel.getOutputCurrent());

    }
}
