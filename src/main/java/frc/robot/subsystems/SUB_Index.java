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
    /** Subsystem hardware components */
    private SparkMax index;
    private SparkMax meteringWheel;
    private SparkClosedLoopController meteringController;
    
    private double targetMeteringRPM = 0;

    private static SUB_Index INSTANCE = null;

    /** @return Single instance of the SUB_Index subsystem */
    public static SUB_Index getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Index();
        }
        return INSTANCE;
    }
    
    private SUB_Index () {
        // Defines motors for indexing and metering
        index = new SparkMax(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
        meteringWheel = new SparkMax(Constants.Index.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        
        // Configure main indexer motor
        SparkMaxConfig indexConfig = new SparkMaxConfig();
        indexConfig.smartCurrentLimit(80);
        indexConfig.inverted(true);
        index.configure(indexConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        // Configure high-speed metering wheel with aggressive PID
        SparkMaxConfig meteringConfig = new SparkMaxConfig();
        meteringConfig.smartCurrentLimit(60);
        
        double kP = 0.00005; // Aggressive P for rapid speed ramp
        double kI = 0.0;
        double kD = 0.0; 
        double kFF = 0.0021; // Based on NEO nominal RPM at 12V
        
        meteringConfig.closedLoop.pid(kP, kI, kD);
        meteringConfig.closedLoop.velocityFF(kFF);
        meteringConfig.encoder.uvwMeasurementPeriod(8);
        meteringConfig.encoder.uvwAverageDepth(2);
        meteringWheel.configure(meteringConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        meteringController = meteringWheel.getClosedLoopController();
    }
    
    /** @param speed Target percent output for indexing [-1.0, 1.0] */
    public void set(double speed){
        index.set(speed);
    }
    
    /** @return Current velocity of the indexer in RPM */
    public double indexRPM(){
        return index.getEncoder().getVelocity();
    }
    
    /** @return Current velocity of the metering wheel in RPM */
    public double intakeMeteringRPM(){
        return meteringWheel.getEncoder().getVelocity(); 
    }

    /** @param speed Target percent output for metering [-1.0, 1.0] */
    public void setMeteringSpeed(double speed) {
        targetMeteringRPM = -1;
        meteringWheel.set(speed);
    }

    /** @param volts Target voltage for the index motor */
    public void setVolts(double volts) {
        index.setVoltage(volts);
    }

    /** @param volts Target voltage for the metering motor */
    public void setMeteringVolts(double volts) {
        targetMeteringRPM = -1;
        meteringWheel.setVoltage(volts);
    }

    /** @param wheelRPM Target velocity for the metering wheel in RPM */
    public void setMeteringRPM(double wheelRPM) {
        targetMeteringRPM = wheelRPM;
        meteringController.setReference(wheelRPM, ControlType.kVelocity);
    }
    
    /** Stops the metering wheel entirely */
    public void stopMetering() {
        targetMeteringRPM = 0;
        meteringWheel.set(0);
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Index/Index RPM", indexRPM());
        SmartDashboard.putNumber("Index/Index Output Current", index.getOutputCurrent());
        SmartDashboard.putNumber("Index/Metering Output Current", meteringWheel.getOutputCurrent());
        SmartDashboard.putNumber("Index/Metering RPM", intakeMeteringRPM());
        SmartDashboard.putNumber("Index/Metering Target RPM", targetMeteringRPM);
        SmartDashboard.putNumber("Index/Metering Bus Voltage", meteringWheel.getBusVoltage());
        SmartDashboard.putNumber("Index/Index Bus Voltage", index.getBusVoltage());
        SmartDashboard.putNumber("Index/Metering Encoder Pos", meteringWheel.getEncoder().getPosition());
        SmartDashboard.putNumber("Index/Index Encoder Pos", index.getEncoder().getPosition());
        SmartDashboard.putNumber("Index/Metering Motor Temp", meteringWheel.getMotorTemperature());
        SmartDashboard.putNumber("Index/Index Motor Temp", index.getMotorTemperature());
    }
}
