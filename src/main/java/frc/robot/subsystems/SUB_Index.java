package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Index extends SubsystemBase {
    /** Subsystem hardware components */
    private SparkMax LeftIndexer;
    private SparkMax RightIndexer;
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
        LeftIndexer = new SparkMax(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
        RightIndexer = new SparkMax(Constants.Index.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        
        // Configure main indexer motor
        SparkMaxConfig LeftIndexConfig = new SparkMaxConfig();
        LeftIndexConfig.smartCurrentLimit(60);
        LeftIndexConfig.follow(RightIndexer, true);
        LeftIndexer.configure(LeftIndexConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        SparkMaxConfig RightIndexConfig = new SparkMaxConfig();
        RightIndexConfig.smartCurrentLimit(60);
        RightIndexConfig.inverted(true);
        RightIndexer.configure(RightIndexConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        
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

        
    }
    
    /** @param speed Target percent output for indexing [-1.0, 1.0] */
    public void set(double speed){
        RightIndexer.set(speed);
    }
    
    /** @return Current velocity of the indexer in RPM */
    public double indexRPM(){
        return (RightIndexer.getEncoder().getVelocity() + LeftIndexer.getEncoder().getVelocity())/2;
    }
    


    /** @param volts Target voltage for the index motor */
    public void setVolts(double volts) {
        RightIndexer.setVoltage(volts);
    }




    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Index RPM", indexRPM());
        SmartDashboard.putNumber("Right Index Output Current", RightIndexer.getOutputCurrent());
        SmartDashboard.putNumber("Left Index Output Current", LeftIndexer.getOutputCurrent());

        SmartDashboard.putNumber("Right Index Bus Voltage", RightIndexer.getBusVoltage());
        SmartDashboard.putNumber("Left Index Bus Voltage", LeftIndexer.getBusVoltage());

        SmartDashboard.putNumber("Right Index Encoder Pos", RightIndexer.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Index Encoder Pos", LeftIndexer.getEncoder().getPosition());

        SmartDashboard.putNumber("Right Index Motor Temp", RightIndexer.getMotorTemperature());
        SmartDashboard.putNumber("Left Index Motor Temp", LeftIndexer.getMotorTemperature());

        Alert.alertNeoFaults(RightIndexer);
        Alert.alertNeoWarnings(RightIndexer);
        Alert.alertNeoFaults(LeftIndexer);
        Alert.alertNeoWarnings(LeftIndexer);

    }
}
