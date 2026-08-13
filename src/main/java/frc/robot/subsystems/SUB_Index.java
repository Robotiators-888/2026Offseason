package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Index extends SubsystemBase {
    /** Subsystem hardware components */
    private final SparkMax LeftIndexer;
    private final SparkMax RightIndexer;

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
        LeftIndexConfig.smartCurrentLimit(15);
        LeftIndexConfig.follow(RightIndexer, true);
        LeftIndexer.configure(LeftIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SparkMaxConfig RightIndexConfig = new SparkMaxConfig();
        RightIndexConfig.smartCurrentLimit(15);
        RightIndexConfig.inverted(true);
        RightIndexer.configure(RightIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    /** @param speed Target percent output for indexing [-1.0, 1.0] */
    public void set(double speed){
        RightIndexer.set(speed);
    }
    
    /**
     * @return Current velocity of the indexer in RPM
     *
     * <p>Reads the leader only. Averaging the two used to return roughly zero: the left indexer
     * follows inverted, so its encoder reports the opposite sign and the two cancelled.
     */
    public double indexRPM(){
        return RightIndexer.getEncoder().getVelocity();
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
