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
    // Initializes variables in the subsystem
    private SparkMax index;
    private SparkMax meteringWheel;
    private SparkClosedLoopController meteringController;
    
    private double targetMeteringRPM = 0;

    // Sets up singleton
    private static SUB_Index INSTANCE = null;
    public static SUB_Index getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Index();
        }
        return INSTANCE;
    }
    
    private SUB_Index () {
        //Defines motors for metering and index motor
        index = new SparkMax(Constants.Index.KINDEX_MOTOR_CANID, MotorType.kBrushless);
        meteringWheel = new SparkMax(Constants.Index.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        //Sets up config for motors
        SparkMaxConfig indexConfig = new SparkMaxConfig();
        indexConfig.smartCurrentLimit(50); //sets current limit for index
        indexConfig.inverted(true); //makes index motor inverted
        index.configure(indexConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters); //sets sparkmax to persist mode so it wont lose settings
        SparkMaxConfig meteringConfig = new SparkMaxConfig(); //sets up config for metering wheel
        meteringConfig.smartCurrentLimit(60); // sets current limit
        double kP = 0.00005; // Super Aggressive P to get metering wheel to speed FAST
        double kI = 0.0;
        double kD = 0.0; 
        double kFF = 0.0021; // NEO Nominal RPM at 12V is ~5676. Max Wheel RPM is 5676. Since we measure the wheel, not the flywheel reduction: 1/5676 = 0.000176
        meteringConfig.closedLoop.pid(kP, kI, kD); //Applys PIDs
        meteringConfig.closedLoop.velocityFF(kFF); //applies Kff
        meteringConfig.encoder.uvwMeasurementPeriod(8);// TODO: Sidh Comment
        meteringConfig.encoder.uvwAverageDepth(2);
        meteringWheel.configure(meteringConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters); // Sets SparkMax to persist mode so it wont lose settings
        meteringController = meteringWheel.getClosedLoopController();
    }
    
    //Creates method to set index motor speed
    public void set(double speed){
        index.set(speed);
    }
    
    //Gets RPM of index motor
    public double indexRPM(){
        return index.getEncoder().getVelocity();
    }
    
    //Returns RPM of Metering wheel
    public double intakeMeteringRPM(){
        return meteringWheel.getEncoder().getVelocity(); 
    }

    //Sets Speed of motor
    public void setMeteringSpeed(double speed) {
        targetMeteringRPM = -1;
        meteringWheel.set(speed);
    }

    //Sets voltage of index motor
    public void setVolts(double volts) {
        index.setVoltage(volts);
    }

    //Sets voltage of metering motor
    public void setMeteringVolts(double volts) {
        targetMeteringRPM = -1;
        meteringWheel.setVoltage(volts);
    }

    //Sets RPM of metering wheel
    public void setMeteringRPM(double wheelRPM) {
        targetMeteringRPM = wheelRPM;
        meteringController.setReference(wheelRPM, ControlType.kVelocity);
    }
    
    //Stops metering wheel
    public void stopMetering() {
        targetMeteringRPM = 0;
        meteringWheel.set(0);
    }

    // Logs everything every periodic
    public void periodic() {
        SmartDashboard.putNumber("Index/Index RPM", indexRPM()); //Puts Index RPM into the dashboard
        
        SmartDashboard.putNumber("Index/Index Output Current", index.getOutputCurrent()); //Inputs how many amps are going from controller to motor
        SmartDashboard.putNumber("Index/Metering Output Current", meteringWheel.getOutputCurrent());
        
        SmartDashboard.putNumber("Index/Metering RPM", intakeMeteringRPM()); //Inputs metering wheel RPM
        SmartDashboard.putNumber("Index/Metering Target RPM", targetMeteringRPM); //Inputs metering wheel target RPM
        
        SmartDashboard.putNumber("Index/Metering Bus Voltage", meteringWheel.getBusVoltage()); //Returns volatge going into metering wheel Talon FX
        SmartDashboard.putNumber("Index/Index Bus Voltage", index.getBusVoltage()); //Returns voltage going into Indexing sparkmax

        SmartDashboard.putNumber("Index/Metering Encoder Pos", meteringWheel.getEncoder().getPosition());
        SmartDashboard.putNumber("Index/Index Encoder Pos", index.getEncoder().getPosition()); //Inputs how many amps are going from controller to motor
    
        SmartDashboard.putNumber("Index/Metering Motor Temp", meteringWheel.getMotorTemperature());
        SmartDashboard.putNumber("Index/Index Motor Temp", index.getMotorTemperature());
    }
}
