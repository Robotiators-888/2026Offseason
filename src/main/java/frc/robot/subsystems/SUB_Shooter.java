package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;
    private TalonFX topFlywheel;
    private TalonFX bottomFlywheel;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }
    private SUB_Shooter(){
        //Defines flywheel motors with their IDs
        topFlywheel = new TalonFX(Constants.Shooter.kSHOOTER_topFlywheel_MOTOR_CANID); 
        bottomFlywheel = new TalonFX(Constants.Shooter.kSHOOTER_bottomFlywheel_MOTOR_CANID);
        // Metering wheel speed at .3
        // distanceToRPM.put(Units.inchesToMeters(64), 1241.0);
        // distanceToRPM.put(Units.inchesToMeters(95), 1430.0);
        // distanceToRPM.put(Units.inchesToMeters(129), 1660.0);
        // distanceToRPM.put(Units.inchesToMeters(164), 1450.0);
        // distanceToRPM.put(Units.inchesToMeters(107), 1275.0);
        // distanceToRPM.put(Units.inchesToMeters(86.61), 1245.0);
        //sets RPMs based on meter distance
        distanceToRPM.put(2.49493587092, 1250.0);
        distanceToRPM.put(3.03308176613, 1375.0+15);
        distanceToRPM.put(1.6346195276, 1075.0-25);
        distanceToRPM.put(4.10526503, 1575.0+25);
        distanceToRPM.put(5.34766117, 1750.0+40);
        distanceToRPM.put(10.5, 2400.0); //TODO:  VERY TEMPORARY< NEEDS  OTBE TESTED IRL
        configFlywheel();
    }

    private void configFlywheel() {
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true; //Enables current limit
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 70;  //sets Supply Current limit to 70 amps
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Sets flywheel to coast when not running (wont immediatly stop)
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //Positive motor will make motor spin clockwise
        //Sets PID values
        shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
        shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV; // The rpm in the docs means the target rpm we want to reach on average, not that we should multiply the rpm in code. Wtih our previous code we would have tripped the breaker if it had worked...
        shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
        shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP; 
        shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
        shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD; 
        //applies configs to the two shooter motors
        topFlywheel.getConfigurator().apply(shooterConfig);
        bottomFlywheel.getConfigurator().apply(shooterConfig);
        bottomFlywheel.setControl(new Follower(topFlywheel.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    
    @Deprecated
    //Creates method to set flywheel speeds
    public void set(double speed){
        topFlywheel.set(speed);
    }

    //Creates method to set RPM
    public void setRPM(double rpm) {
        this.desiredSpeed = rpm;
        topFlywheel.setControl(m_request.withVelocity(rpm / 60.0));
    }

    //returns flywheel RPM
    public double flywheelRPM() {
        return (topFlywheel.getVelocity().getValue().in(RPM) + bottomFlywheel.getVelocity().getValue().in(RPM)) / 2;
    }
  
    //Returns if the motor is at the needed RPM
    public boolean atDesiredRPM() {
        // return true;
        return Math.abs(flywheelRPM() - desiredSpeed) < 75; // Allow a tolerance of 50 RPM
        // This logic makes absolutely now sense?: return topFlywheel.getMotionMagicIsRunning().getValue(); //TODO: Is this correct for flywheel rpm speed?
    }

    //Sets RPM based on distance
    public void shootMeters(double meters) {
        // query the map for the RPM associated with this distance
        double targetRPM = distanceToRPM.get(meters);

        // Pass it to your existing setRPM method
        setRPM(targetRPM);
    }

    //returns RPM needed based on distance
    public double getDistanceRPM (double meters) {
        return distanceToRPM.get(meters);
    }

    //Sets motor speed to 0
    public void stop() {
        this.desiredSpeed = 0;
        topFlywheel.setControl(voltageRequest.withOutput(0));
    }

    //Sets motor volts to what is inputed
    public void setVolts(double volts) {
        topFlywheel.setControl(voltageRequest.withOutput(volts));
    }

    public void periodic() {
      SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed); //Puts desired RPM into smart dashboard
      
      SmartDashboard.putNumber("Shooter/Top Motor Stator Current", topFlywheel.getStatorCurrent().getValueAsDouble()); //Puts top flywheel stator current into dashboard
      SmartDashboard.putNumber("Shooter/Bottom Motor Stator Current", bottomFlywheel.getStatorCurrent().getValueAsDouble()); //Puts bottom flywheel stator current into dashboard
      
      SmartDashboard.putNumber("Shooter/Top Motor Supply Current", topFlywheel.getSupplyCurrent().getValueAsDouble()); //Puts top flywheel supply current into dashboard
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Current", bottomFlywheel.getSupplyCurrent().getValueAsDouble()); //Buts bottom flywheel supply current into dashboard
      
      SmartDashboard.putNumber("Shooter/Top Motor Supply Voltage", topFlywheel.getSupplyVoltage().getValueAsDouble());  //puts motor supply voltage into table
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Voltage", bottomFlywheel.getSupplyVoltage().getValueAsDouble());
      
      SmartDashboard.putNumber("Shooter/Top Motor Motor Voltage", topFlywheel.getMotorVoltage().getValueAsDouble()); //puts motor voltage into table
      SmartDashboard.putNumber("Shooter/Bottom Motor Motor Voltage", bottomFlywheel.getMotorVoltage().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/Top Motor Encoder Pos", topFlywheel.getPosition().getValueAsDouble()); //puts angle of top flywheel into table
      SmartDashboard.putNumber("Shooter/Bottom Motor Motor Encoder Pos", bottomFlywheel.getPosition().getValueAsDouble()); //puts angle of bottom table
      
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Top)", topFlywheel.getVelocity().getValue().in(RPM)); //Puts RPM of top flywheel into table
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Bottom)", bottomFlywheel.getVelocity().getValue().in(RPM)); //Puts RPM of bottom flywheel into table
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM()); //Puts average RPM of the flywheels into table
    }
    public double getExpectedTOF(double distanceMeters) {
        double targetRPM = distanceToRPM.get(distanceMeters);
        // 0.00434 is the estimated conversion factor from RPM to horizontal velocity (m/s)
        double averageHorizontalVelocity = targetRPM * 0.00434;
        if (averageHorizontalVelocity <= 0.0) {
            return 0.0;
        }
        return distanceMeters / averageHorizontalVelocity;
    }
    public static double getExpectedTOFStatic(double distanceMeters) {
        double targetRPM = getInstance().distanceToRPM.get(distanceMeters);
        // 0.00434 is the estimated conversion factor from RPM to horizontal velocity (m/s)
        double averageHorizontalVelocity = targetRPM * 0.00434;
        if (averageHorizontalVelocity <= 0.0) {
            return 0.0;
        }
        return distanceMeters / averageHorizontalVelocity;
    }
}
