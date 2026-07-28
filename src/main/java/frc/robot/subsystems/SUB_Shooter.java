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
import frc.robot.utils.Alert;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;

    /** Subsystem hardware and control state */
    private TalonFX MotorOne;
    private TalonFX MotorTwo;
    private TalonFX MotorThree;
    private TalonFX MotorFour;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private double fuelShot = 0;
    private double lastRPM = 0;
    private double dipRPM = 0;
    private boolean hasGoneDown = false;
    private boolean isShooting;
    
    /** Interpolation map for distance-based RPM calibration */
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    /** @return Single instance of the SUB_Shooter subsystem */
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }

    private SUB_Shooter() {
        // Initialize dual flywheel motors
        MotorOne = new TalonFX(Constants.Shooter.kSHOOTER_MotorOne_MOTOR_CANID); 
        MotorTwo = new TalonFX(Constants.Shooter.kSHOOTER_MotorTwo_MOTOR_CANID);
        MotorThree = new TalonFX(Constants.Shooter.kSHOOTER_MotorThree_MOTOR_CANID);
        MotorFour = new TalonFX(Constants.Shooter.kSHOOTER_MotorFour_MOTOR_CANID);

        // Populate distance-to-RPM look-up table (meters -> RPM)
        distanceToRPM.put(2.49493587092, 1250.0);
        distanceToRPM.put(3.03308176613, 1375.0+15);
        distanceToRPM.put(1.6346195276, 1075.0-25);
        distanceToRPM.put(4.10526503, 1575.0+25);
        distanceToRPM.put(5.34766117, 1750.0+40);
        distanceToRPM.put(10.5, 2400.0); // TODO: REDO
        
        configFlywheel();
    }

    private void configFlywheel() {
        // Configure current limits and neutral mode
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.StatorCurrentLimit = 100;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure PID control loop coefficients
        shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
        shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV;
        shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
        shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP; 
        shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
        shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD; 

        MotorOne.getConfigurator().apply(shooterConfig);
        MotorTwo.getConfigurator().apply(shooterConfig);
        MotorThree.getConfigurator().apply(shooterConfig);
        MotorFour.getConfigurator().apply(shooterConfig);
        
        // Synchronize bottom flywheel to top flywheel
        MotorTwo.setControl(new Follower(MotorOne.getDeviceID(), MotorAlignmentValue.Aligned));
        MotorThree.setControl(new Follower(MotorOne.getDeviceID(), MotorAlignmentValue.Aligned));
        MotorFour.setControl(new Follower(MotorOne.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    
    @Deprecated
    public void set(double speed) {
        MotorOne.set(speed);
        isShooting = speed!=0;
    }

    /** @param rpm Target velocity for both flywheels */
    public void setRPM(double rpm) {
        this.desiredSpeed = rpm;
        MotorOne.setControl(m_request.withVelocity(rpm / 60.0));
        isShooting = rpm!=0;
    }

    /** @return Average current RPM of both flywheels */
    public double flywheelRPM() {
        return (MotorOne.getVelocity().getValue().in(RPM) + MotorTwo.getVelocity().getValue().in(RPM) + MotorThree.getVelocity().getValue().in(RPM) + MotorFour.getVelocity().getValue().in(RPM)) / 4;
    }
  
    /** @return true if flywheels are within the tolerance of the target RPM */
    public boolean atDesiredRPM() {
        return Math.abs(flywheelRPM() - desiredSpeed) < 75;
    }

    /** 
     * Sets target RPM based on distance to hub.
     * @param meters Distance to target in meters
     */
    public void shootMeters(double meters) {
        double targetRPM = distanceToRPM.get(meters);
        setRPM(targetRPM);
    }

    /** 
     * Calibration utility for autonomous logic.
     * @param meters Distance to target in meters
     * @return Required RPM from the look-up table
     */
    public double getDistanceRPM (double meters) {
        return distanceToRPM.get(meters);
    }

    /** Stops both flywheels */
    public void stop() {
        this.desiredSpeed = 0;
        MotorOne.setControl(voltageRequest.withOutput(0));
    }

    /** @param volts Direct voltage output for manual testing */
    public void setVolts(double volts) {
        MotorOne.setControl(voltageRequest.withOutput(volts));
        isShooting = volts!=0;
    }

    @Override
    public void periodic() {
      updateFuelShot();
      // Telemetry logging for dashboard and diagnostics
      SmartDashboard.putNumber("Shooter/Fuel Shot", fuelShot);
      SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
      SmartDashboard.putNumber("Shooter/Top Motor Stator Current", MotorOne.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Stator Current", MotorTwo.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Stator Current", MotorThree.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Stator Current", MotorFour.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Supply Current", MotorOne.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Current", MotorTwo.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Supply Current", MotorThree.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Current", MotorFour.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Supply Voltage", MotorOne.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Voltage", MotorTwo.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Supply Voltage", MotorThree.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Voltage", MotorFour.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Voltage", MotorOne.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Voltage", MotorTwo.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Voltage", MotorThree.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Voltage", MotorFour.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Encoder Pos", MotorOne.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Encoder Pos", MotorTwo.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Encoder Pos", MotorThree.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Encoder Pos", MotorFour.getPosition().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/Top Motor Torque Current", MotorOne.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Torque Current", MotorTwo.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Torque Current", MotorThree.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Torque Current", MotorFour.getTorqueCurrent().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/Top Motor Device Temp", MotorOne.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Device Temp", MotorTwo.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Device Temp", MotorThree.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Device Temp", MotorFour.getDeviceTemp().getValueAsDouble());


      SmartDashboard.putNumber("Shooter/Top Motor Processor Temp", MotorOne.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Processor Temp", MotorTwo.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Processor Temp", MotorThree.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Processor Temp", MotorFour.getProcessorTemp().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/FlywheelRPM (Top)", MotorOne.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Bottom)", MotorTwo.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Top)", MotorThree.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Bottom)", MotorFour.getVelocity().getValue().in(RPM));


      SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());
      
      Alert.alertKraken(MotorOne);
      Alert.alertKraken(MotorTwo);
    }

    private void updateFuelShot () {
        double currentAmps = desiredSpeed - flywheelRPM();
        if (isShooting) {
            if (!hasGoneDown && currentAmps > lastRPM) {
                hasGoneDown = true;
                dipRPM += currentAmps - dipRPM;
            }
            if (hasGoneDown && currentAmps > lastRPM) {
                dipRPM += currentAmps - dipRPM;
            }
            if (hasGoneDown && currentAmps < lastRPM) {
                hasGoneDown = false;
                if (dipRPM > 100)
                    fuelShot++;
                dipRPM = 0;
            }
        }
        else {
            dipRPM = 0;
            hasGoneDown = false;
        }
        lastRPM = currentAmps;
    }

    /** 
     * Calculates time-of-flight based on projectile physics.
     * @param distanceMeters Distance to target
     * @return Estimated seconds until impact
     */
    public double getExpectedTOF(double distanceMeters) {
        return distanceMeters*0.215298795+0.753755412;
    }

}
