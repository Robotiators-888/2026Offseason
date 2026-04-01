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
    private TalonFX topFlywheel;
    private TalonFX bottomFlywheel;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private double fuelShot = 0;
    private double lastAmps = 0;
    private double spikeAmps = 0;
    private boolean hasGoneUp = false;
    
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
        topFlywheel = new TalonFX(Constants.Shooter.kSHOOTER_topFlywheel_MOTOR_CANID); 
        bottomFlywheel = new TalonFX(Constants.Shooter.kSHOOTER_bottomFlywheel_MOTOR_CANID);

        // Populate distance-to-RPM look-up table (meters -> RPM)
        distanceToRPM.put(2.49493587092, 1250.0);
        distanceToRPM.put(3.03308176613, 1375.0+15);
        distanceToRPM.put(1.6346195276, 1075.0-25);
        distanceToRPM.put(4.10526503, 1575.0+25);
        distanceToRPM.put(5.34766117, 1750.0+40);
        distanceToRPM.put(10.5, 2400.0); // TODO: Field test required
        
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

        topFlywheel.getConfigurator().apply(shooterConfig);
        bottomFlywheel.getConfigurator().apply(shooterConfig);
        
        // Synchronize bottom flywheel to top flywheel
        bottomFlywheel.setControl(new Follower(topFlywheel.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    
    @Deprecated
    public void set(double speed) {
        topFlywheel.set(speed);
    }

    /** @param rpm Target velocity for both flywheels */
    public void setRPM(double rpm) {
        this.desiredSpeed = rpm;
        topFlywheel.setControl(m_request.withVelocity(rpm / 60.0));
    }

    /** @return Average current RPM of both flywheels */
    public double flywheelRPM() {
        return (topFlywheel.getVelocity().getValue().in(RPM) + bottomFlywheel.getVelocity().getValue().in(RPM)) / 2;
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
        topFlywheel.setControl(voltageRequest.withOutput(0));
    }

    /** @param volts Direct voltage output for manual testing */
    public void setVolts(double volts) {
        topFlywheel.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void periodic() {
      updateFuelShot();
      // Telemetry logging for dashboard and diagnostics
      SmartDashboard.putNumber("Shooter/Fuel Shot", fuelShot);
      SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
      SmartDashboard.putNumber("Shooter/Top Motor Stator Current", topFlywheel.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Stator Current", bottomFlywheel.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Supply Current", topFlywheel.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Current", bottomFlywheel.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Supply Voltage", topFlywheel.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Supply Voltage", bottomFlywheel.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Voltage", topFlywheel.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Voltage", bottomFlywheel.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Encoder Pos", topFlywheel.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Encoder Pos", bottomFlywheel.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Torque Current", topFlywheel.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Torque Current", bottomFlywheel.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Device Temp", topFlywheel.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Device Temp", bottomFlywheel.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Top Motor Processor Temp", topFlywheel.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Bottom Motor Processor Temp", bottomFlywheel.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Top)", topFlywheel.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Bottom)", bottomFlywheel.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());
      
      Alert.alertKraken(topFlywheel);
      Alert.alertKraken(bottomFlywheel);
    }

    private void updateFuelShot () {
        double currentAmps = topFlywheel.getStatorCurrent().getValueAsDouble();
        if (!hasGoneUp && currentAmps > lastAmps) {
            hasGoneUp = true;
            spikeAmps += currentAmps - lastAmps;
        }
        if (hasGoneUp && currentAmps > lastAmps) {
            spikeAmps += currentAmps - lastAmps;
        }
        if (hasGoneUp && currentAmps < lastAmps) {
            hasGoneUp = false;
            if (spikeAmps > 50)
                fuelShot++;
            spikeAmps = 0;
        }
        lastAmps = currentAmps;
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
