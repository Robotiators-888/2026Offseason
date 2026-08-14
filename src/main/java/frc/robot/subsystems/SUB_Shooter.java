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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;
import frc.robot.utils.TrajectorySolver;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;

    /** Subsystem hardware and control state */
    private final TalonFX MotorOne;
    private final TalonFX MotorTwo;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration shooterLowConfig = new TalonFXConfiguration();
    public static boolean isShooting = false;
    private boolean lastIsShooting = false;
    
    /** Interpolation map for distance-based RPM calibration */
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    /** @return Single instance of the SUB_Shooter subsystem */
    public static SUB_Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
        }
        return INSTANCE;
    }

    private SUB_Shooter() {
        // Initialize dual flywheel motors
        MotorOne = new TalonFX(Constants.Shooter.kSHOOTER_MotorOne_MOTOR_CANID); 
        MotorTwo = new TalonFX(Constants.Shooter.kSHOOTER_MotorTwo_MOTOR_CANID);

        // Populate distance-to-RPM look-up table (meters -> RPM)
        distanceToRPM.put(2.49493587092, 1250.0);
        distanceToRPM.put(3.03308176613, 1375.0 + 15);
        distanceToRPM.put(1.6346195276, 1075.0 - 25);
        distanceToRPM.put(4.10526503, 1575.0 + 25);
        distanceToRPM.put(5.34766117, 1750.0 + 40);
        distanceToRPM.put(10.5, 2400.0);
        
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

        shooterLowConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterLowConfig.CurrentLimits.StatorCurrentLimit = 100;
        shooterLowConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterLowConfig.CurrentLimits.SupplyCurrentLimit = 10;
        shooterLowConfig.CurrentLimits.SupplyCurrentLowerLimit = 5;
        shooterLowConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        shooterLowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterLowConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure PID control loop coefficients
        shooterLowConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
        shooterLowConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV;
        shooterLowConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
        shooterLowConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP; 
        shooterLowConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
        shooterLowConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD; 

        MotorOne.getConfigurator().apply(shooterConfig);
        MotorTwo.getConfigurator().apply(shooterConfig);
        
        // Synchronize bottom flywheel to top flywheel
        MotorTwo.setControl(new Follower(MotorOne.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * Calculates required shooter RPM using projectile physics.
     * @param distance Distance in meters
     * @param angleRadians Launch angle in radians
     * @return Target flywheel RPM
     */
    public static double findoptimalRPM(final double distance, final double angleRadians) {
        double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        double denom = 2.0 * (distance * Math.tan(angleRadians) - height);
        if (denom <= 0.001 || distance <= 0.1) {
            return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
        }
        
        double cosAngle = Math.cos(angleRadians);
        if (Math.abs(cosAngle) < 0.001) {
            return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
        }

        double exitVelocity = (1.0 / cosAngle) * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance) / denom);
        double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
        double surfaceSpeed = exitVelocity / Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
        double rps = surfaceSpeed / (Math.PI * wheelDiameterMeters);
        double exitRPM = rps * 60.0;
        
        if (Double.isNaN(exitRPM) || Double.isInfinite(exitRPM)) {
            return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
        }
        return exitRPM;
    }

    /**
     * Calculates trajectory setpoints prioritizing current flywheel RPM (hood-first).
     */
    public TrajectorySolver.TrajectoryResult getTrajectory(double distanceMeters) {
        double currentRPM = (this.desiredSpeed > 500) ? this.desiredSpeed : flywheelRPM();
        if (currentRPM < 500) {
            currentRPM = Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
        }
        return TrajectorySolver.calculateTrajectory(currentRPM, distanceMeters);
    }

    @Deprecated
    public void set(final double speed) {
        MotorOne.set(speed);
    }

    /** @param rpm Target velocity for both flywheels */
    public void setRPM(final double rpm) {
        this.desiredSpeed = rpm;
        MotorOne.setControl(m_request.withVelocity(rpm / 60.0));
    }

    /** @return Average current RPM of both flywheels */
    public double flywheelRPM() {
        return (MotorOne.getVelocity().getValue().in(RPM) + MotorTwo.getVelocity().getValue().in(RPM)) / 2.0;
    }
  
    /** @return true if flywheels are within the tolerance of the target RPM */
    public boolean atDesiredRPM() {
        return Math.abs(flywheelRPM() - desiredSpeed) < 75;
    }

    /** 
     * Sets target RPM based on distance to hub.
     * @param meters Distance to target in meters
     */
    public void shootMeters(final double meters) {
        double targetRPM = distanceToRPM.get(meters);
        setRPM(targetRPM);
    }

    /** 
     * Calibration utility for autonomous logic.
     * @param meters Distance to target in meters
     * @return Required RPM from the look-up table
     */
    public double getDistanceRPM(final double meters) {
        return distanceToRPM.get(meters);
    }

    /** Stops both flywheels */
    public void stop() {
        this.desiredSpeed = 0;
        MotorOne.setControl(voltageRequest.withOutput(0));
    }

    /** @param volts Direct voltage output for manual testing */
    public void setVolts(final double volts) {
        MotorOne.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void periodic() {
      // Telemetry logging for dashboard and diagnostics
      SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
      SmartDashboard.putNumber("Shooter/Motor One Stator Current", MotorOne.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Stator Current", MotorTwo.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Supply Current", MotorOne.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Supply Current", MotorTwo.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Supply Voltage", MotorOne.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Supply Voltage", MotorTwo.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Voltage", MotorOne.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Voltage", MotorTwo.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Encoder Pos", MotorOne.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Encoder Pos", MotorTwo.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Torque Current", MotorOne.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Torque Current", MotorTwo.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Device Temp", MotorOne.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Device Temp", MotorTwo.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Processor Temp", MotorOne.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Processor Temp", MotorTwo.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/FlywheelRPM (One)", MotorOne.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Two)", MotorTwo.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());
      
      Alert.alertKraken(MotorOne);
      Alert.alertKraken(MotorTwo);

      // Only apply configuration on state transition, not every tick
      if (SUB_Shooter.isShooting != lastIsShooting) {
        TalonFXConfiguration configToApply = SUB_Shooter.isShooting ? shooterConfig : shooterLowConfig;
        MotorOne.getConfigurator().apply(configToApply);
        MotorTwo.getConfigurator().apply(configToApply);
        lastIsShooting = SUB_Shooter.isShooting;
      }
    }

    /** 
     * Calculates time-of-flight based on projectile physics.
     * @param distanceMeters Distance to target
     * @return Estimated seconds until impact
     */
    public double getExpectedTOF(final double distanceMeters) {
        return distanceMeters * 0.215298795 + 0.753755412;
    }
}
