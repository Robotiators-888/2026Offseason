package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;
    private TalonFX topFlywheel;
    private TalonFX bottomFlywheel;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    private BooleanSupplier canShoot;
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }
    private SUB_Shooter(){
        canShoot = () -> true;
        topFlywheel = new TalonFX(Constants.Shooter.kSHOOTER_topFlywheel_MOTOR_CANID); 
        bottomFlywheel = new TalonFX(Constants.Shooter.kSHOOTER_bottomFlywheel_MOTOR_CANID);
        distanceToRPM.put(Units.inchesToMeters(64), 1241.0);
        distanceToRPM.put(Units.inchesToMeters(95), 1430.0);
        distanceToRPM.put(Units.inchesToMeters(129), 1660.0);
        configFlywheel();
    }

    private void configFlywheel() {
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40; 
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 2; 
        shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
        shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV; // The rpm in the docs means the target rpm we want to reach on average, not that we should multiply the rpm in code. Wtih our previous code we would have tripped the breaker if it had worked...
        shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
        shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP; 
        shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
        shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD; 
        topFlywheel.getConfigurator().apply(shooterConfig);
        bottomFlywheel.getConfigurator().apply(shooterConfig);
        bottomFlywheel.setControl(new Follower(topFlywheel.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Deprecated
    public void set(double speed){
        if (canShoot.getAsBoolean()) { 
            topFlywheel.set(speed);
        }
        else {
            stop();
        }
    }

    public void setRPM(double rpm) {
        if (canShoot.getAsBoolean()) { 
            this.desiredSpeed = rpm;
            topFlywheel.setControl(m_request.withVelocity(rpm / 60.0));
        }
        else {
            stop();
        }
    }

    public double flywheelRPM() {
        return topFlywheel.getVelocity().getValue().in(RPM);
    }
  
    public boolean atDesiredRPM() {
        // return true;
        return Math.abs(flywheelRPM() - desiredSpeed) < 75; // Allow a tolerance of 50 RPM
        // This logic makes absolutely now sense?: return topFlywheel.getMotionMagicIsRunning().getValue(); //TODO: Is this correct for flywheel rpm speed?
    }

    public void shootMeters(double meters) {
        // query the map for the RPM associated with this distance
        double targetRPM = distanceToRPM.get(meters);
        
        // Pass it to your existing setRPM method
        setRPM(targetRPM);
    }

    public void stop() {
        this.desiredSpeed = 0;
        topFlywheel.setControl(voltageRequest.withOutput(0));
    }

    public void setVolts(double volts) {
        if (canShoot.getAsBoolean()) {
            topFlywheel.setControl(voltageRequest.withOutput(volts));
        }
        else {
            stop();
        }
    }

    public double getCurrentDrawTop () {
        return topFlywheel.getStatorCurrent().getValueAsDouble();
    }

    public void stopIfFalse (BooleanSupplier supplier) {
        canShoot = supplier;
        stop();
    }

    public void periodic() {
      SmartDashboard.putNumber("FlywheelRPM", flywheelRPM());
      SmartDashboard.putNumber("Desired RPM", desiredSpeed);
      SmartDashboard.putNumber("Top Motor Amperage", getCurrentDrawTop());
    }
}
