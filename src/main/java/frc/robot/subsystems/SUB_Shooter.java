package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;
    private TalonFX flyWheel1;
    private TalonFX flyWheel2;
    private SparkMax meteringWheel;
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }
    private SUB_Shooter(){
        flyWheel1 = new TalonFX(Constants.Shooter.kSHOOTER_FLYWHEEL1_MOTOR_CANID); 
        flyWheel2 = new TalonFX(Constants.Shooter.kSHOOTER_FLYWHEEL2_MOTOR_CANID);
        meteringWheel = new SparkMax(Constants.Shooter.kMETERING_WHEEL_CANID, MotorType.kBrushless);
        configFlywheel();
    }

    private void configFlywheel() {
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40; 
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; 
        shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
        shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV; // The rpm in the docs means the target rpm we want to reach on average, not that we should multiply the rpm in code. Wtih our previous code we would have tripped the breaker if it had worked...
        shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
        shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP; 
        shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
        shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD; 
        flyWheel1.getConfigurator().apply(shooterConfig);
        flyWheel2.getConfigurator().apply(shooterConfig);
        flyWheel2.setControl(new Follower(flyWheel1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Deprecated
    public void set(double speed){
        flyWheel1.set(speed);
    }

    public void setRPM(double rpm) {
        this.desiredSpeed = rpm;
        flyWheel1.setControl(m_request.withVelocity(rpm / 60.0));
    }

    public void setMeteringSpeed(double speed) {
        meteringWheel.set(speed);
    }

    public double flywheelRPM() {
        return (flyWheel1.getVelocity().getValue().baseUnitMagnitude()+flyWheel2.getVelocity().getValue().baseUnitMagnitude())/2;
    }
  
    public boolean atDesiredRPM() {
        return Math.abs(flywheelRPM() - desiredSpeed) < 100; // Allow a tolerance of 100 RPM
        // This logic makes absolutely now sense?: return flyWheel1.getMotionMagicIsRunning().getValue(); //TODO: Is this correct for flywheel rpm speed?
    }

    public void shootMeters(double meters) { //TODO: Make a Trapezoidal Motion Profile for shooting at different distances, and test it to find the right values for kS, kV, and kA
        // Example implementation for shooting at a specific distance in meters
        // This would be replaced with actual logic based on distance and shooter characteristics
        setRPM(1000); // Example RPM value for shooting at 1 meter distance
    }

    public void periodic() {
      SmartDashboard.putNumber("FlywheelRPM", flywheelRPM());
    }
}