package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;
    private TalonFX flyWheel1;
    private TalonFX flyWheel2;
    // private double desiredSpeed = 0;
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }
    private SUB_Shooter(){
        flyWheel1 = new TalonFX(Constants.Shooter.kSHOOTER_FLYWHEEL1_MOTOR_CANID); // Might need a CANBus
        flyWheel2 = new TalonFX(Constants.Shooter.kSHOOTER_FLYWHEEL2_MOTOR_CANID);
        flyWheel2.setControl(new Follower(flyWheel1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    // @Deprecated
    // private void set(double speed){
    //     flyWheel1.set(speed);
    // }

    public void setRPM(double rpm) {
        Slot0Configs slotConfig = talonConfig.Slot0;
        slotConfig.kS = 0.25;
        slotConfig.kV = rpm/60.0*0.12;
        slotConfig.kA = rpm/60.0*0.01;
        slotConfig.kP = 0; //TODO: After testing SVA, test PID, default is 4.8
        slotConfig.kI = 0;
        slotConfig.kD = 0; //TODO: After testing SVA, test PID, default is rpm/60.0*0.1

        MotionMagicConfigs motionConfig = talonConfig.MotionMagic;
        motionConfig.MotionMagicCruiseVelocity = rpm/60.0;
        motionConfig.MotionMagicAcceleration = rpm/60.0/5.0;
        motionConfig.MotionMagicJerk = 1600;
        flyWheel1.getConfigurator().apply(talonConfig);
    }

    public double flywheelRPM() {
        return (flyWheel1.getVelocity().getValue().baseUnitMagnitude()+flyWheel2.getVelocity().getValue().baseUnitMagnitude())/2;
    }
  
    public boolean atdesiredRPM() {
        //return flywheelRPM()>= desiredSpeed;
        // return (flyWheel1.getMotionMagicIsRunning().getValue() && flyWheel2.getMotionMagicIsRunning().getValue());
        return flyWheel1.getMotionMagicIsRunning().getValue(); //TODO: Is this correct for flywheel rpm speed?
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