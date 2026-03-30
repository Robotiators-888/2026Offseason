package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Arm extends SubsystemBase {
    /** Subsystem state and configuration constants */
    public static boolean extended;
    private PIDController controller = new PIDController(0.001, 0, 0);
    private SparkMax arm;
    private SparkMax armFollower;
    private boolean stickUp = false;
    private boolean stickDown = false;
    private int periodicCountFault = 0;
    private static SUB_Arm INSTANCE = null;

    /**
     * @return Single instance of the SUB_Arm subsystem
     */
    public static SUB_Arm getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Arm();
        } 
        return INSTANCE;
    }

    private SUB_Arm () {
        // Defines motors with IDs and motor type
        arm = new SparkMax(Constants.Arm.kARM_MOTOR_CANID, MotorType.kBrushless);
        armFollower = new SparkMax(Constants.Arm.kARM_FOLLOWER_MOTOR_CANID, MotorType.kBrushless);
        configureMotors();
    }

    private void configureMotors(){
        // Creates config for motor and encoder (23:1 cycloidal gearbox)
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(360.0 / 23); // Converts rotations to degrees
        config.encoder.velocityConversionFactor((360.0 / 23) / 60.0); // Converts RPM to deg/sec
        config.smartCurrentLimit(35); // Sets stall limit in amps
        config.inverted(true);
        arm.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        // Configure follower motor
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(arm, true); // Opposite direction compared to leader
        followerConfig.smartCurrentLimit(35);
        armFollower.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    /** @return true if the arm has reached the top setpoint or is stuck up */
    public boolean isForwardPressed() {
        return stickUp||Math.abs(arm.getEncoder().getPosition()-Constants.Arm.kARM_TOP_SETPOINT)<10;
    }

    /** @return true if the arm has reached the bottom setpoint or is stuck down */
    public boolean isReversePressed() {
        return stickDown||Math.abs(arm.getEncoder().getPosition()-Constants.Arm.kARM_BOTTOM_SETPOINT)<10;
    }


    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putBoolean("Arm/Arm Forward Limit", isForwardPressed());
        SmartDashboard.putBoolean("Arm/Arm Reverse Limit", isReversePressed());
        SmartDashboard.putNumber("Arm/Arm Encoder Pos", arm.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm/Arm Output Current", arm.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Arm Bus Voltage", arm.getBusVoltage());
        SmartDashboard.putNumber("Arm/Arm Motor Temp", arm.getMotorTemperature());

        SmartDashboard.putNumber("Arm/Arm Follower Encoder Pos", armFollower.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm/Arm Follower Output Current", armFollower.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Arm Follower Bus Voltage", armFollower.getBusVoltage());
        SmartDashboard.putNumber("Arm/Arm Follower Motor Temp", armFollower.getMotorTemperature());

        SmartDashboard.putBoolean("Arm/Stick Up", stickUp);  
        SmartDashboard.putBoolean("Arm/Stick Down", stickDown);

        // Decay the fault counter over time
        if (periodicCountFault > 0) {
            periodicCountFault--;
        }

        Alert.alertNeoFaults(arm);
        Alert.alertNeoWarnings(arm);
        Alert.alertNeoFaults(armFollower);
        Alert.alertNeoWarnings(armFollower);
    }

    /**
     * Sets the arm motor speed with stall detection and limit protection.
     * @param speed Motor percent output [-1.0, 1.0]
     */
    public void setArm(double speed) {
        if (arm.getOutputCurrent() > Constants.Arm.kARM_FAULT_AMPS) {
            periodicCountFault+=2;
        }

        // Logic to redefine zero position upon physical stall (soft limit calibration)
        if (periodicCountFault > 12) {
            if (speed > 0) {
                stickUp = true;
                stickDown = false;
                arm.getEncoder().setPosition(Constants.Arm.kARM_TOP_SETPOINT);
            } else if (speed < 0) {
                stickUp = false;
                stickDown = true;
                arm.getEncoder().setPosition(Constants.Arm.kARM_BOTTOM_SETPOINT);
            }
            speed = 0;
        }

        // Prevent movement beyond soft limits
        if (stickUp) {
            if (speed < 0) {
                stickUp = false;
            } else {
                speed = 0;
            }
        }
        if (stickDown) {
            if (speed > 0) {
                stickDown = false;
            } else {
                speed = 0;
            }
        }
        arm.set(speed);
    }

    public boolean isExtended() {
        return extended;
    }

    /** Drives the arm to the bottom setpoint using PID */
    public void intakeArmDown() {
        setArm(controller.calculate(arm.getEncoder().getPosition(), Constants.Arm.kARM_BOTTOM_SETPOINT)); 
    }

    /** Manual test drive for the arm */
    public void intakeArmTest() {
        arm.set(-.3);
    }

    /** Drives the arm to the top setpoint using PID */
    public void intakeArmUp() {
        setArm(controller.calculate(arm.getEncoder().getPosition(), Constants.Arm.kARM_TOP_SETPOINT)); 
    }

    public boolean isArmDownReached() {
        return Math.abs(arm.getEncoder().getPosition() - Constants.Arm.kARM_BOTTOM_SETPOINT) < 3.0;
    }

    public boolean isArmUpReached() {
        return Math.abs(arm.getEncoder().getPosition() - Constants.Arm.kARM_TOP_SETPOINT) < 3.0;
    }
}
