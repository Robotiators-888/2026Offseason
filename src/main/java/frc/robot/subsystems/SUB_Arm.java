package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SUB_Arm extends SubsystemBase {
    // Initiliazes values and objects used in subsystem
    public static boolean extended;
    // Mr Lange said to increase pid from .001 to .01
    private PIDController controller = new PIDController(0.005, 0, 0);
    private SparkMax arm;
    private SparkMax armFollower;
    private boolean stickUp = false;
    private boolean stickDown = false;
    private int periodicCountFault = 0;
    private static SUB_Arm INSTANCE = null;
    public static SUB_Arm getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Arm();
        } 
        return INSTANCE;
    }

    private SUB_Arm () {
        //Defines motors with IDs and what controller
        arm = new SparkMax(Constants.Arm.kARM_MOTOR_CANID, MotorType.kBrushless);
        armFollower = new SparkMax(Constants.Arm.kARM_FOLLOWER_MOTOR_CANID, MotorType.kBrushless);
        configureMotors();
    }

    private void configureMotors(){
        //Creates config for motors
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(360.0 / 23); // Converts rotations to degrees, Thrifty bot cycloial gearbox 23:1
        config.encoder.velocityConversionFactor((360.0 / 23) / 60.0); // Converts RPM to deg/sec
        config.smartCurrentLimit(35); //Sets stall limit for motor in amps
        config.inverted(true); //Inverts motor
        arm.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters); //Sets persist parameters
        SparkMaxConfig followerConfig = new SparkMaxConfig(); //Creates follower spark max config
        followerConfig.follow(arm, true); // Makes follower opposite compared to leader
        followerConfig.smartCurrentLimit(35);//Sets stall limit for motor in amps
        armFollower.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);  //Sets persist parameters
    }



    // Logs everything every periodic
    public void periodic() {
        if (RobotContainer.highShouldAlert) {
            
            SmartDashboard.putNumber("Arm/Arm Encoder Pos", arm.getEncoder().getPosition()); //Returns angle of intake arm

            SmartDashboard.putNumber("Arm/Arm Output Current", arm.getOutputCurrent()); //Returns how much current is going into the intake arm motors

            SmartDashboard.putBoolean("Arm/Stick Up", stickUp);  
            SmartDashboard.putBoolean("Arm/Stick Down", stickDown);
        }
        if (RobotContainer.slowShouldAlert) {
            SmartDashboard.putNumber("Arm/Arm Bus Voltage", arm.getBusVoltage());

            SmartDashboard.putNumber("Arm/Arm Motor Temp", arm.getMotorTemperature());
        }
        if (periodicCountFault > 0) {
            periodicCountFault--;
        }
    }

    //sets arm to speed put in method
    public void setArm(double speed) {
        if (arm.getOutputCurrent() > Constants.Arm.kARM_FAULT_AMPS) {
            periodicCountFault+=2;
        }
        if (periodicCountFault > 12) {
            //if speed is going up when faults are high the arm is up
            if (speed > 0) {
                stickUp = true;
                stickDown = false;
                arm.getEncoder().setPosition(Constants.Arm.kARM_TOP_SETPOINT);
            //If speed is negative when faults are high the arm is down
            } else if (speed < 0) {
                stickUp = false;
                stickDown = true;
                arm.getEncoder().setPosition(Constants.Arm.kARM_BOTTOM_SETPOINT);
            }
            speed = 0;
        }
        //if arm is up and it moves down then it is set to no longer being up
        if (stickUp) {
            if (speed < 0) {
                stickUp = false;
            } else {
                speed = 0;
            }
        }
        //if arm is down and it moves up then it is set to being no longer down.
        if (stickDown) {
            if (speed > 0) {
                stickDown = false;
            } else {
                speed = 0;
            }
        }
        //sets speed of arm motor
        arm.set(speed);
        
    }


    //sets arm to speed put in method
    public void setArmSlick(double speed) {
        if (arm.getOutputCurrent() > Constants.Arm.kARM_FAULT_AMPS) {
            periodicCountFault+=2;
        }
        if (periodicCountFault > 12) {
            //if speed is going up when faults are high the arm is up
            if (speed > 0) {
                stickUp = true;
                stickDown = false;
                arm.getEncoder().setPosition(Constants.Arm.kARM_TOP_SETPOINT);
            //If speed is negative when faults are high the arm is down
            } else if (speed < 0) {
                stickUp = false;
                stickDown = true;
                arm.getEncoder().setPosition(Constants.Arm.kARM_BOTTOM_SETPOINT);
            }
            speed = 0;
        }
        //sets speed of arm motor
        arm.set(speed);
        
    }

    //Retuns if arm is extended
    public boolean isExtended() {
        return extended;
    }

    //Makes arm go down based on PID
    public void intakeArmDown() {
        setArmSlick(controller.calculate(arm.getEncoder().getPosition(), Constants.Arm.kARM_BOTTOM_SETPOINT)); 
    }
    public void intakeArmTest() {
        arm.set(-.6);
    }

    //Makes arm go up based on PID
    public void intakeArmUp() {
        setArm(controller.calculate(arm.getEncoder().getPosition(), Constants.Arm.kARM_TOP_SETPOINT)); 
    }

    //Returns if arm is down
    public boolean isArmDownReached() {
        return Math.abs(arm.getEncoder().getPosition() - Constants.Arm.kARM_BOTTOM_SETPOINT) < 3.0;
    }

    //Returns if arm is up
    public boolean isArmUpReached() {
        return Math.abs(arm.getEncoder().getPosition() - Constants.Arm.kARM_TOP_SETPOINT) < 3.0;
    }
}
