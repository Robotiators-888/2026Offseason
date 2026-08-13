// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        /** Controller port mapping and driver preferences */
        public static class Operator {
                public static final int kDriver1ControllerPort = 0;
                public static final int kDriver2ControllerPort = 1;
                public static final double kDriveDeadband = 0.1;
        }

        /** Shooter motor IDs and PID/Feedforward tuning (Manually Tuned) */
        public static final class Shooter {
                public static final int kSHOOTER_MotorTwo_MOTOR_CANID = 44;
                public static final int kSHOOTER_MotorOne_MOTOR_CANID = 43;
                public static final double kSHOOTER_FLYWHEEL_RPM = 1000;

                //Physical Specs
                public static final double ShooterDiameter = 3;                
                // Feedforward constants (Manual tuning in progress)
                public static final double kSHOOTER_FLYWHEEL_kS = 0.0;
                public static final double kSHOOTER_FLYWHEEL_kV = 0.14; 
                public static final double kSHOOTER_FLYWHEEL_kA = 0.0;
                
                // PID constants
                public static final double kSHOOTER_FLYWHEEL_kP = 0.5;
                public static final double kSHOOTER_FLYWHEEL_kI = 0;
                public static final double kSHOOTER_FLYWHEEL_kD = 0;

                public static final double kSHOOTER_COMPRESSION_RATIO = .8; // Todo find ratio

                public static final double kGRAVITATIONAL_CONSTANT = 9.80665;
        }

        /** Intake roller motor configuration */
        public static final class Roller {
                public static final int kINTAKE_LEFTMOTOR_CANID = 30; 
                public static final int kINTAKE_RIGHTMOTOR_CANID = 31;
                public static final double kROLLER_MOTOR_SPEED = 0.9;
                public static final double kROLLER_MOTOR_VOLTAGE = 10.91276304645254; 
        }

        /** Intake arm motor configuration and relative setpoints (NEO 2.0 Encoder) */
        public static final class Linear {
                public static final int kLINEAR_MOTOR_CANID = 31; 
                public static final double kLINEAR_MOTOR_SPEED = 0.1; 
                public static final double kLINEAR_FORWARD_SETPOINT = 360.0*4.2; // Degrees (Relative)
                public static final double kLINEAR_BACKWARD_SETPOINT = 0;       // Degrees (Relative)
                public static final double kLINEAR_FAULT_AMPS = 30;       // Stall detection threshold
                public static final PIDController kLINEAR_FAST_PID_CONTROLLER = new PIDController(4, 0, 0); // Change the values!!!
                public static final PIDController kLINEAR_SLOW_PID_CONTROLLER = new PIDController(1, 0, 0); // Change the values!!!
        }
        
        /** Indexing system: Spindexer and Metering wheel */
        public static final class Index {
                public static final int KINDEX_MOTOR_CANID = 41;      // Spindexer motor
                public static final int kMETERING_WHEEL_CANID = 42;   // High-speed feed to shooter
                
                public static final double kINDEX_MOTOR_VOLTS = 8.5;    // Spindexer feed voltage
                public static final double kINDEX_METERING_MOTOR_VOLTS = 8.0; 
                
                // Max RPM of NEO at 12V is 5676; scaled by target voltage
                public static final double kINDEX_METERING_MOTOR_RPM = 5676*(kINDEX_METERING_MOTOR_VOLTS/12.0); 
        }

        /** Standard field measurements in meters */
        public static final class Field {
                public static final double fieldLength = 1653.2 / 100.0; 
                public static final double fieldWidth = 800.1 / 100.0; 
        }

        /** Vision camera names and physical transformations on the robot */
        public static final class PhotonVision {
                public static final String kCamName1 = "BackLeftCam";
                public static final double kMaxZError = 0.2;
                public static final double kMaxAmbiguity = 0.1;
                public static final double kMaxDistance = 12.0;

                public static final Rotation3d cameraRotation = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(-25),
                                Units.degreesToRadians(172-90));
                public static final Transform3d kRobotToCamera1 = new Transform3d(
                                Units.inchesToMeters(-11.55), Units.inchesToMeters(10.5),
                                Units.inchesToMeters(7.8), cameraRotation);

                public static final String kCam2Name = "BackRightCam"; 
                public static final Rotation3d cameraRotation2 = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(-25),
                                Units.degreesToRadians(-172+90));
                public static final Transform3d kRobotToCamera2 = new Transform3d(
                                Units.inchesToMeters(-11.55), Units.inchesToMeters(-10.5),
                                Units.inchesToMeters(7.8), cameraRotation2);


                public static final String kCam3Name = "HighCam";
                public static final Rotation3d cameraRotation3 = new Rotation3d(Units.degreesToRadians(3),
                                 Units.degreesToRadians(-1.5), Units.degreesToRadians(-4));
                public static final Transform3d kRobotToCamera3 = new Transform3d(
                                 Units.inchesToMeters(-4), Units.inchesToMeters(0),
                                 Units.inchesToMeters(20.5), cameraRotation3);
        }

        /** PWM configurations and predefined color values for Blinkin */
        public static class LEDs {
                public static final int kPWMPort = 9;
                public static final double kColorGreen = 0.77;
                public static final double kColorRed = 0.61;
                public static final double kParty_Palette_Twinkles = -0.53;
                public static final double kAllianceColor = (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)== DriverStation.Alliance.Blue) ? 0.0 : 0.5;
        }

        public static class Hood {
                public static final int kHOOD_CAN_ID = 47;
                public static final PIDController kHOOD_PID_CONTROLLER = new PIDController(4,0,0); // Change the values!!!
                public static final double ScoreHeight = 55;
        }
        public static class Metering {
                public static final int kMETERING_MOTOR_CAN_ID = 45;
                public static final int kMETERING_MOTOR_FOLLOWER_CAN_ID = 46;
        }
}
