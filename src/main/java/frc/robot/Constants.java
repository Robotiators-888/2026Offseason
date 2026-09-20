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
 * The Constants class provides a central location for robot-wide numerical and boolean constants.
 *
 * <p>All constants are declared globally (`public static final`). Every constant explicitly
 * documents its physical units (e.g. meters, volts, RPM, degrees, CAN IDs) and physical relevance.
 */
public final class Constants {
        private Constants() {}

        /** Controller port mapping and driver preference constants. */
        public static class Operator {
                private Operator() {}

                /** USB port index for primary driver Xbox controller. */
                public static final int kDriver1ControllerPort = 0;

                /** USB port index for secondary driver Xbox controller. */
                public static final int kDriver2ControllerPort = 1;

                /** Joystick deadband threshold (unitless, 0.0 to 1.0 scale). */
                public static final double kDriveDeadband = 0.1;
        }

        /** Shooter motor CAN IDs, flywheel specifications, and PIDF gain parameters. */
        public static final class Shooter {
                private Shooter() {}

                /** CAN ID for follower flywheel TalonFX motor controller. */
                public static final int kSHOOTER_FOLLOWER_MOTOR_CANID = 44;

                /** CAN ID for leader flywheel TalonFX motor controller. */
                public static final int kSHOOTER_LEADER_MOTOR_CANID = 43;

                /** Nominal flywheel rotational speed setpoint in RPM. */
                public static final double kSHOOTER_FLYWHEEL_RPM = 1000;

                /** Outer diameter of the shooter flywheel wheels in inches. */
                public static final double ShooterDiameter = 3;

                /** Static friction feedforward gain kS for shooter flywheel (amps). */
                public static final double kSHOOTER_FLYWHEEL_kS = 0; // Theoreticaly should be 0, but IRL should be set to the minimum amount of amps needed to turn the flywheel when we can test

                /** Velocity feedforward gain kV for shooter flywheel (amps per RPS). */
                public static final double kSHOOTER_FLYWHEEL_kV = 0.0;  // Theoreticaly should be 0, but IRL should be set to the minimum amount of amps to keep a consistnet speed when we can test

                /** Acceleration feedforward gain kA for shooter flywheel (amps per RPS/s). */
                public static final double kSHOOTER_FLYWHEEL_kA = 0; // Not considered by code

                /** Proportional gain kP for shooter flywheel closed-loop control. */
                public static final double kSHOOTER_FLYWHEEL_kP = 4.0; // Ths controls two motors, so the lets say the RPM drops 2 RPS when we shoot a ball, it will apply a total 16 Amps to correct. If we need to increaese it, I wouldn't go above 8.0 for P

                /** Integral gain kI for shooter flywheel closed-loop control. */
                public static final double kSHOOTER_FLYWHEEL_kI = 0.0;

                /** Derivative gain kD for shooter flywheel closed-loop control. */
                public static final double kSHOOTER_FLYWHEEL_kD = 0.0;

                /** Compression ratio applied to game piece during launch */
                public static final double kSHOOTER_COMPRESSION_RATIO = .8; // TODO: NEED TO CHANGE THIS ASAP. THIS DETERMINES SHOOTER ACCURACY AND IS THE ONLY VARIABLE THAT NEEDS TO BE TUNED

                /** Standard acceleration due to gravity in m/s^2. */
                public static final double kGRAVITATIONAL_CONSTANT = 9.80665;

                public static final double kStatorCurrentLimit = 100; // Amperes
                public static final double kSupplyCurrentLimit = 60; // Amperes
                public static final double kSupplyCurrentLowerLimit = 40; // Amperes
                public static final double kSupplyCurrentLowerTime = 1.0; // Seconds

                public static final double kLowSupplyCurrentLimit = 10; // Amperes
                public static final double kLowSupplyCurrentLowerLimit = 5; // Amperes
                public static final double kLowSupplyCurrentLowerTime = 1.0; // Seconds

                public static final double kRPMTolerance = 75; // RPM

                public static final double kRPMZone1 = 2200.0;
                public static final double kRPMZone2 = 2750.0;
                public static final double kRPMZone3 = 3750.0;
                public static final double kRPMIdle = kRPMZone2;

                public static final double kZone3ThresholdMeters = 6.0;
                public static final double kZone2InitialThresholdMeters = 3.2;
                public static final double kZone1To2HysteresisMeters = 3.35;
                public static final double kZone2To1HysteresisMeters = 3.05;
        }

        /** Intake roller motor CAN IDs, speeds, and target voltages. */
        public static final class Roller {
                private Roller() {}

                /** CAN ID for left roller SPARK Max motor controller. */
                public static final int kINTAKE_LEFTMOTOR_CANID = 30;

                /** CAN ID for right roller SPARK Max motor controller. */
                public static final int kINTAKE_RIGHTMOTOR_CANID = 31;

                /** Duty cycle speed output for intake roller motor (scaled -1.0 to 1.0). */
                public static final double kROLLER_MOTOR_SPEED = 0.9;

                /** Target output voltage for intake roller motors in volts. */
                public static final double kROLLER_MOTOR_VOLTAGE = 10.91276304645254;

                /** Target RPM for intake roller motors. */
                public static final double kROLLER_MOTOR_RPM = 1000;

                public static final double kSupplyCurrentLimit = 30; // Amperes
                public static final double kSupplyCurrentLowerLimit = 15; // Amperes
                public static final double kSupplyCurrentLowerTime = 1.0; // Seconds

                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                public static final double kP = 15.0; // For every drop in 60 RPM we pull 15 amps to garuntee it never falls low and always can keep pulling balls.
                public static final double kI = 0.0;
                public static final double kD = 0.0; // TODO: If the roller jitters, maybe add a D value of like 0.2 or 0.4
        }

        /**
         * Linear intake deploy motor CAN ID, position setpoints, stall limits, and PID
         * controllers.
         */
        public static final class Linear {
                private Linear() {}

                /** CAN ID for linear intake deploy SPARK Max motor controller. */
                public static final int kLINEAR_MOTOR_CANID = 32;

                /** Open-loop speed factor for linear intake deployment (scaled -1.0 to 1.0). */
                public static final double kLINEAR_MOTOR_SPEED = 0.1;

                /** Extended forward setpoint position in relative encoder rotations. */
                public static final double kLINEAR_FORWARD_SETPOINT = 4.2;

                /** Retracted backward setpoint position in relative encoder rotations. */
                public static final double kLINEAR_BACKWARD_SETPOINT = 0;

                /** Motor stall detection threshold limit in amperes. */
                public static final double kLINEAR_FAULT_AMPS = 30;

                /** High-speed positional PID controller for linear intake movement. */
                public static final PIDController kLINEAR_PID_CONTROLLER =
                    new PIDController(1.0, 0, 0); // 1.0 means that if the motor is off 1 rotation from its target, we will put the full 12 V to get it back. TODO: Maybe make this less for a less agressive PID since it is only lik 4.5. If there is oscillation, just lower the P

                public static final double kLinearAgitatePeriodics =
                    100; // Number of periodic cycles for linear intake movement

                public static final int kStallLimit = 35; // Amperes
                public static final int kFreeLimit = 5; // Amperes

                public static final double kTolerance =
                    0.1; // Tolerance for linear intake position in rotations
        }

        /** Spindexer and metering wheel CAN IDs, voltage limits, and calculated RPM values. */
        public static final class Index {
                private Index() {}

                /** CAN ID for spindexer motor controller. */
                public static final int kINDEX_MOTOR_CANID = 41;

                /** CAN ID for high-speed metering feed motor controller. */
                public static final int kINDEX_FOLLOWER_WHEEL_CANID = 42;

                /** Operating voltage for spindexer motor in volts. */
                public static final double kINDEX_MOTOR_VOLTS = 8.5;

                public static final int kSmartCurrentLimit = 15; // Amps
        }

        /** Field geometry and physical dimension constants in meters. */
        public static final class Field {
                private Field() {}

                /** Total length of the competition field along X axis in meters. */
                public static final double fieldLength = 1653.2 / 100.0;

                /** Total width of the competition field along Y axis in meters. */
                public static final double fieldWidth = 800.1 / 100.0;
        }

        /** Vision camera names, filtering thresholds, and 3D transform configurations. */
        public static final class PhotonVision {
                private PhotonVision() {}

                /** Device name of camera 1 on local network. */
                public static final String kCamName1 = "BackLeftCam";

                /** Maximum acceptable Z offset error in meters. */
                public static final double kMaxZError = 0.2;

                /** Maximum acceptable target pose ambiguity ratio (0.0 to 1.0). */
                public static final double kMaxAmbiguity = 0.1;

                /** Maximum camera detection range in meters. */
                public static final double kMaxDistance = 12.0;

                /** 3D rotation of back-left camera relative to robot frame (radians). */ // TODO: GET THESE VALUES FROM THE CAD. WE DON'T HAVE TIME TO MEASUR THIS ON THE FIELD
                // public static final Rotation3d cameraRotation =
                //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-25),
                //         Units.degreesToRadians(172 - 90));

                // /** Physical 3D offset transform of back-left camera from robot center. */
                // public static final Transform3d kRobotToCamera1 =
                //     new Transform3d(Units.inchesToMeters(-11.55), Units.inchesToMeters(10.5),
                //         Units.inchesToMeters(7.8), cameraRotation);

                /** Device name of camera 2 on local network. */
                public static final String kCam2Name = "BackRightCam";

                // /** 3D rotation of back-right camera relative to robot frame (radians). */
                // public static final Rotation3d cameraRotation2 =
                //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-25),
                //         Units.degreesToRadians(-172 + 90));

                // /** Physical 3D offset transform of back-right camera from robot center. */
                // public static final Transform3d kRobotToCamera2 =
                //     new Transform3d(Units.inchesToMeters(-11.55), Units.inchesToMeters(-10.5),
                //         Units.inchesToMeters(7.8), cameraRotation2);

                /** Device name of camera 3 on local network. */
                public static final String kCam3Name = "HighCam";

                // /** 3D rotation of high camera relative to robot frame (radians). */
                // public static final Rotation3d cameraRotation3 =
                //     new Rotation3d(Units.degreesToRadians(3), Units.degreesToRadians(-1.5),
                //         Units.degreesToRadians(-4));

                // /** Physical 3D offset transform of high camera from robot center. */
                // public static final Transform3d kRobotToCamera3 =
                //     new Transform3d(Units.inchesToMeters(-4), Units.inchesToMeters(0),
                //         Units.inchesToMeters(20.5), cameraRotation3);
        }

        /** PWM port assignments and preset Blinkin LED pattern code constants. */
        public static class LEDs {
                private LEDs() {}

                /** PWM port assignment on roboRIO for LED light controller. */
                public static final int kPWMPort = 9;

                /** Blinkin PWM signal pulse value for Solid Green pattern. */
                public static final double kColorGreen = 0.77;

                /** Blinkin PWM signal pulse value for Solid Red pattern. */
                public static final double kColorRed = 0.61;

                /** Blinkin PWM signal pulse value for Party Palette Twinkles pattern. */
                public static final double kParty_Palette_Twinkles = -0.53;

                /** Dynamic PWM signal value based on current FMS alliance color assignment. */
                public static final double kAllianceColor =
                    (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Blue)
                    ? 0.0
                    : 0.5;
        }

        /** Adjustable hood CAN ID, PID gains, and scoring target height. */
        public static class Hood {
                private Hood() {}

                /** CAN ID for adjustable hood SPARK Max motor controller. */
                public static final int kHOOD_CAN_ID = 47;

                /** Positional PID controller for hood angle control. */
                public static final PIDController kHOOD_PID_CONTROLLER = new PIDController(4, 0, 0);

                /** Scoring target height in inches. */
                public static final double ScoreHeight = 55;

                public static final double kStatorCurrentLimit = 25; // Amperes
                public static final double kSupplyCurrentLimit = 7; // Amperes
                public static final double kSupplyCurrentLowerLimit = 5; // Amperes
                public static final double kSupplyCurrentLowerTime = 0.5; // Seconds

                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                public static final double kP = 100.0; // For every 30 degrees off we output 25 amps to instanly jump to the position
                public static final double kI = 0.0;
                public static final double kD = 4.0; // to dampen the rapid speeds this will move at and prevent overshooting
                public static final double kG = 0; // If you want this, make sure to chnage the code to make it cosine gravity

                public static final double kHoodTolerance =
                    0.05; // Tolerance for hood position in rotations
        }

        /** Metering system motor CAN IDs. */
        public static class Metering {
                private Metering() {}

                /** CAN ID for primary metering motor controller. */
                public static final int kMETERING_MOTOR_CAN_ID = 45;

                /** CAN ID for follower metering motor controller. */
                public static final int kMETERING_MOTOR_FOLLOWER_CAN_ID = 46;

                public static final double kStatorCurrentLimit = 120; // Amperes
                public static final double kSupplyCurrentLimit = 60; // Amperes
                public static final double kSupplyCurrentLowerLimit = 25; // Amperes
                public static final double kSupplyCurrentLowerTime = 0.5; // Seconds

                public static final double kS = 0.0;
                public static final double kV = 0.0;
                public static final double kA = 0.0;
                public static final double kP = 8.0; // 8 Amps per 60 RPM off
                public static final double kI = 0.0;
                public static final double kD = 0.2; // Get rid of oscillation

                public static final double kMETERING_MOTOR_RPM =
                    1000; // Target RPM for metering motor
        }
}
