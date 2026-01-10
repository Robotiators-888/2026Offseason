// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

        public static class Operator {
                public static final int kDriver1ControllerPort = 0;
                public static final int kDriver2ControllerPort = 1;
                public static final double kDriveDeadband = 0.05;
        }

        public static class Swerve {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 12; // on shreyan soul it is 14
                                                                       // tooth

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps =
                                Motor.kVortexFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = Units.inchesToMeters(2 * 1.6243455433105947);
                // Thrifty tread 2.95in
                // Orange Tread 2.70
                // Black Rev 2.95
                public static final double kWheelCircumferenceMeters =
                                kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the
                // bevel pinion
                public static final double kDrivingMotorReduction =
                                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps =
                                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor =
                                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor =
                                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters
                                                                                                    // per
                                                                                                    // second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians

                public static final double kTurningVelocityFactor = (2 * Math.PI) / 60.0; // radians
                                                                                          // per
                                                                                          // second
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians
                                                                                                 // per
                                                                                                 // second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput =
                                kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;
                public static final double headingTolerance = Degrees.of(1).in(Radians);
                // Max Rot = Max Linear ((meters/sec)/60 (m/s)) / radius
                public static final double kMaxRotationalSpeed =
                                (kDrivingMotorFreeSpeedRps / 60) / Drivetrain.kTrackRadius;

                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 60; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps

        }

        public static final class Drivetrain {
                public static final int kFRONT_LEFT_DRIVE_MOTOR_CANID = 20;
                public static final int kFRONT_LEFT_STEER_MOTOR_CANID = 21;
                public static final int kFRONT_RIGHT_DRIVE_MOTOR_CANID = 22;
                public static final int kFRONT_RIGHT_STEER_MOTOR_CANID = 23;
                public static final int kBACK_RIGHT_DRIVE_MOTOR_CANID = 24;
                public static final int kBACK_RIGHT_STEER_MOTOR_CANID = 25;
                public static final int kBACK_LEFT_DRIVE_MOTOR_CANID = 26;
                public static final int kBACK_LEFT_STEER_MOTOR_CANID = 27;

                public static final int kPigeon_CANID = 10;

                public static final Rotation2d shooterSide = new Rotation2d(0);
                public static final Rotation2d intakeSide = new Rotation2d(180);

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double kMaxSpeedMetersPerSecond = 5.74;
                public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

                public static final double kDirectionSlewRate = 100000; // radians per second
                public static final double kMagnitudeSlewRate = 100000; // percent per second (1 =
                                                                        // 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 =
                                                                      // 100%)

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(23.5);
                // 30.5inches by 27inches
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(27);

                public static final double kTrackRadius = Units.inchesToMeters(17.8972763);// ((23.5/2)^2+(27/2)^2)^0.5
                public static final double kMaxModuleSpeed = Units.feetToMeters(15);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics =
                                new SwerveDriveKinematics(
                                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                                new Translation2d(-kWheelBase / 2,
                                                                -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2.0;
                public static final double kFrontRightChassisAngularOffset = 0.0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2.0;

                public static final boolean kGyroReversed = true;

                public static final double kGyroRotation = 0;

                // Constants for CMD_ReefAlign
                public static final double kXShiftMagnitude =
                                Units.inchesToMeters(5+(30.5 / 2)); // Distance away from
                                                                 
                                // the April Tag
                public static final double kYShiftMagnitude = Units.inchesToMeters(6.5); // Distance
                                                                                         // shifted
                                                                                         // to the
                                                                                         // left/right
                                                                                         // of the
                                                                                         // April
                                                                                         // Tag

        }

        // Motor Constants
        public static final class Motor {
                public static final double kVortexFreeSpeedRpm = 6784;
                public static final double kNeoFreeSpeedRpm = 5676;
        }

        public static final class Field {
                public static final double fieldLength = 1755.0 / 100.0;
                public static final double fieldWidth = 805.0 / 100.0;
        }

        public static final class PhotonVision {

                public static final String kCam1Name = "AprilTagCam1";
                public static final Rotation3d cameraRotation = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(0),
                                Units.degreesToRadians(-25));
                public static final Transform3d kRobotToCamera1 = new Transform3d(
                                Units.inchesToMeters(15.25 - 7.625), Units.inchesToMeters(13.5 - 2.75),
                                Units.inchesToMeters(11), cameraRotation);

                public static final String kCam2Name = "AprilTagCam2"; // TODO: Change to the correct name(AprilTagCam) and Transform3d and Rotation3d (Make sure to use this Transform3d and Rotation3d for the other camera)
                public static final Rotation3d cameraRotation2 = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(0),
                                Units.degreesToRadians(25)); // CCW positive yaw with it circling around the z axis, zero is straight forward
                public static final Transform3d kRobotToCamera2 = new Transform3d(
                                Units.inchesToMeters(15.25-7.625), Units.inchesToMeters(-13.5+2.75), // X is forward and the camera is in front of the center of the robot, Y positive is left and the camera is on the right of the robot, Z is up from the ground and it is above the ground
                                Units.inchesToMeters(11), cameraRotation2);


                public static final String kCam3Name = "AprilTagHighCam";
                public static final Rotation3d cameraRotation3 = new Rotation3d(0,
                                 Units.degreesToRadians(0), Units.degreesToRadians(8));
                public static final Transform3d kRobotToCamera3 = new Transform3d(
                                 Units.inchesToMeters(-7+3.25), Units.inchesToMeters(-10),
                                 Units.inchesToMeters(23.5), cameraRotation);
        }

        public static final class PivotConstants {
                public static final ArmFeedforward noCoralArmFeedforward =
                                new ArmFeedforward(0, 0.69, 0.34);
                public static final ArmFeedforward coralArmFeedforward =
                                new ArmFeedforward(0, 0.69, 0.34); // TODO: Set it the same for now,
                                                                   // change later

                public static final double kAlgaeSafeSetpoint = 160;
                public static final double kAlgaeScoringSetpoint = 132;

                public static final double kIntakeSetpoint = 326;
                public static final double kElevatingSetpoint = 288;
                public static final double kCoralSetpoint = 300;
                public static final double kAlgaeSetpoint = 176;
                public static final double toleranceDegrees = 5;

                public static final double kL1Setpoint = 300;
                public static final double kL2Setpoint = 294;
                public static final double kL3Setpoint = 295;
                public static final double kL4Setpoint = 273;

                public static final double kUpperBoundStuckPoint = 340.0;
                public static final double kLowerBoundStuckPoint = 327.0;
        }

        public static final class Elevator {
                public static final double kHomingEncoderLocation = 0;
                public static final double kResetHomingThreshold = 0.05;
                public static final double kEncoderNearZero = 0.01;
                public static final double kHomingEmergencyCurrent = 40;
                public static final double kHomingVoltage = -0.25;
                public static final double kTolerance = 0.05;


                public static final double kMaxUpVoltage = 6.375;
                public static final double kMaxUpErrorThreshold = 0.25;
                public static final double kHighUpVoltage = 5.1;
                public static final double kHighUpErrorThreshold = 0.15;
                public static final double kMediumUpVoltage = 4.2;
                public static final double kMediumUpErrorThreshold = 0.06;
                public static final double kSlowUpVoltage = 3.4;

                public static final double kMaxDownVoltage = -2.65;
                public static final double kMaxDownErrorThreshold = 0.25;
                public static final double kHighDownVoltage = -2.1;
                public static final double kHighDownErrorThreshold = 0.20;
                public static final double kMediumDownVoltage = -1.2;
                public static final double kMediumDownErrorThreshold = 0.09;
                public static final double kSlowDownVoltage = -0.55;
                public static final double kSlowDownThreshold = 0.06;

                public static final double kEmptyHoldingVoltage = 0.6;
                public static final double kEmptyHoldingVoltageTop = 0.74;
                public static final double kCoralHoldingVoltage = 0.85;
                public static final double kAlgaeHoldingVoltage = 0.72;

                public static final double kStartingSetpoint = 0;
                public static final double kL1Setpoint = 0.1;
                public static final double kL2Setpoint = 0.162;
                public static final double kL3Setpoint = 0.375;
                public static final double kL4Setpoint = 0.705;
                public static final double kAlgaeSetpoint = 0.508;
                public static final double kProcessorSetpoint = 0.104;
        }

        public static class Roller {
                public static final int kRollerCanID = 30;
                public static final int kHelperCanID = 34;
                public static final double kIntakeCurrentThreshold = 35; // Amps
                public static final int kRollerCurrentLimit = 60;
                public static final int kHelperCurrentLimit = 20;
                public static final double kRollerHelperSpeed = 0.7;
                public static final double kIntakeSpeed = 0.2; // Percent
                public static final double kIntakeFinishSpeed = 0.1; // Percent
                public static final double kIntakeStartingTime = 1.25; // Seconds
                public static final double kIntakeFinishTime = 0.12; // Seconds

                public static final double kEjectSpeed = 0.7; // Percent
                public static final double kFreeSpinThreshold = 420; // RPM
        }
        public static class Climber {
                public static final int kClimberCanID = 40;
                public static final double kClimberPercentOutput = 0.2;
        }

        public static class LEDs {
                public static final int kPWMPort = 9;
                public static final double kColorGreen = 0.77;
                public static final double kColorRed = 0.61;
                public static final double kParty_Palette_Twinkles = -0.53;
        }
}
