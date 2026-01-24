// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        public static final class Drivetrain {}

        public static final class Field {
                public static final double fieldLength = 1653.2 / 100.0;
                public static final double fieldWidth = 800.1 / 100.0;
        }

        public static final class PhotonVision {

                public static final String kCamName = "AprilTagCam1";
                public static final double kMaxZError = 0.2;
                public static final double kMaxAmbiguity = 0.1;
                public static final double kMaxDistance = 12.0;

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


                // public static final String kCam3Name = "AprilTagHighCam";
                // public static final Rotation3d cameraRotation3 = new Rotation3d(0,
                //                  Units.degreesToRadians(0), Units.degreesToRadians(8));
                // public static final Transform3d kRobotToCamera3 = new Transform3d(
                //                  Units.inchesToMeters(-7+3.25), Units.inchesToMeters(-10),
                //                  Units.inchesToMeters(23.5), cameraRotation);
        }

        
        
}
