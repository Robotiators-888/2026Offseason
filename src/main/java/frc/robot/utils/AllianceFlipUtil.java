// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Field;
import java.util.Optional;

/**
 * Utility functions for flipping field coordinates between blue and red alliance perspectives.
 *
 * <p>Origin is default defined at rightmost point on blue alliance wall.
 */
public class AllianceFlipUtil {
        private AllianceFlipUtil() {}

        /** Field geometry flip modes (CenterPoint 180 deg rotation or Mirror along X axis). */
        public static enum FieldFlipType {
                /** Rotates 180 degrees around center point of field. */
                CenterPointFlip,
                /** Mirrors coordinates along field centerline X axis. */
                MirrorFlip,
        }

        /** Default flip type configuration for field transformations. */
        public static final FieldFlipType defaultFlipType = FieldFlipType.CenterPointFlip;

        /**
         * Flips a 2D translation vector to the current alliance side using default flip type.
         *
         * @param translation Blue alliance 2D translation vector in meters.
         * @return Alliance-adjusted 2D translation vector in meters.
         */
        public static Translation2d apply(Translation2d translation) {
                return apply(translation, defaultFlipType);
        }

        /**
         * Flips a 2D translation vector to the current alliance side using specified flip type.
         *
         * @param translation Blue alliance 2D translation vector in meters.
         * @param flipType Specified field flip mode.
         * @return Alliance-adjusted 2D translation vector in meters.
         */
        public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
                if (!shouldFlip())
                        return translation;
                switch (flipType) {
                        default:
                        case CenterPointFlip:
                                return new Translation2d(Field.fieldLength - translation.getX(),
                                    Field.fieldWidth - translation.getY());
                        case MirrorFlip:
                                return new Translation2d(
                                    Field.fieldLength - translation.getX(), translation.getY());
                }
        }

        /**
         * Flips a 2D rotation to current alliance side using default flip type.
         *
         * @param rotation Blue alliance 2D rotation.
         * @return Alliance-adjusted 2D rotation.
         */
        public static Rotation2d apply(Rotation2d rotation) {
                return apply(rotation, defaultFlipType);
        }

        /**
         * Flips a 2D rotation to current alliance side using specified flip type.
         *
         * @param rotation Blue alliance 2D rotation.
         * @param flipType Specified field flip mode.
         * @return Alliance-adjusted 2D rotation.
         */
        public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
                if (!shouldFlip())
                        return rotation;
                switch (flipType) {
                        default:
                        case CenterPointFlip:
                                return rotation.rotateBy(Rotation2d.fromRotations(0.5));
                        case MirrorFlip:
                                return new Rotation2d(-rotation.getCos(), rotation.getSin());
                }
        }

        /**
         * Flips a 2D pose to current alliance side using default flip type.
         *
         * @param pose Blue alliance 2D pose.
         * @return Alliance-adjusted 2D pose.
         */
        public static Pose2d apply(Pose2d pose) {
                return apply(pose, defaultFlipType);
        }

        /**
         * Flips a 2D pose to current alliance side using specified flip type.
         *
         * @param pose Blue alliance 2D pose.
         * @param flipType Specified field flip mode.
         * @return Alliance-adjusted 2D pose.
         */
        public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
                if (!shouldFlip())
                        return pose;
                return new Pose2d(
                    apply(pose.getTranslation(), flipType), apply(pose.getRotation(), flipType));
        }

        /**
         * Flips field-relative chassis speeds to current alliance perspective using default flip type.
         *
         * @param speeds Field-relative chassis speeds.
         * @return Alliance-adjusted chassis speeds.
         */
        public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds) {
                return applyFieldRelative(speeds, defaultFlipType);
        }

        /**
         * Flips field-relative chassis speeds to current alliance perspective using specified flip type.
         *
         * @param speeds Field-relative chassis speeds.
         * @param flipType Specified field flip mode.
         * @return Alliance-adjusted chassis speeds.
         */
        public static ChassisSpeeds applyFieldRelative(
            ChassisSpeeds speeds, FieldFlipType flipType) {
                if (!shouldFlip())
                        return speeds;
                switch (flipType) {
                        default:
                        case CenterPointFlip:
                                return new ChassisSpeeds(-speeds.vxMetersPerSecond,
                                    -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
                        case MirrorFlip:
                                return new ChassisSpeeds(-speeds.vxMetersPerSecond,
                                    speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
                }
        }

        /**
         * Flips robot-relative chassis speeds to current alliance perspective using default flip type.
         *
         * @param speeds Robot-relative chassis speeds.
         * @param robotRotation Current robot rotation.
         * @return Alliance-adjusted robot-relative chassis speeds.
         */
        public static ChassisSpeeds applyRobotRelative(
            ChassisSpeeds speeds, Rotation2d robotRotation) {
                return applyRobotRelative(speeds, robotRotation, defaultFlipType);
        }

        /**
         * Flips robot-relative chassis speeds to current alliance perspective using specified flip type.
         *
         * @param speeds Robot-relative chassis speeds.
         * @param robotRotation Current robot rotation.
         * @param flipType Specified field flip mode.
         * @return Alliance-adjusted robot-relative chassis speeds.
         */
        public static ChassisSpeeds applyRobotRelative(
            ChassisSpeeds speeds, Rotation2d robotRotation, FieldFlipType flipType) {
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    applyFieldRelative(
                        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotRotation)),
                    robotRotation);
        }

        /**
         * Returns whether pose and rotation coordinates should be flipped (true if current alliance is Red).
         *
         * @return True if on Red alliance, false otherwise.
         */
        public static boolean shouldFlip() {
                return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
        }
}
