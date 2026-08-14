// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Field;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {
  public static enum FieldFlipType {
    CenterPointFlip, MirrorFlip,
  }

  public static final FieldFlipType defaultFlipType = FieldFlipType.CenterPointFlip;

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    return apply(translation, defaultFlipType);
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
    if (!shouldFlip())
      return translation;
    switch (flipType) {
      default:
      case CenterPointFlip:
        return new Translation2d(Field.fieldLength - translation.getX(),
            Field.fieldWidth - translation.getY());
      case MirrorFlip:
        return new Translation2d(Field.fieldLength - translation.getX(), translation.getY());
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    return apply(rotation, defaultFlipType);
  }

  /** Flips a rotation based on the current alliance color. */
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

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    return apply(pose, defaultFlipType);
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
    if (!shouldFlip())
      return pose;
    return new Pose2d(apply(pose.getTranslation(), flipType), apply(pose.getRotation(), flipType));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }
}
