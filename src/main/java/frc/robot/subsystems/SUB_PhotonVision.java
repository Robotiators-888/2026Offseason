// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVision;
import frc.robot.utils.Alert;

public class SUB_PhotonVision extends SubsystemBase {
  private static SUB_PhotonVision INSTANCE = null;

  /** Hardware cameras and pose estimators */
  private final PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCamName1);
  private final PhotonCamera cam2 = new PhotonCamera(PhotonVision.kCam2Name);
  private final PhotonCamera cam3 = new PhotonCamera(PhotonVision.kCam3Name);
  private final PhotonPoseEstimator poseEstimator1;
  private final PhotonPoseEstimator poseEstimator2;
  private final PhotonPoseEstimator poseEstimator3;

  /** @return Single instance of the SUB_PhotonVision subsystem */
  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  private SUB_PhotonVision() {
    // Shared with Constants.Field, so the tag poses and the field dimensions can never disagree.
    final AprilTagFieldLayout atField = Constants.Field.kTagLayout;

    // Initialize cameras and pose estimators with MULTI_TAG_PNP strategy
    cam1.setPipelineIndex(0);
    cam2.setPipelineIndex(0);
    cam3.setPipelineIndex(0);

    poseEstimator1 = new PhotonPoseEstimator(atField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera1);
    poseEstimator2 = new PhotonPoseEstimator(atField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera2);
    poseEstimator3 = new PhotonPoseEstimator(atField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera3);

    // Set fallback strategy for single-tag scenarios
    poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Runs every unread frame from a camera through its pose estimator and returns all resulting
   * estimates, oldest first.
   *
   * <p>The previous per-camera copies of this kept only the newest frame with targets and threw
   * the rest away — at 30-60 fps against a 50 Hz loop that silently discarded valid
   * measurements, and during a loop overrun it could drop many. The drivetrain's pose estimator
   * has a timestamped buffer precisely so every measurement can be fused.
   */
  private static List<EstimatedRobotPose> drainCamera(final PhotonCamera camera,
      final PhotonPoseEstimator estimator) {
    final List<EstimatedRobotPose> poses = new ArrayList<>();
    for (final PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        estimator.update(result).ifPresent(poses::add);
      }
    }
    return poses;
  }

  /** @return All new pose estimates from camera 1 since the last call, oldest first */
  public List<EstimatedRobotPose> getCam1Poses() {
    return drainCamera(cam1, poseEstimator1);
  }

  /** @return All new pose estimates from camera 2 since the last call, oldest first */
  public List<EstimatedRobotPose> getCam2Poses() {
    return drainCamera(cam2, poseEstimator2);
  }

  /** @return All new pose estimates from camera 3 since the last call, oldest first */
  public List<EstimatedRobotPose> getCam3Poses() {
    return drainCamera(cam3, poseEstimator3);
  }

  @Override
  public void periodic() {
    // Check connection status and report errors. registerError only bumps a counter after the
    // first occurrence — the dashboard render happens once per loop in Alert.periodic().
   if (!cam1.isConnected()) {
      Alert.registerError("PhotonVision Camera 1 Disconnected");
    }
    if (!cam2.isConnected()) {
      Alert.registerError("PhotonVision Camera 2 Disconnected");
    }
    if (!cam3.isConnected()) {
      Alert.registerError("PhotonVision Camera 3 Disconnected");
    }
  }
}
