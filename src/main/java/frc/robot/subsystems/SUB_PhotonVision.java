// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;
import frc.robot.utils.Alert;

public class SUB_PhotonVision extends SubsystemBase {
  // Set up singleton
  private static SUB_PhotonVision INSTANCE = null;

  // Create cameras and targets
  private final List<PhotonCamera> cams = new ArrayList<PhotonCamera>();
  private final List<PhotonTrackedTarget> bestTargets = new ArrayList<PhotonTrackedTarget>();
  private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<PhotonPoseEstimator>();
  public AprilTagFieldLayout at_field;

  // Function to get singleton
  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  private SUB_PhotonVision() {
    cams.add(new PhotonCamera(PhotonVision.kCamName1));
    cams.add(new PhotonCamera(PhotonVision.kCam2Name));
    cams.add(new PhotonCamera(PhotonVision.kCam3Name));
    bestTargets.add(null);
    bestTargets.add(null);
    bestTargets.add(null);
    poseEstimators.add(new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera1));
    poseEstimators.add(new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera2)); //TODO: For more camera (like 4 camera so we have one for climb) could we run vision on the RIO without losing too much processing?);
    poseEstimators.add(new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera3)); //TODO: For more camera (like 4 camera so we have one for climb) could we run vision on the RIO without losing too much processing?);
    // Load the correct field
    at_field =  AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark); // TODO: Change for diff events

    // Set up cameras and pose estimators
    cams.get(0).setPipelineIndex(0);
    cams.get(1).setPipelineIndex(0);
    cams.get(2).setPipelineIndex(0);
    // Make sure Multi Tag is enabled
    poseEstimators.get(0).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimators.get(1).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimators.get(2).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  // Functions to get camera poses
  public Optional<EstimatedRobotPose> getCamPose(int index) {
    List<PhotonPipelineResult> results = cams.get(index).getAllUnreadResults();
  
    Optional<EstimatedRobotPose> finalPose = Optional.empty();
    // Process results in reverse order to find the latest  result
    java.util.ListIterator<PhotonPipelineResult> iterator = results.listIterator(results.size());
    while (iterator.hasPrevious()) {
      PhotonPipelineResult result = iterator.previous();
      if (result.hasTargets()) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        bestTargets.set(index, bestTarget);
        finalPose = poseEstimators.get(index).update(result);
        break; // Found the latest result, stop processing older ones
      }
    }
    return finalPose;
  }

  // Get best camera targets
  public PhotonTrackedTarget getCamBestTarget(int index) {
    return bestTargets.get(index);
  }

  // Get target Yaw Pich or Area
  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  // Get the id of a photon target
  public int getId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  // Alert if cameras are disconnected
  @Override
  public void periodic() {
    for (int i = 0; i < cams.size(); i++) {
      if (!cams.get(i).isConnected()) {
        Alert.registerError("PhotonVision Camera " + (i+1) + " Disconnected");
      }
    }
  }
}