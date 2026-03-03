// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO: Replace with new system built on Kraken branch
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;
import frc.robot.utils.Alert;

public class SUB_PhotonVision extends SubsystemBase {
  private static SUB_PhotonVision INSTANCE = null;

  private final PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCamName1);
  private final PhotonCamera cam2 = new PhotonCamera(PhotonVision.kCam2Name);
  private final PhotonCamera cam3 = new PhotonCamera(PhotonVision.kCam3Name);
  private PhotonTrackedTarget cam1BestTarget;
  private PhotonTrackedTarget cam2BestTarget;
  private PhotonTrackedTarget cam3BestTarget;
  private final PhotonPoseEstimator poseEstimator1;
  private final PhotonPoseEstimator poseEstimator2;
  private final PhotonPoseEstimator poseEstimator3;
  public AprilTagFieldLayout at_field;
  private static final double FUEL_DIAMETER_METERS = 0.15; 
  private static final int FUEL_PIPELINE_INDEX = 1; 

  public static SUB_PhotonVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_PhotonVision();
    }
    return INSTANCE;
  }

  private SUB_PhotonVision() {
    at_field =  AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark); // TODO: Change for diff events

    cam1.setPipelineIndex(0);
    cam2.setPipelineIndex(0);
    cam3.setPipelineIndex(0);

    poseEstimator1 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        PhotonVision.kRobotToCamera1);
    poseEstimator2 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera2); //TODO: For more camera (like 4 camera so we have one for climb) could we run vision on the RIO without losing too much processing?
    poseEstimator3 = new PhotonPoseEstimator(at_field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
         PhotonVision.kRobotToCamera3); //TODO: For more camera (like 4 camera so we have one for climb) could we run vision on the RIO without losing too much processing?
    poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<EstimatedRobotPose> getCam1Pose() {
    List<PhotonPipelineResult> results1 = cam1.getAllUnreadResults();
  
    Optional<EstimatedRobotPose> finalPose1 = Optional.empty();
    // Process results in reverse order to find the latest valid result
    java.util.ListIterator<PhotonPipelineResult> iterator = results1.listIterator(results1.size());
    while (iterator.hasPrevious()) {
      PhotonPipelineResult result = iterator.previous();
      if (result.hasTargets()) {
        cam1BestTarget = result.getBestTarget();
        finalPose1 = poseEstimator1.update(result);
        break; // Found the latest result, stop processing older ones
      }
    }
    return finalPose1;
  }

  public Optional<EstimatedRobotPose> getCam2Pose() {
    List<PhotonPipelineResult> results2 = cam2.getAllUnreadResults();
    Optional<EstimatedRobotPose> finalPose2 = Optional.empty();
    // Process results in reverse order to find the latest valid result
    java.util.ListIterator<PhotonPipelineResult> iterator = results2.listIterator(results2.size());
    while (iterator.hasPrevious()) {
      PhotonPipelineResult result = iterator.previous();
      if (result.hasTargets()) {
        cam2BestTarget = result.getBestTarget();
        finalPose2 = poseEstimator2.update(result);
        break; // Found the latest result, stop processing older ones
      }
    }
    return finalPose2;
  }
  public Optional<EstimatedRobotPose> getCam3Pose() {
    List<PhotonPipelineResult> results3 = cam3.getAllUnreadResults();
    Optional<EstimatedRobotPose> finalPose3 = Optional.empty();
    // Process results in reverse order to find the latest valid result
    java.util.ListIterator<PhotonPipelineResult> iterator = results3.listIterator(results3.size());
    while (iterator.hasPrevious()) {
      PhotonPipelineResult result = iterator.previous();
      if (result.hasTargets()) {
        cam3BestTarget = result.getBestTarget();
        finalPose3 = poseEstimator3.update(result);
        break; // Found the latest result, stop processing older ones
      }
    }
    return finalPose3;
  }

  public PhotonTrackedTarget getCam1BestTarget() {
    return cam1BestTarget;
  }

  public PhotonTrackedTarget getCam2BestTarget() {
    return cam2BestTarget;
  }
  public PhotonTrackedTarget getCam3BestTarget() {
    return cam3BestTarget;
  }

  public double getTargetYaw(PhotonTrackedTarget target) {
    return target.getYaw();
  }

  public double getTargetPitch(PhotonTrackedTarget target) {
    return target.getPitch();
  }

  public double getTargetArea(PhotonTrackedTarget target) {
    return target.getArea();
  }

  public int getId(PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

 public List<Pose3d> getFieldFuelPoses(Pose2d currentRobotPose) {
    List<Pose3d> allFuelPoses = new ArrayList<>();
    
    PhotonCamera[] cameras = {cam1, cam2, cam3};
    Transform3d[] cameraTransforms = {
        PhotonVision.kRobotToCamera1, 
        PhotonVision.kRobotToCamera2, 
        PhotonVision.kRobotToCamera3
    };

    for (int i = 0; i < cameras.length; i++) {
        if (!cameras[i].isConnected()) continue;

        // Ensure we only process if we are on the Fuel Pipeline
        // (If your pipeline switching logic is elsewhere, remove this check or ensure it matches)
        if (cameras[i].getPipelineIndex() != FUEL_PIPELINE_INDEX) continue;

        List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
        if (results.isEmpty()) continue;
        
        // Use the latest result
        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        
        if (latestResult.hasTargets()) {
            // Retrieve Camera Matrix (Intrinsics)
            Optional<Matrix<N3, N3>> cameraMatrixOpt = cameras[i].getCameraMatrix();
            if (cameraMatrixOpt.isEmpty()) continue; 
            
            Matrix<N3, N3> cameraMatrix = cameraMatrixOpt.get();
            // [ fx,  0, cx ]
            // [  0, fy, cy ]
            // [  0,  0,  1 ]
            double fx = cameraMatrix.get(0, 0);
            double fy = cameraMatrix.get(1, 1);
            
            // Average focal lengths for a generalized pixel focal length
            double focalLengthPixels = (fx + fy) / 2.0;

            Pose3d robotPose3d = new Pose3d(currentRobotPose);
            Pose3d cameraPose3d = robotPose3d.transformBy(cameraTransforms[i]);

            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                
                // Get the corners of the minimum area bounding rectangle
                List<TargetCorner> corners = target.getMinAreaRectCorners();
                if (corners == null || corners.size() < 4) continue;
                
                // Calculate all 6 pairwise distances between the 4 corners
                // (4 sides + 2 diagonals)
                double[] distances = new double[6];
                int distIdx = 0;
                for (int j = 0; j < corners.size(); j++) {
                    for (int k = j + 1; k < corners.size(); k++) {
                        double dx = corners.get(j).x - corners.get(k).x;
                        double dy = corners.get(j).y - corners.get(k).y;
                        distances[distIdx++] = Math.sqrt(dx * dx + dy * dy);
                    }
                }
                
                // Sort distances: smallest 4 are the rectangle sides, largest 2 are diagonals
                Arrays.sort(distances);
                
                // distances[0] and distances[1] are the two lengths of the shorter side
                // distances[2] and distances[3] are the two lengths of the longer side
                double widthPixels = (distances[0] + distances[1]) / 2.0;
                double heightPixels = (distances[2] + distances[3]) / 2.0;
                
                // The average of the width and height gives a robust pixel diameter for the sphere
                double pixelDiameter = (widthPixels + heightPixels) / 2.0;

                // Distance calculation using the pinhole camera model: Z = (f * Real_Size) / Pixel_Size
                double distanceMeters = (FUEL_DIAMETER_METERS * focalLengthPixels) / pixelDiameter;

                // Trigonometry to get translation from camera (PhotonVision gives Pitch/Yaw in degrees)
                // Note: Positive Pitch is UP, Positive Yaw is LEFT
                double pitchRad = Math.toRadians(target.getPitch());
                double yawRad = Math.toRadians(target.getYaw());

                // Convert Spherical (Distance, Pitch, Yaw) to Cartesian (x, y, z) in Camera Frame
                // Camera Frame: X is forward, Y is left, Z is up
                double xCam = distanceMeters * Math.cos(pitchRad) * Math.cos(yawRad);
                double yCam = distanceMeters * Math.cos(pitchRad) * Math.sin(yawRad);
                double zCam = distanceMeters * Math.sin(pitchRad);

                Translation3d cameraToBallTranslation = new Translation3d(xCam, yCam, zCam);

                // Transform the ball relative to the camera's position on the field
                // This automatically handles the camera's rotation (like if Cam 3 is sideways)
                Pose3d ballFieldPose = cameraPose3d.transformBy(new Transform3d(cameraToBallTranslation, new Rotation3d()));
                allFuelPoses.add(ballFieldPose);
            }
        }
    }
    return allFuelPoses;
  }

  @Override
  public void periodic() {
   if (!cam1.isConnected()) {
      Alert.registerError("PhotonVision Camera 1 Disconnected");
    }
    if (!cam2.isConnected()) {
      Alert.registerError("PhotonVision Camera 2 Disconnected");
    }
    if (!cam3.isConnected()) {
      Alert.registerError("PhotonVision Camera 2 Disconnected");
    }
  }
}