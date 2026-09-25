// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVision;
import frc.robot.utils.Alert;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Subsystem managing PhotonVision AprilTag camera interfaces and multi-camera pose estimation.
 *
 * <p>Hardware:
 * <ul>
 *   <li>Back-Left Camera ("BackLeftCam") with 3D transform {@link
 * PhotonVision#kRobotToCamera1}</li> <li>Back-Right Camera ("BackRightCam") with 3D transform
 * {@link PhotonVision#kRobotToCamera2}</li> <li>High Camera ("HighCam") with 3D transform {@link
 * PhotonVision#kRobotToCamera3}</li>
 * </ul>
 */
public class SUB_PhotonVision extends SubsystemBase {
        private static SUB_PhotonVision INSTANCE = null;

        /** Hardware cameras and targeting state */
        private final PhotonCamera cam1 = new PhotonCamera(PhotonVision.kCamName1);
        private final PhotonCamera cam2 = new PhotonCamera(PhotonVision.kCam2Name);
        private final PhotonCamera cam3 = new PhotonCamera(PhotonVision.kCam3Name);
        private PhotonTrackedTarget cam1BestTarget;
        private PhotonTrackedTarget cam2BestTarget;
        private PhotonTrackedTarget cam3BestTarget;
        private final PhotonPoseEstimator poseEstimator1;
        private final PhotonPoseEstimator poseEstimator2;
        private final PhotonPoseEstimator poseEstimator3;

        /** AprilTag field layout configuration for target coordinate localization. */
        public AprilTagFieldLayout at_field;

        /**
         * Singleton pattern provider for the PhotonVision subsystem.
         *
         * @return Single instance of the {@link SUB_PhotonVision} subsystem.
         */
        public static SUB_PhotonVision getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_PhotonVision();
                }
                return INSTANCE;
        }

        /**
         * Private constructor initializing camera pipelines, 2026 AprilTag field layout,
         * and PhotonPoseEstimators with MULTI_TAG_PNP_ON_COPROCESSOR strategy.
         */
        private SUB_PhotonVision() {
                // Load the 2026 field layout
                at_field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

                // Initialize cameras and pose estimators with MULTI_TAG_PNP strategy
                cam1.setPipelineIndex(0);
                cam2.setPipelineIndex(0);
                cam3.setPipelineIndex(0);

                poseEstimator1 = new PhotonPoseEstimator(at_field,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonVision.kRobotToCamera1);
                poseEstimator2 = new PhotonPoseEstimator(at_field,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonVision.kRobotToCamera2);
                poseEstimator3 = new PhotonPoseEstimator(at_field,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonVision.kRobotToCamera3);

                // Set fallback strategy for single-tag scenarios
                poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        /**
         * Processes unread frames from camera 1 and returns the latest estimated robot pose.
         *
         * @return Optional containing {@link EstimatedRobotPose} if valid targets were visible.
         */
        public Optional<EstimatedRobotPose> getCam1Pose() {
                List<PhotonPipelineResult> results1 = cam1.getAllUnreadResults();

                Optional<EstimatedRobotPose> finalPose1 = Optional.empty();
                // Process results in reverse order to find the latest valid result
                java.util.ListIterator<PhotonPipelineResult> iterator =
                    results1.listIterator(results1.size());
                while (iterator.hasPrevious()) {
                        PhotonPipelineResult result = iterator.previous();
                        if (result.hasTargets()) {
                                cam1BestTarget = result.getBestTarget();
                                finalPose1 = poseEstimator1.update(result);
                                break;
                        }
                }
                return finalPose1;
        }

        /**
         * Processes unread frames from camera 2 and returns the latest estimated robot pose.
         *
         * @return Optional containing {@link EstimatedRobotPose} if valid targets were visible.
         */
        public Optional<EstimatedRobotPose> getCam2Pose() {
                List<PhotonPipelineResult> results2 = cam2.getAllUnreadResults();
                Optional<EstimatedRobotPose> finalPose2 = Optional.empty();
                java.util.ListIterator<PhotonPipelineResult> iterator =
                    results2.listIterator(results2.size());
                while (iterator.hasPrevious()) {
                        PhotonPipelineResult result = iterator.previous();
                        if (result.hasTargets()) {
                                cam2BestTarget = result.getBestTarget();
                                finalPose2 = poseEstimator2.update(result);
                                break;
                        }
                }
                return finalPose2;
        }

        /**
         * Processes unread frames from camera 3 and returns the latest estimated robot pose.
         *
         * @return Optional containing {@link EstimatedRobotPose} if valid targets were visible.
         */
        public Optional<EstimatedRobotPose> getCam3Pose() {
                List<PhotonPipelineResult> results3 = cam3.getAllUnreadResults();
                Optional<EstimatedRobotPose> finalPose3 = Optional.empty();
                java.util.ListIterator<PhotonPipelineResult> iterator =
                    results3.listIterator(results3.size());
                while (iterator.hasPrevious()) {
                        PhotonPipelineResult result = iterator.previous();
                        if (result.hasTargets()) {
                                cam3BestTarget = result.getBestTarget();
                                finalPose3 = poseEstimator3.update(result);
                                break;
                        }
                }
                return finalPose3;
        }

        /**
         * Gets the best target tracked by camera 1.
         *
         * @return Best {@link PhotonTrackedTarget} for camera 1.
         */
        public PhotonTrackedTarget getCam1BestTarget() {
                return cam1BestTarget;
        }

        /**
         * Gets the best target tracked by camera 2.
         *
         * @return Best {@link PhotonTrackedTarget} for camera 2.
         */
        public PhotonTrackedTarget getCam2BestTarget() {
                return cam2BestTarget;
        }

        /**
         * Gets the best target tracked by camera 3.
         *
         * @return Best {@link PhotonTrackedTarget} for camera 3.
         */
        public PhotonTrackedTarget getCam3BestTarget() {
                return cam3BestTarget;
        }

        /**
         * Gets yaw angle offset to specified tracked target in degrees.
         *
         * @param target Tracked target.
         * @return Horizontal offset angle in degrees.
         */
        public double getTargetYaw(PhotonTrackedTarget target) {
                return target.getYaw();
        }

        /**
         * Gets pitch angle offset to specified tracked target in degrees.
         *
         * @param target Tracked target.
         * @return Vertical offset angle in degrees.
         */
        public double getTargetPitch(PhotonTrackedTarget target) {
                return target.getPitch();
        }

        /**
         * Gets area of target bounding box as percentage of image area (0.0 to 100.0).
         *
         * @param target Tracked target.
         * @return Target area percentage.
         */
        public double getTargetArea(PhotonTrackedTarget target) {
                return target.getArea();
        }

        /**
         * Gets the AprilTag fiducial ID of the specified tracked target.
         *
         * @param target Tracked target.
         * @return The fiducial ID integer of the tracked tag.
         */
        public int getId(PhotonTrackedTarget target) {
                return target.getFiducialId();
        }

        private org.photonvision.simulation.VisionSystemSim visionSim = null;
        private org.photonvision.simulation.PhotonCameraSim camSim1 = null;
        private org.photonvision.simulation.PhotonCameraSim camSim2 = null;
        private org.photonvision.simulation.PhotonCameraSim camSim3 = null;
        private java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> simPoseSupplier = null;

        /**
         * Enables simulated cameras using PhotonVision VisionSystemSim tied to virtual field tags.
         *
         * @param poseSupplier Supplier providing the simulated drivetrain pose.
         */
        public void enableSimulation(
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
                this.simPoseSupplier = poseSupplier;
                this.visionSim = new org.photonvision.simulation.VisionSystemSim("main");
                if (at_field != null) {
                        this.visionSim.addAprilTags(at_field);
                }

                org.photonvision.simulation.SimCameraProperties cameraProperties =
                    new org.photonvision.simulation.SimCameraProperties();
                cameraProperties.setCalibration(
                    1280, 800, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(99.41));
                cameraProperties.setFPS(60.0);
                cameraProperties.setAvgLatencyMs(20.0);
                cameraProperties.setLatencyStdDevMs(5.0);

                camSim1 = new org.photonvision.simulation.PhotonCameraSim(
                    cam1, cameraProperties, at_field);
                camSim2 = new org.photonvision.simulation.PhotonCameraSim(
                    cam2, cameraProperties, at_field);
                camSim3 = new org.photonvision.simulation.PhotonCameraSim(
                    cam3, cameraProperties, at_field);

                visionSim.addCamera(camSim1, PhotonVision.kRobotToCamera1);
                visionSim.addCamera(camSim2, PhotonVision.kRobotToCamera2);
                visionSim.addCamera(camSim3, PhotonVision.kRobotToCamera3);
        }

        /**
         * Simulation periodic loop (20ms). Updates simulated camera scene with virtual robot pose.
         */
        @Override
        public void simulationPeriodic() {
                if (visionSim != null && simPoseSupplier != null) {
                        visionSim.update(simPoseSupplier.get());
                }
        }

        /**
         * Periodic subsystem loop (20ms). Checks connection status of each hardware camera
         * and reports errors to dashboard using Alert utility if disconnected.
         */
        @Override
        public void periodic() {
                // Check connection status and report errors
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
