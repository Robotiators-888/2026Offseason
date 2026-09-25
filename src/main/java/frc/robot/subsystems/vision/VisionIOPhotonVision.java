package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Real hardware implementation of VisionIO for physical PhotonVision cameras.
 */
public class VisionIOPhotonVision implements VisionIO {
        protected final PhotonCamera camera;
        protected final Transform3d robotToCamera;
        protected final AprilTagFieldLayout fieldLayout;

        public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
                this.camera = new PhotonCamera(name);
                this.robotToCamera = robotToCamera;
                AprilTagFieldLayout layout;
                try {
                        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                } catch (Exception e) {
                        layout = null;
                }
                this.fieldLayout = layout;
        }

        public VisionIOPhotonVision(
            String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
                this.camera = new PhotonCamera(name);
                this.robotToCamera = robotToCamera;
                this.fieldLayout = layout;
        }

        @Override
        public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = camera.isConnected();

                Set<Integer> tagIds = new HashSet<>();
                List<PoseObservation> observations = new ArrayList<>();

                for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                        if (result.hasTargets()) {
                                PhotonTrackedTarget best = result.getBestTarget();
                                inputs.latestTargetObservation = new TargetObservation(
                                    Rotation2d.fromDegrees(best.getYaw()),
                                    Rotation2d.fromDegrees(best.getPitch())
                                );
                        } else {
                                inputs.latestTargetObservation = new TargetObservation(
                                    Rotation2d.kZero, Rotation2d.kZero
                                );
                        }

                        if (result.getMultiTagResult().isPresent()) {
                                var multitag = result.getMultiTagResult().get();
                                Transform3d fieldToCamera = multitag.estimatedPose.best;
                                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                                Pose3d robotPose = new Pose3d(
                                    fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                                double totalDist = 0.0;
                                for (PhotonTrackedTarget target : result.getTargets()) {
                                        totalDist += target.getBestCameraToTarget()
                                                         .getTranslation()
                                                         .getNorm();
                                        tagIds.add(target.getFiducialId());
                                }
                                double avgDist = result.getTargets().isEmpty()
                                    ? 0.0
                                    : totalDist / result.getTargets().size();

                                observations.add(new PoseObservation(
                                    result.getTimestampSeconds(),
                                    robotPose,
                                    multitag.estimatedPose.ambiguity,
                                    multitag.fiducialIDsUsed.size(),
                                    avgDist,
                                    0 // MultiTag
                                ));
                        } else if (!result.getTargets().isEmpty()) {
                                PhotonTrackedTarget target = result.getTargets().get(0);
                                if (fieldLayout != null) {
                                        var tagPoseOpt = fieldLayout.getTagPose(target.getFiducialId());
                                        if (tagPoseOpt.isPresent()) {
                                                Pose3d tagPose = tagPoseOpt.get();
                                                Transform3d fieldToTarget = new Transform3d(
                                                    tagPose.getTranslation(), tagPose.getRotation());
                                                Transform3d cameraToTarget =
                                                    target.getBestCameraToTarget();
                                                Transform3d fieldToCamera =
                                                    fieldToTarget.plus(cameraToTarget.inverse());
                                                Transform3d fieldToRobot =
                                                    fieldToCamera.plus(robotToCamera.inverse());
                                                Pose3d robotPose = new Pose3d(
                                                    fieldToRobot.getTranslation(),
                                                    fieldToRobot.getRotation());

                                                tagIds.add(target.getFiducialId());
                                                observations.add(new PoseObservation(
                                                    result.getTimestampSeconds(),
                                                    robotPose,
                                                    target.getPoseAmbiguity(),
                                                    1,
                                                    cameraToTarget.getTranslation().getNorm(),
                                                    1 // Single Tag
                                                ));
                                        }
                                }
                        }
                }

                inputs.poseObservations = observations.toArray(new PoseObservation[0]);
                inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
        }
}
