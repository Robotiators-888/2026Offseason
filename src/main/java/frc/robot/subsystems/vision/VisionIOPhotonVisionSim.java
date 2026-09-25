package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

/**
 * Simulation implementation of VisionIO utilizing PhotonVision's VisionSystemSim.
 * Simulates real camera frames, lens distortion, latency, and tag detection against virtual field tags.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
        private final PhotonCameraSim cameraSim;
        private final VisionSystemSim visionSim;
        private final Supplier<Pose2d> poseSupplier;

        public VisionIOPhotonVisionSim(
            String name,
            Transform3d robotToCamera,
            AprilTagFieldLayout fieldLayout,
            VisionSystemSim sharedVisionSim,
            Supplier<Pose2d> poseSupplier) {
                super(name, robotToCamera, fieldLayout);
                this.poseSupplier = poseSupplier;
                this.visionSim = sharedVisionSim != null ? sharedVisionSim : new VisionSystemSim("main");

                if (fieldLayout != null) {
                        this.visionSim.addAprilTags(fieldLayout);
                }

                SimCameraProperties cameraProperties = new SimCameraProperties();
                cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(99.41));
                cameraProperties.setCalibError(0.0, 0.0);
                cameraProperties.setFPS(60.0);
                cameraProperties.setAvgLatencyMs(20.0);
                cameraProperties.setLatencyStdDevMs(5.0);

                this.cameraSim = new PhotonCameraSim(camera, cameraProperties, fieldLayout);
                this.visionSim.addCamera(cameraSim, robotToCamera);
        }

        @Override
        public void updateInputs(VisionIOInputs inputs) {
                if (poseSupplier != null) {
                        visionSim.update(poseSupplier.get());
                }
                super.updateInputs(inputs);
        }

        public PhotonCameraSim getCameraSim() {
                return cameraSim;
        }

        public VisionSystemSim getVisionSim() {
                return visionSim;
        }
}
