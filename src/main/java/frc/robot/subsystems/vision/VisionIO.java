package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for vision cameras with AdvantageKit logging and replay.
 */
public interface VisionIO {
        @AutoLog
        public static class VisionIOInputs {
                public boolean connected = false;
                public TargetObservation latestTargetObservation =
                    new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
                public PoseObservation[] poseObservations = new PoseObservation[0];
                public int[] tagIds = new int[0];
        }

        default void updateInputs(VisionIOInputs inputs) {}
}
