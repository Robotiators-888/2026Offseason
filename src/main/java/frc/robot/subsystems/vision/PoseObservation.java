package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * Represents a timestamped 3D pose estimate from vision target tracking.
 */
public class PoseObservation implements StructSerializable {
        public final double timestamp;
        public final Pose3d pose;
        public final double ambiguity;
        public final int tagCount;
        public final double averageTagDistance;
        public final int type;

        public PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
            double averageTagDistance, int type) {
                this.timestamp = timestamp;
                this.pose = pose != null ? pose : new Pose3d();
                this.ambiguity = ambiguity;
                this.tagCount = tagCount;
                this.averageTagDistance = averageTagDistance;
                this.type = type;
        }

        public static final Struct<PoseObservation> struct = new Struct<>() {
                @Override
                public Class<PoseObservation> getTypeClass() {
                        return PoseObservation.class;
                }

                @Override
                public String getTypeName() {
                        return "PoseObservation";
                }

                @Override
                public int getSize() {
                        return 32 + Pose3d.struct.getSize();
                }

                @Override
                public String getSchema() {
                        return "double timestamp;Pose3d pose;double ambiguity;int32 tagCount;double averageTagDistance;int32 type";
                }

                @Override
                public Struct<?>[] getNested() {
                        return new Struct<?>[] { Pose3d.struct };
                }

                @Override
                public void pack(ByteBuffer bb, PoseObservation value) {
                        bb.putDouble(value.timestamp);
                        Pose3d.struct.pack(bb, value.pose);
                        bb.putDouble(value.ambiguity);
                        bb.putInt(value.tagCount);
                        bb.putDouble(value.averageTagDistance);
                        bb.putInt(value.type);
                }

                @Override
                public PoseObservation unpack(ByteBuffer bb) {
                        double timestamp = bb.getDouble();
                        Pose3d pose = Pose3d.struct.unpack(bb);
                        double ambiguity = bb.getDouble();
                        int tagCount = bb.getInt();
                        double avgDist = bb.getDouble();
                        int type = bb.getInt();
                        return new PoseObservation(
                            timestamp, pose, ambiguity, tagCount, avgDist, type);
                }
        };
}
