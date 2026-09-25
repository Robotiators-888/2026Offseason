package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * Represents raw yaw and pitch angular target observations from a vision camera.
 */
public class TargetObservation implements StructSerializable {
        public final Rotation2d tx;
        public final Rotation2d ty;

        public TargetObservation(Rotation2d tx, Rotation2d ty) {
                this.tx = tx != null ? tx : Rotation2d.kZero;
                this.ty = ty != null ? ty : Rotation2d.kZero;
        }

        public static final Struct<TargetObservation> struct = new Struct<>() {
                @Override
                public Class<TargetObservation> getTypeClass() {
                        return TargetObservation.class;
                }

                @Override
                public String getTypeName() {
                        return "TargetObservation";
                }

                @Override
                public int getSize() {
                        return Rotation2d.struct.getSize() * 2;
                }

                @Override
                public String getSchema() {
                        return "Rotation2d tx;Rotation2d ty";
                }

                @Override
                public Struct<?>[] getNested() {
                        return new Struct<?>[] { Rotation2d.struct };
                }

                @Override
                public void pack(ByteBuffer bb, TargetObservation value) {
                        Rotation2d.struct.pack(bb, value.tx);
                        Rotation2d.struct.pack(bb, value.ty);
                }

                @Override
                public TargetObservation unpack(ByteBuffer bb) {
                        Rotation2d tx = Rotation2d.struct.unpack(bb);
                        Rotation2d ty = Rotation2d.struct.unpack(bb);
                        return new TargetObservation(tx, ty);
                }
        };
}
