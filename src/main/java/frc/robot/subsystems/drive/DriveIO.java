package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

/**
 * DriveIO subsystem hardware abstraction layer for AdvantageKit logging and replay.
 */
public interface DriveIO {
        @AutoLog
        public static class DriveIOInputs {
                public Pose2d pose = new Pose2d();
                public double gyroAngleDeg = 0.0;
                public double rollVelocityDegPerSec = 0.0;
                public double pitchVelocityDegPerSec = 0.0;
                public double yawVelocityDegPerSec = 0.0;

                public SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                        new SwerveModuleState(), new SwerveModuleState(),
                        new SwerveModuleState(), new SwerveModuleState()
                };
                public SwerveModuleState[] moduleTargets = new SwerveModuleState[] {
                        new SwerveModuleState(), new SwerveModuleState(),
                        new SwerveModuleState(), new SwerveModuleState()
                };
                public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                        new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition()
                };
                public ChassisSpeeds speeds = new ChassisSpeeds();
                public double odometryPeriod = 0.0;
                public int successfulDaqs = 0;
                public int failedDaqs = 0;

                public double[] driveAppliedVolts = new double[4];
                public double[] driveSupplyCurrentAmps = new double[4];
                public double[] driveStatorCurrentAmps = new double[4];
                public double[] steerAppliedVolts = new double[4];
                public double[] steerSupplyCurrentAmps = new double[4];
                public double[] steerStatorCurrentAmps = new double[4];
                public double batteryVoltage = 12.0;

                public void fromSwerveDriveState(SwerveDriveState stateIn) {
                        this.pose = stateIn.Pose;
                        this.gyroAngleDeg = stateIn.Pose.getRotation().getDegrees();
                        this.moduleStates = stateIn.ModuleStates;
                        this.moduleTargets = stateIn.ModuleTargets;
                        this.modulePositions = stateIn.ModulePositions;
                        this.speeds = stateIn.Speeds;
                        this.odometryPeriod = stateIn.OdometryPeriod;
                        this.successfulDaqs = stateIn.SuccessfulDaqs;
                        this.failedDaqs = stateIn.FailedDaqs;
                }
        }

        default void updateInputs(DriveIOInputs inputs) {}
        default void setControl(SwerveRequest request) {}
        default void resetOdometry(Pose2d pose) {}
        default void seedFieldCentric() {}
        default void setOperatorPerspectiveForward(Rotation2d yaw) {}
        default void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}
        default void setStateStdDevs(Matrix<N3, N1> stateStdDevs) {}
}
