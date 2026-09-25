package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

/**
 * Hardware implementation of DriveIO utilizing Phoenix 6 SwerveDrivetrain with CAN bus
 * optimization and odometry thread priority.
 */
public class DriveIOHardware extends TunerSwerveDrivetrain implements DriveIO {
        protected final AtomicReference<SwerveDriveState> telemetryCache =
            new AtomicReference<>(new SwerveDriveState());

        protected final Consumer<SwerveDriveState> telemetryConsumer = swerveDriveState -> {
                telemetryCache.set(swerveDriveState.clone());
        };

        private final StatusSignal<AngularVelocity> angularPitchVelocity;
        private final StatusSignal<AngularVelocity> angularRollVelocity;
        private final StatusSignal<AngularVelocity> angularYawVelocity;
        private final StatusSignal<?>[] gyroSignals;

        public DriveIOHardware(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, 100.0, modules);

                this.getOdometryThread().setThreadPriority(99);
                registerTelemetry(telemetryConsumer);

                this.angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
                this.angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
                this.angularYawVelocity = getPigeon2().getAngularVelocityZWorld();

                this.gyroSignals = new StatusSignal<?>[] {
                    angularPitchVelocity, angularRollVelocity, angularYawVelocity
                };

                BaseStatusSignal.setUpdateFrequencyForAll(100.0, gyroSignals);

                ParentDevice.optimizeBusUtilizationForAll(
                    getPigeon2(),
                    getModule(0).getDriveMotor(), getModule(0).getSteerMotor(),
                    getModule(1).getDriveMotor(), getModule(1).getSteerMotor(),
                    getModule(2).getDriveMotor(), getModule(2).getSteerMotor(),
                    getModule(3).getDriveMotor(), getModule(3).getSteerMotor()
                );
        }

        @Override
        public void updateInputs(DriveIOInputs inputs) {
                BaseStatusSignal.refreshAll(gyroSignals);

                SwerveDriveState cached = telemetryCache.get();
                if (cached != null && cached.ModuleStates != null) {
                        inputs.fromSwerveDriveState(cached);
                }

                inputs.rollVelocityDegPerSec = angularRollVelocity.getValue().in(DegreesPerSecond);
                inputs.pitchVelocityDegPerSec = angularPitchVelocity.getValue().in(DegreesPerSecond);
                inputs.yawVelocityDegPerSec = angularYawVelocity.getValue().in(DegreesPerSecond);

                for (int i = 0; i < 4; i++) {
                        var mod = getModule(i);
                        inputs.driveAppliedVolts[i] =
                            mod.getDriveMotor().getMotorVoltage().getValueAsDouble();
                        inputs.driveSupplyCurrentAmps[i] =
                            mod.getDriveMotor().getSupplyCurrent().getValueAsDouble();
                        inputs.driveStatorCurrentAmps[i] =
                            mod.getDriveMotor().getStatorCurrent().getValueAsDouble();
                        inputs.steerAppliedVolts[i] =
                            mod.getSteerMotor().getMotorVoltage().getValueAsDouble();
                        inputs.steerSupplyCurrentAmps[i] =
                            mod.getSteerMotor().getSupplyCurrent().getValueAsDouble();
                        inputs.steerStatorCurrentAmps[i] =
                            mod.getSteerMotor().getStatorCurrent().getValueAsDouble();
                }

                inputs.batteryVoltage = RobotController.getBatteryVoltage();
        }

        @Override
        public void resetOdometry(Pose2d pose) {
                resetPose(pose);
        }

        @Override
        public void setStateStdDevs(Matrix<N3, N1> stateStdDevs) {
                // Configures state standard deviations if needed
        }
}
