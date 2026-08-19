package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Command-based wrapper for CTRE Phoenix 6 Swerve Drivetrain.
 *
 * <p>Handles swerve drive kinematics, odometry update loops, PathPlanner auto integration,
 * vision measurement updates, operator alliance orientation, and NetworkTable logging.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
        /** Simulation update loop period in seconds (4 ms = 0.004 s). */
        private static final double kSimLoopPeriod = 0.004;

        /** Simulation periodic notifier. */
        private Notifier m_simNotifier = null;

        /** Timestamp of last simulation update step in seconds. */
        private double m_lastSimTime;

        /** Blue alliance perspective forward rotation (0 degrees towards red wall). */
        private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

        /** Red alliance perspective forward rotation (180 degrees towards blue wall). */
        private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

        /** Flag tracking whether driver perspective has been applied. */
        private boolean m_hasAppliedOperatorPerspective = false;

        /** Swerve request for applying chassis speeds in autonomous routines. */
        private final SwerveRequest.ApplyRobotSpeeds autoRequest =
            new SwerveRequest.ApplyRobotSpeeds();

        /** Swerve request object for SysId translation characterization. */
        private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();

        /** SysId characterization routine for drivetrain translation. */
        private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                Volts.of(4),
                null,
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

        /** Active SysId routine to execute. */
        private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

        /** Flag indicating autonomous target pose reached status. */
        private boolean reachedAutoTarget = false;

        /** Flag indicating intake routine completion status. */
        private boolean intakeComplete = true;

        /** Debug publisher for target X pose visualization on dashboard. */
        public final StructPublisher<Pose2d> publisher1 =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/debugXPoint", Pose2d.struct)
                .publish();

        /** Debug publisher for target Y pose visualization on dashboard. */
        public final StructPublisher<Pose2d> publisher2 =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/debugYPoint", Pose2d.struct)
                .publish();

        /** Publisher for Camera 1 estimated 3D pose. */
        public final StructPublisher<Pose3d> publisher3 =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/PhotonCam1Pose", Pose3d.struct)
                .publish();

        /** Publisher for Camera 2 estimated 3D pose. */
        public final StructPublisher<Pose3d> publisher4 =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/PhotonCam2Pose", Pose3d.struct)
                .publish();

        /** Publisher for High Camera estimated 3D pose. */
        public final StructPublisher<Pose3d> publisher5 =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/HighCamPose", Pose3d.struct)
                .publish();

        /** Publisher for selected target pose on dashboard. */
        public final StructPublisher<Pose2d> selectPosePublisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/SelectedPose", Pose2d.struct)
                .publish();

        /** Publisher for current robot 2D pose on dashboard. */
        public final StructPublisher<Pose2d> robotPosePublisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/Robot Pose", Pose2d.struct)
                .publish();

        /** Publisher for trench test path 1 pose. */
        public final StructPublisher<Pose2d> testPath1Publisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/TestPath1", Pose2d.struct)
                .publish();

        /** Publisher for trench test path 2 pose. */
        public final StructPublisher<Pose2d> testPath2Publisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/TestPath2", Pose2d.struct)
                .publish();

        /** Publisher for trench test path 3 pose. */
        public final StructPublisher<Pose2d> testPath3Publisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/TestPath3", Pose2d.struct)
                .publish();

        /** Publisher for trench test path 4 pose. */
        public final StructPublisher<Pose2d> testPath4Publisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/TestPath4", Pose2d.struct)
                .publish();

        /** Publisher for selected trench test path pose. */
        public final StructPublisher<Pose2d> selectedTestPathPublisher =
            NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/Drivetrain/SelectedTestPath", Pose2d.struct)
                .publish();

        /** Publisher for actual swerve module states array. */
        public final StructArrayPublisher<SwerveModuleState> swerveModuleStatesPublisher =
            NetworkTableInstance.getDefault()
                .getStructArrayTopic(
                    "SmartDashboard/Drivetrain/SwerveModuleStates", SwerveModuleState.struct)
                .publish();

        /** Publisher for desired swerve module states array. */
        public final StructArrayPublisher<SwerveModuleState> desiredSwerveModuleStatesPublisher =
            NetworkTableInstance.getDefault()
                .getStructArrayTopic("SmartDashboard/Drivetrain/Desired SwerveModuleStates",
                    SwerveModuleState.struct)
                .publish();

        /**
         * Constructs a CTRE SwerveDrivetrain using specified constants.
         *
         * @param drivetrainConstants Drivetrain-wide constants for swerve drive.
         * @param modules Constants for each specific module.
         */
        public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, modules);
                if (Utils.isSimulation()) {
                        startSimThread();
                }
                configurePathPlanner();
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using specified constants and update frequency.
         *
         * @param drivetrainConstants Drivetrain-wide constants for swerve drive.
         * @param odometryUpdateFrequency Odometry update loop frequency in Hertz (Hz).
         * @param modules Constants for each specific module.
         */
        public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, odometryUpdateFrequency, modules);
                if (Utils.isSimulation()) {
                        startSimThread();
                }
                configurePathPlanner();
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using specified constants, frequency, and standard deviations.
         *
         * @param drivetrainConstants Drivetrain-wide constants for swerve drive.
         * @param odometryUpdateFrequency Odometry update loop frequency in Hertz (Hz).
         * @param odometryStandardDeviation Odometry measurement noise standard deviations.
         * @param visionStandardDeviation Vision measurement noise standard deviations.
         * @param modules Constants for each specific module.
         */
        public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation,
                    visionStandardDeviation, modules);
                if (Utils.isSimulation()) {
                        startSimThread();
                }
                configurePathPlanner();
        }

        /**
         * Calculates current robot-relative chassis speeds based on active swerve module states.
         *
         * @return Current robot {@link ChassisSpeeds} in meters/second and radians/second.
         */
        public ChassisSpeeds getCurrentRobotChassisSpeeds() {
                return this.getKinematics().toChassisSpeeds(getState().ModuleStates);
        }

        /**
         * Configures PathPlanner AutoBuilder settings and holonomic controller.
         */
        private void configurePathPlanner() {
                // RobotConfig config;
                // try {
                //         config = RobotConfig.fromGUISettings();
                // } catch (Exception e) {
                //         e.printStackTrace();
                //         return;
                // }

                // AutoBuilder.configure(()
                //                           -> this.getState().Pose,
                //     this::resetPose,
                //     this::getCurrentRobotChassisSpeeds,
                //     (speeds, feedforwards)
                //         -> this.setControl(autoRequest.withSpeeds(
                //             speeds)),
                //     new PPHolonomicDriveController(
                //         new PIDConstants(10, 0, 0), new PIDConstants(10, 0, 0)),
                //     config, () -> {
                //             var alliance = DriverStation.getAlliance();
                //             if (alliance.isPresent()) {
                //                     return alliance.get() == DriverStation.Alliance.Red;
                //             }
                //             return false;
                //     }, this);
        }

        /**
         * Returns a command that applies the specified control request to this swerve drivetrain.
         *
         * @param request Function returning the request to apply
         * @return Command to run
         */
        public Command applyRequest(Supplier<SwerveRequest> request) {
                return run(() -> this.setControl(request.get()));
        }

        /**
         * Runs the SysId Quasistatic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Quasistatic test
         * @return Command to run
         */
        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
                return m_sysIdRoutineToApply.quasistatic(direction);
        }

        /**
         * Runs the SysId Dynamic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Dynamic test
         * @return Command to run
         */
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
                return m_sysIdRoutineToApply.dynamic(direction);
        }

        /**
         * Periodic subsystem loop called every 20ms. Updates alliance operator perspective orientation.
         */
        @Override
        public void periodic() {
                if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
                        DriverStation.getAlliance().ifPresent(allianceColor -> {
                                setOperatorPerspectiveForward(allianceColor == Alliance.Red
                                        ? kRedAlliancePerspectiveRotation
                                        : kBlueAlliancePerspectiveRotation);
                                m_hasAppliedOperatorPerspective = true;
                        });
                }
        }

        /**
         * Starts periodic physics simulation thread.
         */
        private void startSimThread() {
                m_lastSimTime = Utils.getCurrentTimeSeconds();

                /* Run simulation at a faster rate so PID gains behave more reasonably */
                m_simNotifier = new Notifier(() -> {
                        final double currentTime = Utils.getCurrentTimeSeconds();
                        double deltaTime = currentTime - m_lastSimTime;
                        m_lastSimTime = currentTime;

                        /* use the measured time delta, get battery voltage from WPILib */
                        updateSimState(deltaTime, RobotController.getBatteryVoltage());
                });
                m_simNotifier.startPeriodic(kSimLoopPeriod);
        }

        /**
         * Adds a vision measurement to the Kalman Filter.
         *
         * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
         * @param timestampSeconds The timestamp of the vision measurement in seconds.
         */
        @Override
        public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
                super.addVisionMeasurement(
                    visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
        }

        /**
         * Adds a vision measurement to the Kalman Filter with custom standard deviations.
         *
         * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
         * @param timestampSeconds The timestamp of the vision measurement in seconds.
         * @param visionMeasurementStdDevs Standard deviations of vision measurement [x, y, theta]ᵀ (meters, radians).
         */
        @Override
        public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
                super.addVisionMeasurement(visionRobotPoseMeters,
                    Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
        }

        /**
         * Return the pose at a given timestamp, if the buffer is not empty.
         *
         * @param timestampSeconds The timestamp of the pose in seconds.
         * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
         */
        @Override
        public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
                return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
        }

        /**
         * Gets the current 2D pose of the robot on the field from odometry.
         *
         * @return Current robot {@link Pose2d} in meters and rotation.
         */
        public Pose2d getPose() {
                return this.getState().Pose;
        }

        /**
         * Sets whether the robot has reached its autonomous target position.
         *
         * @param value Target reached flag value.
         */
        public void setReachedTarget(boolean value) {
                reachedAutoTarget = value;
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
                    "Drivetrain/ReachedAutoTarget", reachedAutoTarget);
        }

        /**
         * Gets whether the robot has reached its autonomous target position.
         *
         * @return True if target reached, false otherwise.
         */
        public boolean getReachedTarget() {
                return reachedAutoTarget;
        }

        /**
         * Sets whether game piece intake operation is complete.
         *
         * @param value Intake complete flag value.
         */
        public void setIntakeComplete(boolean value) {
                intakeComplete = value;
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
                    "Drivetrain/IntakeComplete", intakeComplete);
        }

        /**
         * Gets whether game piece intake operation is complete.
         *
         * @return True if intake complete, false otherwise.
         */
        public boolean getIntakeComplete() {
                return intakeComplete;
        }
}
