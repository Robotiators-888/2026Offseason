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
import org.littletonrobotics.junction.Logger;

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
            new SysIdRoutine.Config(null, Volts.of(4), null,
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
         * Constructs a CTRE SwerveDrivetrain using specified constants, frequency, and standard
         * deviations.
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
                if (Utils.isSimulation() && mapleSimDrive != null) {
                        return mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative();
                }
                return this.getKinematics().toChassisSpeeds(getState().ModuleStates);
        }

        /**
         * Configures PathPlanner AutoBuilder settings and holonomic controller.
         */
        private void configurePathPlanner() {
                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        e.printStackTrace();
                        return;
                }

                AutoBuilder.configure(
                    this::getPose,
                    this::resetPose, this::getCurrentRobotChassisSpeeds,
                    (speeds, feedforwards) -> {
                            if (speeds != null && !Double.isNaN(speeds.vxMetersPerSecond)
                                && !Double.isNaN(speeds.vyMetersPerSecond)
                                && !Double.isNaN(speeds.omegaRadiansPerSecond)) {
                                    this.setControl(autoRequest.withSpeeds(speeds));
                            }
                    },
                    new PPHolonomicDriveController(
                        new PIDConstants(10, 0, 0), new PIDConstants(10, 0, 0)),
                    config, () -> {
                            var alliance = DriverStation.getAlliance();
                            if (alliance.isPresent()) {
                                    return alliance.get() == DriverStation.Alliance.Red;
                            }
                            return false;
                    }, this);
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
        private org.ironmaple.simulation.drivesims.SwerveDriveSimulation mapleSimDrive = null;
        private final org.dyn4j.geometry.Convex m_simBumperShape =
            org.dyn4j.geometry.Geometry.createRectangle(
                Inches.of(Constants.ROBOT_LENGTH_INCHES).in(Meters),
                Inches.of(Constants.ROBOT_WIDTH_INCHES).in(Meters));

        /**
         * Gets the active IronMaple SwerveDriveSimulation instance when running in simulation mode.
         *
         * @return SwerveDriveSimulation or null if on real hardware.
         */
        public org.ironmaple.simulation.drivesims.SwerveDriveSimulation getMapleSimDrive() {
                return mapleSimDrive;
        }

        @Override
        public void resetPose(Pose2d pose) {
                Pose2d finalPose = pose;
                if (Utils.isSimulation() && mapleSimDrive != null && m_simBumperShape != null) {
                        finalPose = org.ironmaple.simulation.MapleSimObstacleResolver.resolvePoseClipping(
                            pose, m_simBumperShape, org.ironmaple.simulation.SimulatedArena.getInstance());
                        mapleSimDrive.setSimulationWorldPose(finalPose);
                        if (finalPose.getX() != pose.getX() || finalPose.getY() != pose.getY()) {
                                Logger.recordOutput("Drive/UnclippedPose", finalPose);
                        }
                }
                super.resetPose(finalPose);
        }

        /**
         * Periodic subsystem loop called every 20ms. Updates alliance operator perspective
         * orientation and logs simulated battery sag telemetry in simulation mode.
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

                if (Utils.isSimulation()) {
                        double simBatteryVolts = org.ironmaple.simulation.motorsims.SimulatedBattery
                                                     .getBatteryVoltage()
                                                     .in(Volts);
                        double simBatteryCurrent = org.ironmaple.simulation.motorsims.SimulatedBattery
                                                       .getTotalCurrentDrawn()
                                                       .in(Amps);
                        Logger.recordOutput("Drive/SimulatedBatteryVoltage", simBatteryVolts);
                        Logger.recordOutput("Drive/SimulatedBatteryCurrentAmps", simBatteryCurrent);
                        if (mapleSimDrive != null) {
                                Logger.recordOutput("Drive/SimulatedDrivetrainPose",
                                    mapleSimDrive.getSimulatedDriveTrainPose());
                        }
                }
        }

        /**
         * Starts periodic physics simulation thread utilizing IronMaple (MapleSim) and SimulatedBattery.
         */
        private void startSimThread() {
                try {
                        org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
                            simulationConfig =
                                org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
                                    .Default()
                                    .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
                                    .withBumperSize(Inches.of(Constants.ROBOT_LENGTH_INCHES),
                                        Inches.of(Constants.ROBOT_WIDTH_INCHES))
                                    .withGyro(org.ironmaple.simulation.drivesims.COTS.ofPigeon2())
                                    .withTrackLengthTrackWidth(Inches.of(22.5), Inches.of(22.5))
                                    .withSwerveModule(
                                        new org.ironmaple.simulation.drivesims.configs
                                            .SwerveModuleSimulationConfig(
                                                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
                                                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
                                                6.026785714285714,
                                                26.09090909090909,
                                                Volts.of(0.2),
                                                Volts.of(0.2),
                                                Inches.of(2),
                                                KilogramSquareMeters.of(0.01),
                                                1.2));

                        // Use 2026 Rebuilt Arena without giant ramp obstacle block and install non-sticky contact listener
                        org.ironmaple.simulation.SimulatedArena.overrideInstance(
                            new org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt(false));
                        org.ironmaple.simulation.MapleSimObstacleResolver.installNonStickyContactListener(
                            org.ironmaple.simulation.SimulatedArena.getInstance());

                        Pose2d initialPose = (Math.abs(this.getState().Pose.getX()) < 0.05 && Math.abs(this.getState().Pose.getY()) < 0.05)
                            ? new Pose2d(3.0, 3.0, new Rotation2d())
                            : this.getState().Pose;
                        initialPose = org.ironmaple.simulation.MapleSimObstacleResolver.resolvePoseClipping(
                            initialPose, m_simBumperShape, org.ironmaple.simulation.SimulatedArena.getInstance());
                        this.mapleSimDrive =
                            new org.ironmaple.simulation.drivesims.SwerveDriveSimulation(
                                simulationConfig, initialPose);
                        super.resetPose(initialPose);

                        org.ironmaple.simulation.SimulatedArena.overrideSimulationTimings(
                            Seconds.of(kSimLoopPeriod), 1);
                        org.ironmaple.simulation.SimulatedArena.getInstance()
                            .addDriveTrainSimulation(mapleSimDrive);

                        // Keep CTRE internal odometry state synchronized with MapleSim physics (matches Team 449)
                        this.registerTelemetry(state -> {
                                if (mapleSimDrive != null) {
                                        state.Pose = mapleSimDrive.getSimulatedDriveTrainPose();
                                        state.Speeds = mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative();
                                }
                        });

                        for (int i = 0; i < 4; i++) {
                                final var realModule = this.getModule(i);
                                final var simModule = mapleSimDrive.getModules()[i];

                                simModule.useDriveMotorController(
                                    new org.ironmaple.simulation.motorsims
                                        .SimulatedMotorController() {
                                            @Override
                                            public edu.wpi.first.units.measure.Voltage
                                            updateControlSignal(
                                                edu.wpi.first.units.measure.Angle mechanismAngle,
                                                edu.wpi.first.units.measure.AngularVelocity
                                                    mechanismVelocity,
                                                edu.wpi.first.units.measure.Angle encoderAngle,
                                                edu.wpi.first.units.measure.AngularVelocity
                                                    encoderVelocity) {
                                                    realModule.getDriveMotor()
                                                        .getSimState()
                                                        .setRawRotorPosition(encoderAngle);
                                                    realModule.getDriveMotor()
                                                        .getSimState()
                                                        .setRotorVelocity(encoderVelocity);
                                                    realModule.getDriveMotor()
                                                        .getSimState()
                                                        .setSupplyVoltage(
                                                            org.ironmaple.simulation.motorsims
                                                                .SimulatedBattery
                                                                .getBatteryVoltage());
                                                    edu.wpi.first.units.measure.Voltage driveVolts =
                                                        realModule.getDriveMotor()
                                                            .getSimState()
                                                            .getMotorVoltageMeasure();
                                                    double v = driveVolts.in(Volts);
                                                    if (Double.isNaN(v) || Double.isInfinite(v)) {
                                                            v = 0.0;
                                                    }
                                                    return Volts.of(v);
                                            }
                                    });

                                simModule.useSteerMotorController(
                                    new org.ironmaple.simulation.motorsims
                                        .SimulatedMotorController() {
                                            @Override
                                            public edu.wpi.first.units.measure.Voltage
                                            updateControlSignal(
                                                edu.wpi.first.units.measure.Angle mechanismAngle,
                                                edu.wpi.first.units.measure.AngularVelocity
                                                    mechanismVelocity,
                                                edu.wpi.first.units.measure.Angle encoderAngle,
                                                edu.wpi.first.units.measure.AngularVelocity
                                                    encoderVelocity) {
                                                    realModule.getSteerMotor()
                                                        .getSimState()
                                                        .setRawRotorPosition(encoderAngle);
                                                    realModule.getSteerMotor()
                                                        .getSimState()
                                                        .setRotorVelocity(encoderVelocity);
                                                    realModule.getSteerMotor()
                                                        .getSimState()
                                                        .setSupplyVoltage(
                                                            org.ironmaple.simulation.motorsims
                                                                .SimulatedBattery
                                                                .getBatteryVoltage());
                                                    realModule.getEncoder()
                                                        .getSimState()
                                                        .setRawPosition(mechanismAngle.in(Rotations));
                                                    realModule.getEncoder()
                                                        .getSimState()
                                                        .setVelocity(mechanismVelocity.in(RotationsPerSecond));
                                                    realModule.getEncoder()
                                                        .getSimState()
                                                        .setSupplyVoltage(
                                                            org.ironmaple.simulation.motorsims
                                                                .SimulatedBattery
                                                                .getBatteryVoltage());
                                                    edu.wpi.first.units.measure.Voltage steerVolts =
                                                        realModule.getSteerMotor()
                                                            .getSimState()
                                                            .getMotorVoltageMeasure();
                                                    double v = steerVolts.in(Volts);
                                                    if (Double.isNaN(v) || Double.isInfinite(v)) {
                                                            v = 0.0;
                                                    }
                                                    return Volts.of(v);
                                            }
                                    });
                        }
                } catch (Exception e) {
                        e.printStackTrace();
                }

                m_lastSimTime = Utils.getCurrentTimeSeconds();

                /* Run simulation with MapleSim physics steps (matches Team 449 DriveIOSim architecture) */
                m_simNotifier = new Notifier(() -> {
                        if (mapleSimDrive != null) {
                                org.ironmaple.simulation.SimulatedArena.getInstance()
                                    .simulationPeriodic();

                                Pose2d simPose = mapleSimDrive.getSimulatedDriveTrainPose();
                                var speeds = mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative();

                                getPigeon2().getSimState().setRawYaw(simPose.getRotation().getMeasure());
                                getPigeon2().getSimState().setAngularVelocityZ(
                                    RadiansPerSecond.of(speeds.omegaRadiansPerSecond));
                                getPigeon2().getSimState().setSupplyVoltage(
                                    org.ironmaple.simulation.motorsims.SimulatedBattery.getBatteryVoltage());
                        } else {
                                final double currentTime = Utils.getCurrentTimeSeconds();
                                double deltaTime = currentTime - m_lastSimTime;
                                m_lastSimTime = currentTime;
                                updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        }
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
                if (visionRobotPoseMeters == null
                    || Double.isNaN(visionRobotPoseMeters.getX())
                    || Double.isNaN(visionRobotPoseMeters.getY())
                    || Double.isNaN(visionRobotPoseMeters.getRotation().getRadians())
                    || Double.isInfinite(visionRobotPoseMeters.getX())
                    || Double.isInfinite(visionRobotPoseMeters.getY())
                    || Double.isInfinite(visionRobotPoseMeters.getRotation().getRadians())
                    || Double.isNaN(timestampSeconds)
                    || Double.isInfinite(timestampSeconds)) {
                        return;
                }
                super.addVisionMeasurement(
                    visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
        }

        /**
         * Adds a vision measurement to the Kalman Filter with custom standard deviations.
         *
         * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
         * @param timestampSeconds The timestamp of the vision measurement in seconds.
         * @param visionMeasurementStdDevs Standard deviations of vision measurement [x, y, theta]ᵀ
         *     (meters, radians).
         */
        @Override
        public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
                if (visionRobotPoseMeters == null
                    || Double.isNaN(visionRobotPoseMeters.getX())
                    || Double.isNaN(visionRobotPoseMeters.getY())
                    || Double.isNaN(visionRobotPoseMeters.getRotation().getRadians())
                    || Double.isInfinite(visionRobotPoseMeters.getX())
                    || Double.isInfinite(visionRobotPoseMeters.getY())
                    || Double.isInfinite(visionRobotPoseMeters.getRotation().getRadians())
                    || Double.isNaN(timestampSeconds)
                    || Double.isInfinite(timestampSeconds)
                    || visionMeasurementStdDevs == null) {
                        return;
                }
                for (int i = 0; i < 3; i++) {
                        double stdDev = visionMeasurementStdDevs.get(i, 0);
                        if (Double.isNaN(stdDev) || Double.isInfinite(stdDev) || stdDev <= 0.0) {
                                return;
                        }
                }
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
         * Seeds the field-centric heading from the current rotation.
         */
        @Override
        public void seedFieldCentric() {
                super.seedFieldCentric();
                if (mapleSimDrive != null) {
                        mapleSimDrive.setSimulationWorldPose(new Pose2d(
                            mapleSimDrive.getSimulatedDriveTrainPose().getTranslation(), new Rotation2d()));
                }
        }

        /**
         * Gets the current 2D pose of the robot on the field from odometry or physics simulation.
         *
         * @return Current robot {@link Pose2d} in meters and rotation.
         */
        public Pose2d getPose() {
                if (Utils.isSimulation() && mapleSimDrive != null) {
                        Pose2d simPose = mapleSimDrive.getSimulatedDriveTrainPose();
                        if (simPose != null && !Double.isNaN(simPose.getX()) && !Double.isNaN(simPose.getY())
                            && !Double.isNaN(simPose.getRotation().getRadians())) {
                                return simPose;
                        }
                }
                var statePose = this.getState().Pose;
                if (statePose != null && !Double.isNaN(statePose.getX()) && !Double.isNaN(statePose.getY())
                    && !Double.isNaN(statePose.getRotation().getRadians())) {
                        return statePose;
                }
                return new Pose2d(3.0, 3.0, new Rotation2d());
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
