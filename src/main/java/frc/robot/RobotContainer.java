// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Field;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Linear;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.CommandUtil;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.Elastic;
import frc.robot.utils.Hub;
import frc.robot.utils.RobotTelemetry;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

/**
 * RobotContainer declares all subsystems, operator interface (OI) bindings, commands,
 * and autonomous routines.
 *
 * <p>This class implements the declarative structure of the robot, mapping Xbox controller inputs
 * to commands, configuring default subsystem commands, and initializing PathPlanner autos.
 */
public class RobotContainer {
        /** Drivetrain subsystem initialized via Phoenix 6 TunerConstants. */
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        /** PhotonVision camera vision subsystem instance. */
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();

        /** Maximum linear velocity of the robot in meters/second. */
        private double MaxSpeed = 1.0
            * TunerConstants.kSpeedAt12Volts.in(
                MetersPerSecond); // kSpeedAt12Volts desired top speed

        /** Maximum angular rate of the robot in radians/second (0.75 rot/s). */
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(
            RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        /** Flywheel shooter subsystem instance. */
        public static final SUB_Shooter shooter = SUB_Shooter.getInstance();

        /** Intake roller subsystem instance. */
        public static final SUB_Roller roller = SUB_Roller.getInstance();

        /** Linear intake deploy subsystem instance. */
        public static final SUB_Linear linear = SUB_Linear.getInstance();

        /** Spindexer and feeder index subsystem instance. */
        public static final SUB_Index index = SUB_Index.getInstance();

        /** Adjustable hood subsystem instance. */
        public static final SUB_Hood hood = SUB_Hood.getInstance();

        /** Metering wheel subsystem instance. */
        public static final SUB_Metering metering = SUB_Metering.getInstance();

        /** REV Power Distribution Module for monitoring current and switchable channels. */
        public static final PowerDistribution powerDistribution = new PowerDistribution();


        /** Xbox controller on port 0 for primary driver controls. */
        private final CommandXboxController Driver1 =
            new CommandXboxController(Operator.kDriver1ControllerPort);

        /** Xbox controller on port 1 for secondary operator controls. */
        private final CommandXboxController Driver2 =
            new CommandXboxController(Operator.kDriver2ControllerPort);


        /** Command utility helper for registering PathPlanner named commands and macro routines. */
        public final CommandUtil commandUtil =
            new CommandUtil(drivetrain, roller, index, photonVision, shooter, hood, metering);
        
        
        public final ControllerUtil controllerUtil =
            new ControllerUtil(drivetrain, linear, roller, index, photonVision, shooter, hood, metering, Driver1, Driver2);

        /** Dashboard chooser for autonomous routines. */
        private final SendableChooser<Command> autoChooser;

        /** Slew rate limiters for smooth driver translation (X and Y) and rotation. */
        private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.0, -8.0, 0.0);
        private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.0, -8.0, 0.0);
        private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4.0, -8.0, 0.0);



        

        

        

        /** Swerve request object for robot-centric velocity driving. */
        private final SwerveRequest.RobotCentric driveRobot =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

        /** Swerve request object for field-centric velocity driving. */
        private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric().withDriveRequestType(
                DriveRequestType.Velocity);

        /** Cache for tracking auto name changes in dashboard. */
        private static String autoName, newAutoName;

        /** Previous alliance state tracker. */
        Optional<Alliance> lastAlliance;

        /** Current alliance state tracker. */
        Optional<Alliance> alliance;

        /** Field2d display object for visualizing active autonomous trajectory. */
        public static Field2d autoField = new Field2d();

        /** List index for logging/debugging. */
        public int listIndex = 0;

        

        /** Field2d display object for robot pose visualization. */
        Field2d field;

        /** Telemetry wrapper for robot dashboard reporting. */
        private final RobotTelemetry robotTelemetry;

        /**
         * The container for the robot. Initializes subsystems, sets default commands, configures
         * button bindings, and sets up the autonomous chooser.
         */
        public RobotContainer() {
                field = new Field2d();
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
                        double xInput = xLimiter.calculate(
                            MathUtil.applyDeadband(-Driver1.getLeftY(), Operator.kDriveDeadband));
                        double yInput = yLimiter.calculate(
                            MathUtil.applyDeadband(-Driver1.getLeftX(), Operator.kDriveDeadband));
                        double rotInput = rotLimiter.calculate(
                            MathUtil.applyDeadband(-Driver1.getRightX(), Operator.kDriveDeadband));
                        if (controllerUtil.fieldRelative) {
                                return drive.withVelocityX(xInput * MaxSpeed)
                                    .withVelocityY(yInput * MaxSpeed)
                                    .withRotationalRate(rotInput * MaxAngularRate);
                        } else {
                                return driveRobot.withVelocityX(xInput * MaxSpeed)
                                    .withVelocityY(yInput * MaxSpeed)
                                    .withRotationalRate(rotInput * MaxAngularRate);
                        }
                }));
                roller.setDefaultCommand(new RunCommand(() -> { roller.setRPM(0); }, roller));
                linear.setDefaultCommand(new RunCommand(() -> {
                        linear.forward();
                }, linear));
                shooter.setDefaultCommand(new RunCommand(() -> {                                
                        shooter.setRPM(SUB_Shooter.RPMIdle);
                }, shooter));
                index.setDefaultCommand(new RunCommand(() -> { index.set(0); }, index));
                metering.setDefaultCommand(
                    new RunCommand(() -> { metering.setRPM(0); }, metering));
                hood.setDefaultCommand(new RunCommand(() -> { hood.resetSafe(); }, hood));

                robotTelemetry = new RobotTelemetry(drivetrain, powerDistribution);
                commandUtil.registerAllNamedCommands();
                controllerUtil.configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autos/Auto Chooser", autoChooser);
                SmartDashboard.putData("Autos/Active Auto Path", autoField);
        }


        /**
         * Initializes robot hardware settings such as pathfinding algorithm and power distribution.
         */
        public void robotInit() {
                Pathfinding.setPathfinder(new LocalADStar());
                powerDistribution.setSwitchableChannel(true);
        }

        /**
         * Retrieves the user-selected autonomous command from the dashboard chooser.
         *
         * @return Selected autonomous {@link Command}.
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
                // return Commands.none();
        }

        /**
         * Periodic method called every 20ms during all robot operating modes. Updates dashboard
         * status, nearest trough position calculation, and telemetry outputs.
         */
        public void robotPeriodic() {
                SmartDashboard.putNumber("Stat/Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
                drivetrain.robotPosePublisher.set(drivetrain.getPose());
                field.setRobotPose(drivetrain.getPose());
                SmartDashboard.putData("Drivetrain/Field", field);
                SmartDashboard.putNumber(autoName, listIndex);
                SmartDashboard.putNumber("Shooter/Set RPM (In RobotContainer)", controllerUtil.targetRPM);
                SmartDashboard.putNumber("Drivetrain/Angular Velocity Error (dps)",
                    drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
                robotTelemetry.update();
        }

        /**
         * Initializes settings for autonomous mode, configuring logging callbacks and UI tabs.
         */
        public void autonomousInit() {
                drivetrain.setIntakeComplete(true);
                drivetrain.setReachedTarget(false);
                Elastic.selectTab("Autonomous");
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        Pose2d currentPose = drivetrain.getPose();

                        SmartDashboard.putNumber(
                            "Drivetrain/Stat/X Error", pose.getX() - currentPose.getX());
                        SmartDashboard.putNumber(
                            "Drivetrain/Stat/Y Error", pose.getY() - currentPose.getY());
                        SmartDashboard.putNumber("Drivetrain/Stat/Theta Error",
                            pose.getRotation().getRadians()
                                - currentPose.getRotation().getRadians());
                        SmartDashboard.putNumber(
                            "Drivetrain/Stat/Desired Theta", pose.getRotation().getRadians());
                        SmartDashboard.putNumber(
                            "Drivetrain/Stat/Actual Theta", currentPose.getRotation().getRadians());
                });
        }

        /** Periodic method called every 20ms during autonomous mode. Updates vision localization. */
        public void autonomousPeriodic() {
                photonPoseUpdate();
        }

        /** Initializes settings for test mode. */
        public void testInit() {}

        /** Periodic method called every 20ms during test mode. Updates vision localization. */
        public void testPeriodic() {
                photonPoseUpdate();
        }

        /** Initializes settings for teleoperated mode, configuring dashboard tabs and Hub data. */
        public void teleopInit() {
                Elastic.selectTab("Teleoperated");
                Elastic.Notification notification =
                    new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO,
                        "Alexander the Great would like to remind you:", "CHICKEN JOCKEY!!!!!");
                Elastic.sendNotification(notification);
                Hub.fetchMatchData();
        }

        /** Periodic method called every 20ms during teleoperated mode. Handles vision updates and Hub logic. */
        public void teleopPeriodic() {
                photonPoseUpdate();
                Hub.start(Driver1, Driver2, shooter);
        }

        /**
         * Periodic method called every 20ms while the robot is disabled. Updates active auto trajectory
         * visualization on field dashboard when auto selection or alliance changes.
         */
        public void disabledPeriodic() {
                newAutoName = getAutonomousCommand().getName();
                alliance = DriverStation.getAlliance();
                if (!newAutoName.equals(autoName) || !alliance.equals(lastAlliance)) {
                        autoName = newAutoName;
                        lastAlliance = alliance;
                        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                                try {
                                        List<PathPlannerPath> pathPlannerPaths =
                                            PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                                        List<Pose2d> poses = new ArrayList<>();
                                        for (PathPlannerPath path : pathPlannerPaths) {
                                                if (DriverStation.getAlliance().equals(
                                                        Optional.of(Alliance.Red))) {
                                                        poses.addAll(path.getAllPathPoints()
                                                                .stream()
                                                                .map(point
                                                                    -> new Pose2d(Field.fieldLength
                                                                            - point.position.getX(),
                                                                        Field.fieldWidth
                                                                            - point.position.getY(),
                                                                        new Rotation2d()))
                                                                .collect(Collectors.toList()));
                                                } else {
                                                        poses.addAll(path.getAllPathPoints()
                                                                .stream()
                                                                .map(point
                                                                    -> new Pose2d(
                                                                        point.position.getX(),
                                                                        point.position.getY(),
                                                                        new Rotation2d()))
                                                                .collect(Collectors.toList()));
                                                }
                                        }
                                        autoField.getObject("path").setPoses(poses);
                                } catch (IOException e) {
                                        Alert.registerError(
                                            "Failed to read path file: " + e.getMessage());
                                        return;
                                } catch (ParseException e) {
                                        Alert.registerError(
                                            "Failed to parse path file: " + e.getMessage());
                                        return;
                                }
                        }
                }
                photonPoseUpdate();
        }

        /**
         * Polls vision estimations from all three PhotonVision cameras and updates drivetrain odometry.
         */
        public void photonPoseUpdate() {
                processCameraPose(photonVision.getCam1Pose(), drivetrain.publisher3);
                processCameraPose(photonVision.getCam2Pose(), drivetrain.publisher4);
                processCameraPose(photonVision.getCam3Pose(), drivetrain.publisher5);
        }

        /**
         * Filters and processes estimated pose from a PhotonVision camera, feeding valid measurements
         * to the drivetrain pose estimator.
         *
         * @param poseOptional Estimated camera pose optional wrapper.
         * @param publisher NetworkTable publisher for camera pose visualization.
         */
        private void processCameraPose(
            Optional<EstimatedRobotPose> poseOptional, StructPublisher<Pose3d> publisher) {
                if (poseOptional.isPresent()) {
                        EstimatedRobotPose estimatedPose = poseOptional.get();
                        Pose3d photonPose = estimatedPose.estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                            && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                            && !estimatedPose.targetsUsed.isEmpty()) {
                                double minDist = Double.MAX_VALUE;
                                for (var target : estimatedPose.targetsUsed) {
                                        if (target.getPoseAmbiguity() > 0.2)
                                                continue;
                                        double dist = target.getBestCameraToTarget()
                                                          .getTranslation()
                                                          .getNorm();
                                        if (dist < minDist)
                                                minDist = dist;
                                }

                                if (minDist < 4.0) {
                                        double xyStddev = Math.pow(minDist, 2) / 16.0;
                                        double rotStddev = Units.degreesToRadians(120.0);
                                        SmartDashboard.putNumber(
                                            "Vision/PhotonVision Future TimeStamp?",
                                            Timer.getFPGATimestamp()
                                                - estimatedPose.timestampSeconds);
                                        drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                            estimatedPose.timestampSeconds,
                                            VecBuilder.fill(xyStddev, xyStddev, rotStddev));
                                        publisher.set(photonPose);
                                }
                        }
                }
        }
}
