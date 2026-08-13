// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.Operator;
import frc.robot.commands.CMD_AimBot;
import frc.robot.commands.CMD_Shuttle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Linear;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.CommandUtil;
import frc.robot.utils.Elastic;
import frc.robot.utils.Hub;
import frc.robot.utils.RobotTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final SUB_Shooter shooter = SUB_Shooter.getInstance();
        public static final SUB_Roller roller = SUB_Roller.getInstance();
        public static final SUB_Linear linear = SUB_Linear.getInstance();
        public static final SUB_Index index = SUB_Index.getInstance();
        public static final SUB_Hood hood = SUB_Hood.getInstance();
        public static final SUB_Metering metering = SUB_Metering.getInstance();
        public static final PowerDistribution powerDistribution = new PowerDistribution();
        public final CommandUtil commandUtil = new CommandUtil(drivetrain, linear, roller, index, photonVision, shooter, hood, metering);
        private final SendableChooser<Command> autoChooser;
        private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.0,-8.0,0.0);
        private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.0,-8.0,0.0);
        private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4.0,-8.0,0.0);
        // TrenchCrossing Paths
        private PathPlannerPath pathLeftToNeutral;
        private PathPlannerPath pathNeutralToLeft;
        private PathPlannerPath pathRightToNeutral;
        private PathPlannerPath pathNeutralToRight;
        private boolean fieldRelative = true;
        private Command trenchAlign = Commands.none();
        private boolean trenchAligning = false;

        // xBox Controllers for driver input
        private final CommandXboxController Driver1 = new CommandXboxController(Operator.kDriver1ControllerPort);
        private final CommandXboxController Driver2 = new CommandXboxController(Operator.kDriver2ControllerPort);
        // Driving Swerve Requests
        private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity); 
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); //Control is based on speed
        
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public int listIndex = 0;
        public double targetRPM = 1000;

        /** Ceiling for the manual RPM trim on Driver 2's Y/A buttons. */
        private static final double kMaxManualRPM = 4000;
        Field2d field;
        private final RobotTelemetry robotTelemetry;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                field = new Field2d();
                try {
                        pathLeftToNeutral = PathPlannerPath.fromPathFile("LeftTrough-LeftTroughCenter");
                        pathNeutralToLeft = PathPlannerPath.fromPathFile("LeftTroughCenter-LeftTroughTA");
                        pathRightToNeutral = PathPlannerPath.fromPathFile("RightTrough-RightTroughCenter");
                        pathNeutralToRight = PathPlannerPath.fromPathFile("RightTroughCenter-RightTroughTA");
                } catch (Exception e) {
                        Alert.registerError("Failed to load trench paths: " + e.getMessage());
                }

                drivetrain.setDefaultCommand(            
                        drivetrain.applyRequest(() -> {
                                double xInput = xLimiter.calculate(MathUtil.applyDeadband(-Driver1.getLeftY(), Operator.kDriveDeadband));
                                double yInput = yLimiter.calculate(MathUtil.applyDeadband(-Driver1.getLeftX(), Operator.kDriveDeadband));
                                double rotInput = rotLimiter.calculate(MathUtil.applyDeadband(-Driver1.getRightX(), Operator.kDriveDeadband));
                                if (fieldRelative) {
                                        return drive.withVelocityX(xInput*MaxSpeed)
                                                .withVelocityY(yInput*MaxSpeed)
                                                .withRotationalRate(rotInput*MaxAngularRate);
                                } else {
                                        return driveRobot.withVelocityX(xInput*MaxSpeed)
                                                .withVelocityY(yInput*MaxSpeed)
                                                .withRotationalRate(rotInput*MaxAngularRate);
                                }
                        })
                );
                roller.setDefaultCommand(new RunCommand(() -> {
                        roller.set(0);
                }, roller));
                linear.setDefaultCommand(new InstantCommand(() -> { // Could make it go forward and make it an instant command
                        linear.set(0);
                }, linear));
                shooter.setDefaultCommand(new RunCommand(() -> {
                        // Idle at the speed the current range would need, so a shot only has to wait
                        // out the last few hundred RPM. With no hub in the layout there is nothing
                        // sensible to pre-spin to, so coast instead.
                        Hub.getDistanceToGoal(drivetrain.getPose()).ifPresentOrElse(
                                distance -> shooter.setRPM(SUB_Shooter.findoptimalRPM(
                                        distance,
                                        SUB_Hood.findoptimalangle(distance))),
                                shooter::stop);
                }, shooter));
                index.setDefaultCommand(new InstantCommand(() -> {
                        index.set(0);
                }, index));
                metering.setDefaultCommand(new InstantCommand(() -> {
                        metering.set(0);
                }, metering));
                hood.setDefaultCommand(new RunCommand(() -> {
                        hood.stow();
                }, hood));

                robotTelemetry = new RobotTelemetry(drivetrain, powerDistribution);
                commandUtil.registerAllNamedCommands();
                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Autos/Auto Chooser", autoChooser);
                SmartDashboard.putData("Autos/Active Auto Path", autoField);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers
         * or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                // =========================================================
                // DRIVER 1
                // =========================================================
                Driver1.leftBumper().onTrue(Commands.runOnce(() -> {
                        trenchAligning = true;
                        PathPlannerPath selectedPath = nearestTrenchPath();
                        if (selectedPath == null) {
                                Alert.registerError("No trench paths loaded, cannot align");
                                return;
                        }

                        try {
                                PathConstraints constraints = new PathConstraints(4.0, 4.0,
                                                Units.degreesToRadians(360), Units.degreesToRadians(540));
                                trenchAlign = AutoBuilder.pathfindThenFollowPath(selectedPath, constraints).until(()->{
                                        return !trenchAligning;
                                });
                                CommandScheduler.getInstance().schedule(trenchAlign);
                        } catch (Exception e) {
                                Alert.registerError("Failed to retrieve trench command: " + e.getMessage());
                        }
                })).onFalse(new InstantCommand(()->{trenchAligning=false;}));
                Driver1.rightBumper().whileTrue(Commands.run(() -> {
                        roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE);
                        linear.forward();
                }, roller, linear));
                Driver1.rightTrigger().whileTrue(
                        new ParallelCommandGroup(
                                new CMD_AimBot(
                                        drivetrain, 
                                        photonVision, 
                                        index,
                                        hood,
                                        metering,
                                        shooter,
                                        () -> -(Driver1.getLeftY()),
                                        () -> -(Driver1.getLeftX()) 
                                ),
                                commandUtil.getLinearCompress()
                        )
                );
                Driver1.leftStick().onTrue(new InstantCommand(() -> {
                        fieldRelative = !fieldRelative;
                }
                ));
                // =========================================================
                // DRIVER 2
                // =========================================================
                Driver2.leftTrigger().whileTrue(new RunCommand(() -> shooter.setRPM(targetRPM), shooter));
                Driver2.rightTrigger().whileTrue(new RunCommand(() -> {
                        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                        
                }, index));
                // Clamped so holding the button cannot walk the setpoint into a reverse spin-up.
                Driver2.y().onTrue(new InstantCommand(() -> targetRPM = MathUtil.clamp(targetRPM + 25, 0, kMaxManualRPM)));
                Driver2.a().onTrue(new InstantCommand(() -> targetRPM = MathUtil.clamp(targetRPM - 25, 0, kMaxManualRPM)));
                Driver2.leftBumper().whileTrue(new RunCommand(() -> {
                        index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS);
                        shooter.setVolts(-2.5);
                }, index, shooter));
                Driver2.povDown().whileTrue(Commands.run(()->linear.forward(),linear));
                Driver2.povUp().whileTrue(Commands.run(()->linear.backward(),linear));
                // Driver2.rightBumper().whileTrue(new RunCommand(() -> {
                //         linear.set(MathUtil.applyDeadband(Driver2.getLeftY(), Operator.kDriveDeadband) * Constants.Linear.kLINEAR_MOTOR_SPEED);
                // }, linear));
                // Manual hood jog + re-zero. This used to share Driver2.rightTrigger() with the
                // indexer feed above, so every feed also nudged the hood down and silently
                // redefined its zero on release.
                Driver2.povLeft().whileTrue(new RunCommand(() -> hood.set(-.05), hood))
                        .onFalse(new InstantCommand(() -> hood.resetEncoder(), hood));

                // Proper current-sensing home, bounded by a timeout.
                Driver2.povRight().onTrue(hood.homeCommand());
                Driver2.b().whileTrue(
                        new CMD_Shuttle(drivetrain, photonVision, index, shooter,
                                () -> -(Driver1.getLeftY()),
                                () -> -(Driver1.getLeftX())
                        )
                );
                Driver2.x().onTrue(new InstantCommand(() -> Hub.getDistanceToGoal(drivetrain.getPose())
                        .ifPresent(distance -> targetRPM = shooter.getDistanceRPM(distance))));
        }

        public void robotInit() {
                Pathfinding.setPathfinder(new LocalADStar());
                powerDistribution.setSwitchableChannel(true);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /** The four trench-crossing paths, alliance-flipped. Null entries mean the file failed to load. */
        private PathPlannerPath[] trenchPaths() {
                return new PathPlannerPath[] {
                        pathLeftToNeutral, pathNeutralToLeft, pathRightToNeutral, pathNeutralToRight
                };
        }

        private static final String[] kTrenchNames = {
                "Left Trough -> Neutral Zone",
                "Neutral Zone -> Left Trough",
                "Right Trough -> Neutral Zone",
                "Neutral Zone -> Right Trough"
        };

        /**
         * Index of the trench path whose start pose is closest to the robot, or -1 if none loaded.
         *
         * <p>Paths that failed to load are skipped rather than being substituted with the origin,
         * which used to make the field origin the "nearest" pose and silently mis-select.
         */
        private int nearestTrenchIndex() {
                Translation2d here = drivetrain.getPose().getTranslation();
                PathPlannerPath[] paths = trenchPaths();
                int best = -1;
                double bestDistance = Double.MAX_VALUE;

                for (int i = 0; i < paths.length; i++) {
                        if (paths[i] == null) {
                                continue;
                        }
                        Optional<Pose2d> start = paths[i].getStartingHolonomicPose();
                        if (start.isEmpty()) {
                                continue;
                        }
                        double distance = here.getDistance(
                                        AllianceFlipUtil.apply(start.get()).getTranslation());
                        if (distance < bestDistance) {
                                bestDistance = distance;
                                best = i;
                        }
                }
                return best;
        }

        /** @return The trench path nearest the robot, or null if none are loaded. */
        private PathPlannerPath nearestTrenchPath() {
                int index = nearestTrenchIndex();
                return index < 0 ? null : trenchPaths()[index];
        }

        /** Dashboard preview of which trench path the left bumper would pick. */
        private void publishTrenchPreview() {
                PathPlannerPath[] paths = trenchPaths();
                StructPublisher<Pose2d>[] publishers = drivetrain.testPathPublishers();

                for (int i = 0; i < paths.length; i++) {
                        Pose2d start = paths[i] == null
                                        ? new Pose2d()
                                        : AllianceFlipUtil.apply(paths[i].getStartingHolonomicPose().orElse(new Pose2d()));
                        publishers[i].set(start);
                }

                int nearest = nearestTrenchIndex();
                if (nearest < 0) {
                        SmartDashboard.putString("Trough/Closest", "none loaded");
                        return;
                }
                drivetrain.selectedTestPathPublisher.set(
                                AllianceFlipUtil.apply(paths[nearest].getStartingHolonomicPose().orElse(new Pose2d())));
                SmartDashboard.putString("Trough/Closest", kTrenchNames[nearest]);
        }

        public void robotPeriodic() {
                SmartDashboard.putNumber("Stat/Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
                drivetrain.robotPosePublisher.set(drivetrain.getPose());
                field.setRobotPose(drivetrain.getPose());
                SmartDashboard.putData("Drivetrain/Field", field);
                SmartDashboard.putNumber("Shooter/Set RPM (In RobotContainer)",targetRPM);
                SmartDashboard.putNumber("Drivetrain/Angular Velocity Error (dps)", drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

                publishTrenchPreview();
                robotTelemetry.update();
        }


        public void autonomousInit() {
                drivetrain.setIntakeComplete(true);
                drivetrain.setReachedTarget(false);
                Elastic.selectTab("Autonomous");
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {

                        Pose2d currentPose = drivetrain.getPose();

                        SmartDashboard.putNumber("Drivetrain/Stat/X Error", pose.getX() - currentPose.getX());
                        SmartDashboard.putNumber("Drivetrain/Stat/Y Error", pose.getY() - currentPose.getY());
                        SmartDashboard.putNumber("Drivetrain/Stat/Theta Error", pose.getRotation().getRadians()
                                        - currentPose.getRotation().getRadians());
                        SmartDashboard.putNumber("Drivetrain/Stat/Desired Theta", pose.getRotation().getRadians());
                        SmartDashboard.putNumber("Drivetrain/Stat/Actual Theta",
                                        currentPose.getRotation().getRadians());

                });
        }

        public void autonomousPeriodic() {
                photonPoseUpdate();
        }


        public void testInit() {
        }


        public void testPeriodic() {
                photonPoseUpdate();
        }

        public void teleopInit() {
                // Otherwise robot-centric mode set in a previous session persists into this match.
                fieldRelative = true;
                Elastic.selectTab("Teleoperated");
                Elastic.Notification notification = new Elastic.Notification(
                                Elastic.Notification.NotificationLevel.INFO, "Alexander the Great would like to remind you:", "CHICKEN JOCKEY!!!!!");
                Elastic.sendNotification(notification);
                Hub.fetchMatchData();
        }

        public void teleopPeriodic() {
                photonPoseUpdate();
                Hub.start(Driver1,Driver2,shooter);
        }

        public void disabledPeriodic() {
                final Command selectedAuto = getAutonomousCommand();
                if (selectedAuto == null) {
                        photonPoseUpdate();
                        return;
                }
                newAutoName = selectedAuto.getName();
                alliance = DriverStation.getAlliance();
                if (!newAutoName.equals(autoName) || !alliance.equals(lastAlliance)) {
                        autoName = newAutoName;
                        lastAlliance = alliance;
                        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                                try {
                                        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto
                                                        .getPathGroupFromAutoFile(autoName);
                                        List<Pose2d> poses = new ArrayList<>();
                                        for (PathPlannerPath path : pathPlannerPaths) {

                                                if (DriverStation.getAlliance().equals(
                                                                Optional.of(Alliance.Red))) {
                                                        poses.addAll(path.getAllPathPoints()
                                                                        .stream()
                                                                        .map(point -> new Pose2d(
                                                                                        Field.fieldLength
                                                                                                        - point.position.getX(),
                                                                                        Field.fieldWidth - point.position
                                                                                                        .getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors
                                                                                        .toList()));
                                                } else {
                                                        poses.addAll(path.getAllPathPoints()
                                                                        .stream()
                                                                        .map(point -> new Pose2d(
                                                                                        point.position.getX(),
                                                                                        point.position.getY(),
                                                                                        new Rotation2d()))
                                                                        .collect(Collectors
                                                                                        .toList()));
                                                }
                                        }
                                        autoField.getObject("path").setPoses(poses);
                                } catch (IOException e) {
                                        Alert.registerError("Failed to read path file: " + e.getMessage());
                                        return;
                                } catch (ParseException e) {
                                        Alert.registerError("Failed to parse path file: " + e.getMessage());
                                        return;
                                }
                        }
                }
                photonPoseUpdate();
        }

        public void photonPoseUpdate() {
                processCameraPose(photonVision.getCam1Pose(), drivetrain.publisher3);
                processCameraPose(photonVision.getCam2Pose(), drivetrain.publisher4);
                processCameraPose(photonVision.getCam3Pose(), drivetrain.publisher5);
        }

        private void processCameraPose(Optional<EstimatedRobotPose> poseOptional,
                        StructPublisher<Pose3d> publisher) {
                if (poseOptional.isEmpty()) {
                        return;
                }
                EstimatedRobotPose estimatedPose = poseOptional.get();
                Pose3d photonPose = estimatedPose.estimatedPose;

                if (estimatedPose.targetsUsed.isEmpty()) {
                        return;
                }

                // On the field, in the plane of the floor. The Z check was missing entirely, so a
                // solution floating a metre above the carpet used to be fused as if it were real.
                boolean inBounds = photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                && Math.abs(photonPose.getZ()) <= Constants.PhotonVision.kMaxZError;
                if (!inBounds) {
                        return;
                }

                // Reject the whole frame if any tag that fed the solve is too ambiguous. The old
                // loop used `continue`, which only skipped that tag in the distance search — the
                // pose had already been solved using it, so an ambiguous tag still got through.
                double minDist = Double.MAX_VALUE;
                for (var target : estimatedPose.targetsUsed) {
                        if (target.getPoseAmbiguity() > Constants.PhotonVision.kMaxAmbiguity) {
                                return;
                        }
                        double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                        if (dist < minDist)
                                minDist = dist;
                }

                if (minDist < Constants.PhotonVision.kMaxDistance) {
                        double xyStddev = Math.pow(minDist, 2) / 16.0;
                        double rotStddev = Units.degreesToRadians(120.0);
                        SmartDashboard.putNumber("Vision/PhotonVision Future TimeStamp?",Timer.getFPGATimestamp() - estimatedPose.timestampSeconds );
                        drivetrain.addVisionMeasurement(
                                        photonPose.toPose2d(),
                                        estimatedPose.timestampSeconds,
                                        VecBuilder.fill(xyStddev,xyStddev,rotStddev));
                        publisher.set(photonPose);
                }
        }
}