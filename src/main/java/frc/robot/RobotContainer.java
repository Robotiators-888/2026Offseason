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
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.Operator;
import frc.robot.commands.CMD_AimBot;
import frc.robot.commands.CMD_AimBotAuto;
import frc.robot.commands.CMD_AimBotSpecialLock;
import frc.robot.commands.CMD_PredictiveAim;
import frc.robot.commands.CMD_PredictiveAimAuto;
import frc.robot.commands.CMD_Shuttle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.CommandUtil;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;
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
        public static final SUB_Arm arm = SUB_Arm.getInstance();
        public static final SUB_Index index = SUB_Index.getInstance();
        public static final PowerDistribution powerDistribution = new PowerDistribution();
        public final CommandUtil commandUtil = new CommandUtil(drivetrain, arm, roller, index, photonVision, shooter);
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

                arm.setDefaultCommand(new RunCommand(() -> {
                        arm.setArm(0);
                }, arm));
                shooter.setDefaultCommand(new RunCommand(() -> {
                        shooter.stop();
                }, shooter));
                index.setDefaultCommand(new InstantCommand(() -> {
                        index.set(0);
                        
                }, index));

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
                        Pose2d currentPose = drivetrain.getPose();
                        
                        Pose2d p1 = AllianceFlipUtil.apply(pathLeftToNeutral != null ? pathLeftToNeutral.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());
                        Pose2d p2 = AllianceFlipUtil.apply(pathNeutralToLeft != null ? pathNeutralToLeft.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());
                        Pose2d p3 = AllianceFlipUtil.apply(pathRightToNeutral != null ? pathRightToNeutral.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());
                        Pose2d p4 = AllianceFlipUtil.apply(pathNeutralToRight != null ? pathNeutralToRight.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());

                        double d1 = currentPose.getTranslation().getDistance(p1.getTranslation());
                        double d2 = currentPose.getTranslation().getDistance(p2.getTranslation());
                        double d3 = currentPose.getTranslation().getDistance(p3.getTranslation());
                        double d4 = currentPose.getTranslation().getDistance(p4.getTranslation());

                        double minD = Math.min(Math.min(d1, d2), Math.min(d3, d4));
                        PathPlannerPath selectedPath = pathNeutralToRight;

                        if (minD == d1) {
                                selectedPath = pathLeftToNeutral;
                        } else if (minD == d2) {
                                selectedPath = pathNeutralToLeft;
                        } else if (minD == d3) {
                                selectedPath = pathRightToNeutral;
                        }

                        try {
                                PathConstraints constraints = new PathConstraints(4.0, 4.0,
                                                Units.degreesToRadians(360), Units.degreesToRadians(540));
                                trenchAlign = AutoBuilder.pathfindThenFollowPath(selectedPath, constraints).until(()->{
                                        return !trenchAligning;
                                });
                                trenchAlign.schedule();
                        } catch (Exception e) {
                                Alert.registerError("Failed to retrieve trench command: " + e.getMessage());
                        }
                })).onFalse(new InstantCommand(()->{trenchAligning=false;}));
                Driver1.rightBumper().whileTrue(Commands.run(() -> {
                        roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE);
                        arm.intakeArmTest();
                }, roller, arm));
                Driver1.leftTrigger().whileTrue(
                        new ParallelCommandGroup(
                                new CMD_PredictiveAim(
                                        drivetrain, 
                                        photonVision, 
                                        shooter, 
                                        index,
                                        () -> -(Driver1.getLeftY()),
                                        () -> -(Driver1.getLeftX()) 
                                ),
                                new RunCommand(()->roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE), roller)
                        )
                );
                Driver1.rightTrigger().whileTrue(
                        new ParallelCommandGroup(
                                new CMD_AimBot(
                                        drivetrain, 
                                        photonVision, 
                                        shooter, 
                                        index,
                                        () -> -(Driver1.getLeftY()),
                                        () -> -(Driver1.getLeftX()) 
                                ),
                                getCancellableShakeyCommand(() -> Driver2.leftStick().getAsBoolean())
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
                Driver2.y().onTrue(new InstantCommand(() -> targetRPM += 25));
                Driver2.a().onTrue(new InstantCommand(() -> targetRPM -= 25));
                Driver2.leftBumper().whileTrue(new RunCommand(() -> {
                        index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS);
                        
                        shooter.setVolts(-2.5);
                }, index, shooter));
                Driver2.povDown().onTrue(Commands.run(()->arm.intakeArmDown(),arm));
                Driver2.povUp().onTrue(Commands.run(()->arm.intakeArmUp(),arm));
                Driver2.rightBumper().whileTrue(new RunCommand(() -> {
                        arm.setArm(MathUtil.applyDeadband(Driver2.getLeftY(), Operator.kDriveDeadband) * Constants.Arm.kARM_MOTOR_SPEED);
                }, arm));

                Driver2.b().whileTrue(
                        new CMD_Shuttle(drivetrain, photonVision, index, shooter,
                                () -> -(Driver1.getLeftY()),
                                () -> -(Driver1.getLeftX())
                        )
                );
                Driver2.x().onTrue(new InstantCommand(() -> targetRPM = shooter.getDistanceRPM(
                        drivetrain.getPose().getTranslation().getDistance(
                                SUB_PhotonVision.getInstance().at_field.getTagPose(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
                                ).map(pose -> pose.toPose2d().getTranslation().plus(
                                        new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 0)
                        )).orElse(drivetrain.getPose().getTranslation())
                ))));
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

        public void robotPeriodic() {
                SmartDashboard.putNumber("Stat/Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
                drivetrain.robotPosePublisher.set(drivetrain.getPose());
                field.setRobotPose(drivetrain.getPose());
                SmartDashboard.putData("Drivetrain/Field", field);
                SmartDashboard.putNumber(autoName, listIndex);
                SmartDashboard.putNumber("Shooter/Set RPM (In RobotContainer)",targetRPM);
                SmartDashboard.putNumber("Drivetrain/Angular Velocity Error (dps)", drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
                Pose2d currentPose = drivetrain.getPose();
                
                Pose2d p1 = AllianceFlipUtil.apply(pathLeftToNeutral != null ? pathLeftToNeutral.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());
                Pose2d p2 = AllianceFlipUtil.apply(pathNeutralToLeft != null ? pathNeutralToLeft.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());
                Pose2d p3 = AllianceFlipUtil.apply(pathRightToNeutral != null ? pathRightToNeutral.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());
                Pose2d p4 = AllianceFlipUtil.apply(pathNeutralToRight != null ? pathNeutralToRight.getStartingHolonomicPose().orElse(new Pose2d()) : new Pose2d());

                drivetrain.testPath1Publisher.set(p1);
                drivetrain.testPath2Publisher.set(p2);
                drivetrain.testPath3Publisher.set(p3);
                drivetrain.testPath4Publisher.set(p4);

                double d1 = currentPose.getTranslation().getDistance(p1.getTranslation());
                double d2 = currentPose.getTranslation().getDistance(p2.getTranslation());
                double d3 = currentPose.getTranslation().getDistance(p3.getTranslation());
                double d4 = currentPose.getTranslation().getDistance(p4.getTranslation());

                double minD = Math.min(Math.min(d1, d2), Math.min(d3, d4));
                String closestTrough = "";
                Pose2d closestPose = p1;

                if (minD == d1) {
                        closestTrough = "Left Trough -> Neutral Zone";
                        closestPose = p1;
                } else if (minD == d2) {
                        closestTrough = "Neutral Zone -> Left Trough";
                        closestPose = p2;
                } else if (minD == d3) {
                        closestTrough = "Right Trough -> Neutral Zone";
                        closestPose = p3;
                } else {
                        closestTrough = "Neutral Zone -> Right Trough";
                        closestPose = p4;
                }

                drivetrain.selectedTestPathPublisher.set(closestPose);
                SmartDashboard.putString("Trough/Closest", closestTrough);
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
                newAutoName = getAutonomousCommand().getName();
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
                if (poseOptional.isPresent()) {
                        EstimatedRobotPose estimatedPose = poseOptional.get();
                        Pose3d photonPose = estimatedPose.estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && !estimatedPose.targetsUsed.isEmpty()) {

                                double minDist = Double.MAX_VALUE;
                                for (var target : estimatedPose.targetsUsed) {
                                        if (target.getPoseAmbiguity()>0.2) continue;
                                        double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                                        if (dist < minDist)
                                                minDist = dist;
                                }

                                if (minDist<4.0) {
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
        }

        private Command getCancellableShakeyCommand (BooleanSupplier condition) {
                Command c = new ParallelCommandGroup(
                        new RunCommand(()->roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE), roller),
                        new SequentialCommandGroup(
                                Commands.either(new RunCommand(()->arm.setArm(0), arm).withTimeout(.4), new RunCommand(()->arm.setArm(.15), arm).withTimeout(.4), condition),
                                Commands.either(new RunCommand(()->arm.setArm(0), arm).withTimeout(.4), new RunCommand(()->arm.setArm(-.13), arm).withTimeout(.4), condition)
                        ).repeatedly()
                );
                return c;
        }

}