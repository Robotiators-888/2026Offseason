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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.Operator;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.commands.CMD_AimBot;
import frc.robot.commands.CMD_TrenchCrossing;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.SUB_Climber;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;
import frc.robot.utils.Hub;

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
        private final SendableChooser<Command> autoChooser;
        public static final SUB_LEDs leds = SUB_LEDs.getInstance();
        public static final SUB_Shooter shooter = SUB_Shooter.getInstance();
        public static final SUB_Intake intake = SUB_Intake.getInstance();
        public static final SUB_Index index = SUB_Index.getInstance();
        // public static final SUB_Climber climber = SUB_Climber.getInstance();
        public static final PowerDistribution powerDistribution = new PowerDistribution();
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public int listIndex = 0;
        private Boolean lastActiveAlliance = true;
        public double targetRPM = 1000;
        private final StructArrayPublisher<Pose3d> fuelPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("SmartDashboard/Vision/DetectedFuel", Pose3d.struct)
        .publish();

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController Driver1 = new CommandXboxController(Operator.kDriver1ControllerPort);

        private final CommandXboxController Driver2 = new CommandXboxController(Operator.kDriver2ControllerPort);

        // private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
        //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Operator.kDriveDeadband).withRotationalDeadband(MaxAngularRate * Operator.kDriveDeadband) 
            .withDriveRequestType(DriveRequestType.Velocity); //Control is based on speed

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-Driver1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-Driver1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-Driver1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

                intake.setDefaultCommand(new InstantCommand(() -> {
                        intake.set(0);
                        intake.setArm(0);
                }, intake));
                shooter.setDefaultCommand(new RunCommand(() -> {
                        shooter.stop();
                }, shooter));
                index.setDefaultCommand(new InstantCommand(() -> {
                        index.set(0);
                        index.setMeteringSpeed(0);
                }, index));
                leds.setDefaultCommand(new InstantCommand(() -> leds.set(LEDs.kAllianceColor), leds));
                // climber.setDefaultCommand(new InstantCommand(() -> climber.stopClimb(), climber));

                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                //CLimber
                // NamedCommands.registerCommand("ClimbExtend",
                //         new SequentialCommandGroup (
                //                 new RunCommand(()->intake.retractArm(),intake),
                //                 new InstantCommand(() -> climber.climb(), climber),
                //                 new WaitUntilCommand(() -> climber.hasReachedSetPoint(true)),
                //                 new InstantCommand(() -> climber.stopClimb(),climber)
                //         )
                // );

                // NamedCommands.registerCommand("ClimbRetract",
                //         new SequentialCommandGroup (
                //                 new InstantCommand(() -> climber.unClimb(), climber),
                //                 new WaitUntilCommand(() -> climber.hasReachedSetPoint(false)),
                //                 new InstantCommand(() -> climber.stopClimb())
                //         )
                // );
                
                // Intake

                NamedCommands.registerCommand("Intake",
                        new RunCommand(() -> intake.setVolts(Constants.Intake.kINTAKE_MOTOR_VOLTAGE),intake)
                );


                NamedCommands.registerCommand("StopIntake",
                                new InstantCommand(() -> intake.set(0), intake));


                // Favor "Intake" over this
                NamedCommands.registerCommand("DeployIntakeEncoder", Commands.run(() -> intake.intakeArmDown(), intake).until(() -> intake.isArmDownReached() || intake.isForwardPressed()));

                // Shooter and Indexer
                NamedCommands.registerCommand("ManualShoot", Commands.sequence(
                Commands.run(() -> shooter.setRPM(Constants.Shooter.kSHOOTER_FLYWHEEL_RPM), shooter).until(() -> shooter.atDesiredRPM()),
                Commands.run(() -> {
                        shooter.setRPM(Constants.Shooter.kSHOOTER_FLYWHEEL_RPM);
                        index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
                        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                }, shooter, index)
                ));

                NamedCommands.registerCommand("ShootAutoAim", 
                        new CMD_AimBot(
                                drivetrain, 
                                photonVision, 
                                shooter, 
                                index,
                                () -> -(Driver1.getLeftY()), // X Translation
                                () -> -(Driver1.getLeftX())  // Y Translation
                        )
                );

                NamedCommands.registerCommand("ShootDistance", new SequentialCommandGroup(
                                        Commands.run(()->{
                                                double distance = drivetrain.getPose().getTranslation().getDistance(
                                                        SUB_PhotonVision.getInstance().at_field.getTagPose(
                                                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
                                                        ).map(pose -> pose.toPose2d().getTranslation().plus(
                                                                new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 0)
                                                        )).orElse(drivetrain.getPose().getTranslation())
                                                );
                                                shooter.shootMeters(distance);
                                                index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
                                                index.setVolts(-1.0);
                                        },shooter,index).until(() -> shooter.atDesiredRPM()&&Math.abs(index.intakeMeteringRPM()-Constants.Index.kINDEX_METERING_MOTOR_RPM) < 100),
                                        Commands.run(()->{
                                                index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
                                                index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                                        },shooter,index)
                                ));

                NamedCommands.registerCommand("StopShooting", Commands.parallel(
                                new InstantCommand(() -> {
                                        index.set(0);
                                        index.setMeteringSpeed(0);
                                }, index),
                                new InstantCommand(() -> shooter.stop(), shooter)));

                NamedCommands.registerCommand("ShootOnTheMove", 
                        Commands.runOnce(() -> {
                                PPHolonomicDriveController.overrideRotationFeedback(() -> {
                                        Pose2d currentPose = drivetrain.getPose();
                                        var chassisSpeeds = drivetrain.getCurrentRobotChassisSpeeds();
                                        Rotation2d targetRotation = CMD_AimBot.getTargetRotation(
                                                currentPose, 
                                                CMD_AimBot.getTargetTranslation(photonVision), 
                                                CMD_AimBot.shooterOffset, 
                                                chassisSpeeds.vxMetersPerSecond, 
                                                chassisSpeeds.vyMetersPerSecond, 
                                                shooter
                                        );
                                        return CMD_AimBot.calculateRotationalFeedback(currentPose, targetRotation);
                                });
                        })
                        .andThen(new CMD_AimBot(drivetrain, photonVision, shooter, index, () -> 0.0, () -> 0.0))
                        .finallyDo(() -> PPHolonomicDriveController.clearRotationFeedbackOverride())
                );

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
                // Driver1.leftBumper().onTrue(CMD_TrenchCrossing.create(drivetrain));
                Driver1.rightBumper().whileTrue(new RunCommand(() -> {
                        intake.setVolts(Constants.Intake.kINTAKE_MOTOR_VOLTAGE);
                }, intake));
                Driver1.rightTrigger().whileTrue(
                        new CMD_AimBot(
                                drivetrain, 
                                photonVision, 
                                shooter, 
                                index,
                                () -> -(Driver1.getLeftY()),
                                () -> -(Driver1.getLeftX()) 
                        )
                );
                // =========================================================
                // DRIVER 2
                // =========================================================
                Driver2.leftTrigger().whileTrue(new RunCommand(() -> shooter.setRPM(targetRPM), shooter));
                Driver2.rightTrigger().whileTrue(new RunCommand(() -> {
                        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                        index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
                }, index));
                Driver2.y().onTrue(new InstantCommand(() -> targetRPM += 25));
                Driver2.a().onTrue(new InstantCommand(() -> targetRPM -= 25));
                Driver2.leftBumper().whileTrue(new RunCommand(() -> {
                        index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS);
                        index.setMeteringVolts(-Constants.Index.kINDEX_METERING_MOTOR_VOLTS);
                        shooter.setVolts(-2.5);
                }, intake, index, shooter));
                Driver2.povDown().onTrue(Commands.run(()->intake.intakeArmDown(),intake));
                Driver2.povUp().onTrue(Commands.run(()->intake.intakeArmUp(),intake));
                Driver2.rightBumper().whileTrue(new RunCommand(() -> {
                        intake.setArm(MathUtil.applyDeadband(Driver2.getLeftY(), Operator.kDriveDeadband) * Constants.Intake.kINTAKE_ARM_MOTOR_SPEED);
                //        climber.setClimber(MathUtil.applyDeadband(Driver2.getRightY(), Operator.kDriveDeadband) * Constants.Climber.kCLIMBER_MOTOR_SPEED);
                }, intake));//, climber));
                // Driver2.b().whileTrue(new RunCommand(() -> {
                //         intake.setVolts(-Constants.Intake.kINTAKE_MOTOR_VOLTAGE);
                //         index.setMeteringVolts(-Constants.Index.kINDEX_METERING_MOTOR_VOLTS);
                //         index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS);
                //         shooter.setVolts(-2.5);
                // }, intake,index));
                Driver2.b().whileTrue(
                        new RunCommand(() -> {
                                shooter.setRPM(1000);
                                index.setMeteringVolts(Constants.Index.kINDEX_METERING_MOTOR_VOLTS);
                                index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                        })
                );

                
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

                SmartDashboard.putNumber("Stat/Battery Voltage", powerDistribution.getVoltage());
                SmartDashboard.putNumber("Stat/Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
                drivetrain.robotPosePublisher.set(drivetrain.getPose());
                SmartDashboard.putNumber(autoName, listIndex);
                SmartDashboard.putNumber("Shooter/Set RPM (In RobotContainer)",targetRPM);
                SmartDashboard.putNumber("Drivetrain/Angular Velocity Error (dps)", drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
                fuelPublisher.set(photonVision.getFieldFuelPoses(drivetrain.getPose()).toArray(new Pose3d[0]));

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
                try {
                        PathPlannerPath testPath = PathPlannerPath.fromPathFile("TestPath");
                        
                        NamedCommands.getCommand("ShootOnTheMove").schedule();

                        AutoBuilder.followPath(testPath).schedule();
                } catch (Exception e) {
                        DriverStation.reportError("Failed to load TestPath: " + e.getMessage(), e.getStackTrace());
                }
        }

        public void testPeriodic() {
                photonPoseUpdate();
        }

        public void teleopInit() {
                Elastic.selectTab("Teleoperated");
                Elastic.Notification notification = new Elastic.Notification(
                                Elastic.Notification.NotificationLevel.INFO, "I AM STEVE", "CHICKEN JOCKEY!!!!!");
                Elastic.sendNotification(notification);
                Hub.fetchMatchData();
        }

        public void teleopPeriodic() {
                photonPoseUpdate();
                final Optional<Boolean> activeAlliance = Hub.isAllianceHubActive();
                SmartDashboard.putBoolean("Hub/Last Active Alliance", lastActiveAlliance);
                if (activeAlliance.isPresent() && lastActiveAlliance != activeAlliance.get()) {
                        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Active hub change",
                                        "The active hub has changed!"));
                        // Maybe do a rumble
                        lastActiveAlliance = activeAlliance.get();
                }
                SmartDashboard.putNumber("Hub/Time until next alliance change", Hub.getTimeUntilNextChange());
                if (Hub.isAllianceHubActive().isPresent()) {
                        SmartDashboard.putBoolean("Hub/Is our Alliance Active", Hub.isAllianceHubActive().get());
                }
                if ((Hub.getTimeUntilNextChange() <= 3.25 && Hub.getTimeUntilNextChange() >= 2.75)
                                || (Hub.getTimeUntilNextChange() <= 2.25 && Hub.getTimeUntilNextChange() >= 1.75)
                                || (Hub.getTimeUntilNextChange() <= 1.25 && Hub.getTimeUntilNextChange() >= 0.75)) {
                        Driver1.getHID().setRumble(RumbleType.kLeftRumble, 1);
                        Driver2.getHID().setRumble(RumbleType.kLeftRumble, 1);
                } else {
                        Driver1.getHID().setRumble(RumbleType.kLeftRumble, 0);
                        Driver2.getHID().setRumble(RumbleType.kLeftRumble, 0);
                }
                if (shooter.atDesiredRPM() && CMD_AimBot.isThetaErrorCorrect && CMD_AimBot.isRunning()) {
                        Driver1.getHID().setRumble(RumbleType.kRightRumble, 1);
                        Driver2.getHID().setRumble(RumbleType.kRightRumble, 1);
                } else {
                        Driver1.getHID().setRumble(RumbleType.kRightRumble, 0);
                        Driver2.getHID().setRumble(RumbleType.kRightRumble, 0);
                }
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
                        StructPublisher<Pose2d> publisher) {
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
                                        publisher.set(photonPose.toPose2d());
                                }

                                
                        }
                }
        }

}
