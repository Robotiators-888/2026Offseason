// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.Operator;
import frc.robot.commands.CMD_AimBot;
import frc.robot.subsystems.SUB_Climber;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LEDs;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;


import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.Elastic;
import frc.robot.utils.Elastic.Notification;
import frc.robot.utils.Elastic.Notification.NotificationLevel;
import frc.robot.utils.Hub;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();

        private final SendableChooser<Command> autoChooser;
        public static final SUB_LEDs leds = SUB_LEDs.getInstance();
        public static final SUB_Shooter shooter = SUB_Shooter.getInstance();
        public static final SUB_Intake intake = SUB_Intake.getInstance();
        public static final SUB_Index index = SUB_Index.getInstance();
        public static final SUB_Climber climber = SUB_Climber.getInstance();
        public static final PowerDistribution powerDistribution = new PowerDistribution();
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public int listIndex = 0;
        public int targetId = 7;
        private Boolean lastActiveAlliance = true;
        public double targetRPM = 1000;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController Driver1 =
                        new CommandXboxController(Operator.kDriver1ControllerPort);

        private final CommandXboxController Driver2 =
                        new CommandXboxController(Operator.kDriver2ControllerPort);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Operator.kDriveDeadband)
            .withRotationalDeadband(Operator.kDriveDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(-deadbandCompensate(Driver1.getLeftY()) * TunerConstants.kSpeedAt12VoltsMps)
                        .withVelocityY(-deadbandCompensate(Driver1.getLeftX()) * TunerConstants.kSpeedAt12VoltsMps)
                        .withRotationalRate(-deadbandCompensate(Driver1.getRightX()) * Math.PI * 2)));

                intake.setDefaultCommand(new InstantCommand(() -> intake.set(0), intake));
                shooter.setDefaultCommand(new RunCommand(() -> {
                        shooter.stop();
                }, shooter));
                index.setDefaultCommand(new InstantCommand(() -> {
                        index.set(0);
                        index.setMeteringSpeed(0);
                }, index));
                leds.setDefaultCommand(new InstantCommand(() -> leds.set(LEDs.kAllianceColor), leds));
                climber.setDefaultCommand(new InstantCommand(() -> climber.stopClimb(), climber));
                
                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                //CLimber
                NamedCommands.registerCommand("ClimbExtend",
                        new SequentialCommandGroup (
                                new InstantCommand(() -> climber.climb(), climber),
                                new WaitUntilCommand(() -> climber.hasReachedSetPoint(true)),
                                new InstantCommand(() -> climber.stopClimb())
                        )
                );

                NamedCommands.registerCommand("ClimbRetract",
                        new SequentialCommandGroup (
                                new InstantCommand(() -> climber.unClimb(), climber),
                                new WaitUntilCommand(() -> climber.hasReachedSetPoint(false)),
                                new InstantCommand(() -> climber.stopClimb())
                        )
                );
                
                // Intake

                NamedCommands.registerCommand("Intake", Commands.sequence(
                        Commands.either(
                                Commands.none(),              // If true (already extended)
                                intake.extendArm(),           // If false (retracted)
                                intake::isReversePressed
                        ),
                        new RunCommand(() -> intake.set(Constants.Intake.kINTAKE_MOTOR_SPEED),intake)
                ));

                NamedCommands.registerCommand("StopIntake",
                        new InstantCommand(() -> intake.set(0),intake)
                );

                // Shooter and Indexer
                NamedCommands.registerCommand("ManualShoot", Commands.sequence(
                Commands.run(() -> shooter.setRPM(Constants.Shooter.kSHOOTER_FLYWHEEL_RPM), shooter).until(() -> shooter.atDesiredRPM()),
                Commands.run(() -> {
                        shooter.setRPM(Constants.Shooter.kSHOOTER_FLYWHEEL_RPM);
                        index.setMeteringSpeed(Constants.Shooter.kMETERING_SPEED);
                        index.set(Constants.Index.kINDEX_MOTOR_SPEED);
                }, shooter, index)
                ));

                NamedCommands.registerCommand("ShootDistance", Commands.sequence(
                Commands.run(() -> shooter.shootMeters(
                        drivetrain.getPose().getTranslation().getDistance(
                        SUB_PhotonVision.getInstance().at_field.getTagPose(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
                        ).map(pose -> pose.toPose2d().getTranslation()
                                .plus(new Translation2d(
                                Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 
                                0
                                ))
                        ).orElse(drivetrain.getPose().getTranslation())
                        )), shooter)
                        .until(shooter::atDesiredRPM),

                Commands.run(() -> {
                        index.setMeteringSpeed(Constants.Shooter.kMETERING_SPEED); //Maintianting shoot req means we don't need to constantly set the RPM, just make sure it doesn't drop when we start shooting
                        index.set(Constants.Index.kINDEX_MOTOR_SPEED);
                }, shooter, index)
                ));

                NamedCommands.registerCommand("StopShooting", Commands.parallel(
                        new InstantCommand(() -> {
                                index.set(0);
                                index.setMeteringSpeed(0);
                        }, index),
                        new InstantCommand(() -> shooter.stop(), shooter)
                ));

                // Configure the trigger bindings
                configureBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putData("Active Auto Path", autoField);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
         * predicate, or via the named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers
         * or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
         */
        private void configureBindings() {
                //Test Climber
                Driver2.axisLessThan(1,-0.25).whileTrue(new RunCommand(()->climber.setClimber(0.1),climber));
                Driver2.axisGreaterThan(1,0.25).whileTrue(new RunCommand(()->climber.setClimber(-0.1),climber));
                // Test Shooter
                Driver2.leftTrigger().whileTrue(new RunCommand(()->shooter.set(0.2),shooter)); 
                Driver2.rightTrigger().whileTrue(new RunCommand(()->shooter.setVolts(1),shooter)); 
                Driver2.b().whileTrue(new RunCommand(()->shooter.setVolts(2),shooter));
                Driver2.a().whileTrue(new RunCommand(()->shooter.setRPM(targetRPM),shooter));
                Driver2.povUp().onTrue(new InstantCommand(() -> targetRPM += 100));
                Driver2.povDown().onTrue(new InstantCommand(() -> targetRPM -= 100));
                // Test Intake
                Driver1.povUp().whileTrue(new RunCommand(()->intake.setArm(Constants.Intake.kINTAKE_ARM_MOTOR_SPEED),intake));
                Driver1.povDown().whileTrue(new RunCommand(()->intake.setArm(-Constants.Intake.kINTAKE_ARM_MOTOR_SPEED),intake));


                Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading(), drivetrain)); // TODO:change                
                Driver1.rightTrigger().whileTrue(Commands.sequence(
                        Commands.run(() -> shooter.shootMeters(
                        drivetrain.getPose().getTranslation().getDistance(
                                SUB_PhotonVision.getInstance().at_field.getTagPose(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
                                ).map(pose -> pose.toPose2d().getTranslation()
                                .plus(new Translation2d(
                                        Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 
                                        0
                                ))
                                ).orElse(drivetrain.getPose().getTranslation())
                        )), shooter)
                        .until(shooter::atDesiredRPM),
                        
                        Commands.run(() -> {
                        index.setMeteringSpeed(Constants.Shooter.kMETERING_SPEED);
                        index.set(Constants.Index.kINDEX_MOTOR_SPEED);
                        }, shooter,index)
                )); 

                Driver1.leftTrigger().whileTrue(Commands.sequence(
                        Commands.run(() -> shooter.setRPM(Constants.Shooter.kSHOOTER_FLYWHEEL_RPM), shooter)
                        .until(shooter::atDesiredRPM),
                        
                        Commands.run(() -> {
                        shooter.setRPM(Constants.Shooter.kSHOOTER_FLYWHEEL_RPM);
                        index.setMeteringSpeed(Constants.Shooter.kMETERING_SPEED);
                        index.set(Constants.Index.kINDEX_MOTOR_SPEED);
                        }, shooter, index)
                ));
                
                Driver1.x().toggleOnTrue(new CMD_AimBot(drivetrain, photonVision,
                                () -> -MathUtil.applyDeadband(Driver1.getRawAxis(1),
                                                Operator.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(Driver1.getRawAxis(0),
                                                Operator.kDriveDeadband)));



                // Driver1.povLeft().toggleOnTrue(
                //         Commands.sequence(
                //                 intake.retractArm(),
                //                 new WaitUntilCommand(intake::isReversePressed),
                //                 new InstantCommand(() -> climber.setClimberArmToPosition(45),climber)
                //         )     
                // ).toggleOnFalse(
                //         new InstantCommand(() -> climber.setClimberArmToPosition(0),climber)
                // );

                // Driver1.povUp().onTrue(
                //         new InstantCommand(() -> climber.climb(),climber)
                // );

                // Driver1.povDown().onTrue(
                //         new InstantCommand(() -> climber.unClimb(),climber)
                // );

                // Driver1.povUp().onTrue(
                //         new SequentialCommandGroup (
                //                 new InstantCommand(() -> climber.climb(), climber),
                //                 new WaitUntilCommand(() -> climber.hasReachedSetPoint(true))
                //         )
                // );

                // Driver1.povDown().onTrue(
                //         new SequentialCommandGroup (
                //                 new InstantCommand(() -> climber.unClimb(), climber),
                //                 new WaitUntilCommand(() -> climber.hasReachedSetPoint(false))
                //         )
                // );

                Driver1.a().toggleOnTrue(
                        Commands.sequence(
                                Commands.either(
                                        Commands.none(),              // If true (already extended)
                                        intake.extendArm(),           // If false (retracted)
                                        intake::isForwardPressed
                                ),
                                new RunCommand(() -> intake.set(Constants.Intake.kINTAKE_MOTOR_SPEED),intake)
                        )
                ).toggleOnFalse(
                        Commands.sequence(
                                new InstantCommand(() -> intake.set(0),intake)
                        )
                );







        }                

        public double deadbandCompensate(double axis){
                if (Math.abs(axis) < .1){
                        return 0.0;
                }
                else{
                        return Math.copySign((Math.abs(axis) - .1) * (1/0.9), axis);
                }
        }

        public void robotInit() {
                Pathfinding.setPathfinder(new LocalADStar());
                powerDistribution.setSwitchableChannel(true);
        }

        

        public Command getPathCommand(String pathName) {
                Pathfinding.setPathfinder(new LocalADStar());
                try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                        PathConstraints constraints = new PathConstraints(0.5, 0.5,
                                        Units.degreesToRadians(180), Units.degreesToRadians(180)); // unstable
                        return AutoBuilder.pathfindThenFollowPath(path, constraints);
                } catch (Exception e) {
                        Alert.registerError("Failed to retreive path command: " + e.getMessage());
                        return Commands.none();
                }
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

                SmartDashboard.putNumber("Battery Voltage", powerDistribution.getVoltage());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
                autoField.setRobotPose(drivetrain.getPose());
                SmartDashboard.putNumber(autoName, listIndex);
        }

        public void autonomousInit() {
                drivetrain.setIntakeComplete(true);
                drivetrain.setReachedTarget(false);
                Elastic.selectTab("Autonomous");
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {

                        Pose2d currentPose = drivetrain.getPose();

                        SmartDashboard.putNumber("X Error", pose.getX() - currentPose.getX());
                        SmartDashboard.putNumber("Y Error", pose.getY() - currentPose.getY());
                        SmartDashboard.putNumber("Theta Error", pose.getRotation().getRadians()
                                        - currentPose.getRotation().getRadians());
                        SmartDashboard.putNumber("Desired Theta", pose.getRotation().getRadians());
                        SmartDashboard.putNumber("Actual Theta",
                                        currentPose.getRotation().getRadians());

                });
        }

        public void autonomousPeriodic() {
                photonPoseUpdate();
        }

        public void teleopInit() {
                Elastic.selectTab("Teleoperated");
                Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "I AM STEVE", "CHICKEN JOCKEY!!!!!");
                Elastic.sendNotification(notification);
                Hub.fetchMatchData();
        }

        public void teleopPeriodic() {
                photonPoseUpdate();
                final Optional<Boolean> activeAlliance = Hub.isAllianceHubActive();
                SmartDashboard.putBoolean("Last Active Alliance", lastActiveAlliance);
                if (activeAlliance.isPresent() && lastActiveAlliance != activeAlliance.get()) {
                        Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Active hub change", "The active hub has changed!"));
                        // Maybe do a rumble
                        lastActiveAlliance = activeAlliance.get();
                }
                if ((Hub.getTimeUntilNextChange() <= 3.25 && Hub.getTimeUntilNextChange() >= 2.75) || (Hub.getTimeUntilNextChange() <= 2.25 && Hub.getTimeUntilNextChange() >= 1.75) || (Hub.getTimeUntilNextChange() <= 1.25 && Hub.getTimeUntilNextChange() >= 0.75)) {
                        Driver1.getHID().setRumble(RumbleType.kLeftRumble, 1);
                        Driver2.getHID().setRumble(RumbleType.kLeftRumble, 1);
                }
                else {
                        Driver1.getHID().setRumble(RumbleType.kLeftRumble, 0);
                        Driver2.getHID().setRumble(RumbleType.kLeftRumble, 0);                       
                }
                if (shooter.atDesiredRPM() && CMD_AimBot.isThetaErrorCorrect) {
                        Driver1.getHID().setRumble(RumbleType.kRightRumble, 1);
                        Driver2.getHID().setRumble(RumbleType.kRightRumble, 1);
                }
                else if (CMD_AimBot.isThetaErrorCorrect) {
                        Driver1.getHID().setRumble(RumbleType.kRightRumble, .5);
                        Driver2.getHID().setRumble(RumbleType.kRightRumble, .5);
                }
                else {
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

        public static void photonPoseUpdate() {
                processCameraPose(photonVision.getCam1Pose(), drivetrain.publisher3);
                processCameraPose(photonVision.getCam2Pose(), drivetrain.publisher4);
        }

        private static void processCameraPose(Optional<EstimatedRobotPose> poseOptional, StructPublisher<Pose2d> publisher) {
                if (poseOptional.isPresent()) {
                        EstimatedRobotPose estimatedPose = poseOptional.get();
                        Pose3d photonPose = estimatedPose.estimatedPose;
                        
                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0 && photonPose.getY() <= Field.fieldWidth
                                        && !estimatedPose.targetsUsed.isEmpty()) {

                                double minDist = Double.MAX_VALUE;
                                for (var target : estimatedPose.targetsUsed) {
                                        double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                                        if (dist < minDist) minDist = dist;
                                }

                                double xyStddev = Math.pow(minDist, 2) / 16.0;
                                double rotStddev = Units.degreesToRadians(120.0);

                                drivetrain.addVisionMeasurement(
                                                photonPose.toPose2d(),
                                                estimatedPose.timestampSeconds,
                                                VecBuilder.fill(xyStddev, xyStddev, rotStddev));
                                publisher.set(photonPose.toPose2d());
                        }
                }
        }

}
