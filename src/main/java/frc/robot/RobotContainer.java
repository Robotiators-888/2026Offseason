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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Field;
import frc.robot.Constants.LEDs;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.AutoGenerator;
import frc.robot.utils.Elastic;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private static final SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
        private static final SUB_PhotonVision photonVision = SUB_PhotonVision.getInstance();
        private static final AutoGenerator autoGenerator = AutoGenerator.getInstance();
        private final SendableChooser<Command> autoChooser;
        public static final SUB_LEDs leds = SUB_LEDs.getInstance();
        public static final SUB_Shooter shooter = SUB_Shooter.getInstance();
        public static final SUB_Intake intake = SUB_Intake.getInstance();
        public static final SUB_Index index = SUB_Index.getInstance();
        public static final PowerDistribution powerDistribution = new PowerDistribution();
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public int listIndex = 0;
        public int targetId = 7;

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController Driver1 =
                        new CommandXboxController(Operator.kDriver1ControllerPort);

        private final CommandXboxController Driver2 =
                        new CommandXboxController(Operator.kDriver2ControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                drivetrain.setDefaultCommand(new RunCommand( // Unstable
                                () -> drivetrain.drive(
                                                MathUtil.applyDeadband(Driver1.getRawAxis(1),
                                                                Operator.kDriveDeadband),
                                                MathUtil.applyDeadband(Driver1.getRawAxis(0),
                                                                Operator.kDriveDeadband),
                                                -MathUtil.applyDeadband(Driver1.getRawAxis(4),
                                                                Operator.kDriveDeadband),
                                                true, true),
                                drivetrain));

                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> autoGenerator.setreachedtarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> autoGenerator.setreachedtarget(false)));


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

                Driver1.leftStick().onTrue(new InstantCommand(() -> drivetrain.zeroHeading(), drivetrain)); // TODO:
                                                                                                // Change
                
                Driver2.rightTrigger().whileTrue(new RunCommand(() -> shooter.set(Constants.Shooter.kSHOOTER_FLYWHEEL_MOTOR_SPEED), shooter).andThen(new WaitCommand(0.75)).andThen(new RunCommand(() -> index.set(Constants.Index.kINDEX_MOTOR_SPEED), index))).onFalse(new ParallelCommandGroup(new InstantCommand(() -> index.set(0), index), new InstantCommand(() -> shooter.set(0), shooter)));
                Driver2.rightBumper().whileTrue(new InstantCommand(() -> intake.set(Constants.Intake.kINTAKE_MOTOR_SPEED), intake)).onFalse(new InstantCommand(() -> intake.set(0), intake));
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
                        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
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
        }

        public void autonomousInit() {
                autoGenerator.setintakecomplete(true);
                autoGenerator.setreachedtarget(false);
                Elastic.selectTab("Autonomous");
                leds.set(LEDs.kParty_Palette_Twinkles);
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
                photonAutonPoseUpdate();
        }

        public void teleopInit() {
                leds.setAllianceColor();
                Elastic.selectTab("Teleoperated");
                Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "I AM STEVE", "CHICKEN JOCKEY!!!!!");
                Elastic.sendNotification(notification);
        }

        public void teleopPeriodic() {
                photonPoseUpdate();
        }

        public void disabledPeriodic() {
                newAutoName = getAutonomousCommand().getName();
                alliance = DriverStation.getAlliance();
                if (autoName != newAutoName || alliance != lastAlliance) {
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
                                        e.printStackTrace();
                                        return;
                                } catch (ParseException e) {
                                        e.printStackTrace();
                                        return;
                                }
                        }
                }
                photonPoseUpdate();
        }

        public static void photonPoseUpdate() {
                Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0
                                        && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam1BestTarget() != null) {

                                Pose2d closestTag = photonVision.at_field.getTagPose(
                                                photonVision.getCam1BestTarget().getFiducialId())
                                                .get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d())
                                                .getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / (8.0088 * 0.5);
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(
                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                        }
                }

                photonPoseOptional = photonVision.getCam2Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0
                                        && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam2BestTarget() != null) {

                                Pose2d closestTag = photonVision.at_field.getTagPose(
                                                photonVision.getCam2BestTarget().getFiducialId())
                                                .get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d())
                                                .getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 2) / 8.0088;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher4.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(
                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);

                                drivetrain.publisher4.set(photonPose.toPose2d());
                        }
                }
        }

        public static void photonAutonPoseUpdate() {
                Optional<EstimatedRobotPose> photonPoseOptional = photonVision.getCam1Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0
                                        && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam1BestTarget() != null) {

                                Pose2d closestTag = photonVision.at_field.getTagPose(
                                                photonVision.getCam1BestTarget().getFiducialId())
                                                .get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d())
                                                .getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 1.75) * (3 * (Math.sqrt(Math.pow(drivetrain.getChassisSpeeds().vxMetersPerSecond,2)+Math.pow(drivetrain.getChassisSpeeds().vyMetersPerSecond,2)))/ 4.92) / 3.6;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(
                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);
                                drivetrain.publisher3.set(photonPose.toPose2d());
                        }
                }

                photonPoseOptional = photonVision.getCam2Pose();

                if (photonPoseOptional.isPresent()) {
                        Pose3d photonPose = photonPoseOptional.get().estimatedPose;

                        if (photonPose.getX() >= 0 && photonPose.getX() <= Field.fieldLength
                                        && photonPose.getY() >= 0
                                        && photonPose.getY() <= Field.fieldWidth
                                        && photonVision.getCam2BestTarget() != null) {

                                Pose2d closestTag = photonVision.at_field.getTagPose(
                                                photonVision.getCam2BestTarget().getFiducialId())
                                                .get().toPose2d();
                                Translation2d translate = closestTag.minus(photonPose.toPose2d())
                                                .getTranslation();

                                double distance = translate.getNorm();
                                double xStddev = Math.pow(distance, 1.75) * (3 * (Math.sqrt(Math.pow(drivetrain.getChassisSpeeds().vxMetersPerSecond,2)+Math.pow(drivetrain.getChassisSpeeds().vyMetersPerSecond,2)))/ 4.92) / 3.6;
                                double yStddev = xStddev;
                                double rotStddev = Units.degreesToRadians(120.0);
                                drivetrain.publisher4.set(photonPose.toPose2d());
                                drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(
                                                VecBuilder.fill(xStddev, yStddev, rotStddev));
                                drivetrain.addVisionMeasurement(photonPose.toPose2d(),
                                                photonPoseOptional.get().timestampSeconds);

                                drivetrain.publisher4.set(photonPose.toPose2d());
                        }
                }
        }
}
