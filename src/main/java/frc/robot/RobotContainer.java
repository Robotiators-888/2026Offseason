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
import frc.robot.commands.CMD_AimBotMove;
import frc.robot.commands.CMD_Shuttle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_LEDs;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.AllianceFlipUtil;
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
        public static final SUB_Roller roller = SUB_Roller.getInstance();
        public static final SUB_Arm arm = SUB_Arm.getInstance();
        public static final SUB_Index index = SUB_Index.getInstance();
        public static final PowerDistribution powerDistribution = new PowerDistribution();
        private static String autoName, newAutoName;
        Optional<Alliance> lastAlliance;
        Optional<Alliance> alliance;
        public static Field2d autoField = new Field2d();
        public int listIndex = 0;
        private Boolean lastActiveAlliance = true;
        public double targetRPM = 1000;
        Field2d field;
        private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.0,-8.0,0.0);
        private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.0,-8.0,0.0);
        private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4.0,-8.0,0.0);
        private boolean isTeleop = false;
        private boolean isShaking = false; 
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

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                field = new Field2d();
                try {
                        pathLeftToNeutral = PathPlannerPath.fromPathFile("LeftTrough-LeftTroughCenter");
                        pathNeutralToLeft = PathPlannerPath.fromPathFile("LeftTroughCenter-LeftTrough");
                        pathRightToNeutral = PathPlannerPath.fromPathFile("RightTrough-RightTroughCenter");
                        pathNeutralToRight = PathPlannerPath.fromPathFile("RightTroughCenter-RightTrough");
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
                        index.setMeteringSpeed(0);
                }, index));
                leds.setDefaultCommand(new InstantCommand(() -> leds.set(LEDs.kAllianceColor), leds));

                NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                
                // Intake

                NamedCommands.registerCommand("Intake",
                        new RunCommand(() -> {
                                arm.intakeArmTest();
                                roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE);
                        }
                                
                        ,arm,roller)
                );


                NamedCommands.registerCommand("StopIntake",
                                Commands.parallel(
                                        new InstantCommand(() -> roller.set(0), roller),
                                        new InstantCommand(() -> arm.setArm(0), arm)
                                )
                );

                NamedCommands.registerCommand("DeployIntakeEncoder", Commands.run(() -> arm.intakeArmDown(), arm).until(() -> arm.isArmDownReached() || arm.isForwardPressed()));

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
                        new CMD_AimBotAuto(
                                drivetrain, 
                                photonVision, 
                                shooter, 
                                index
                        )
                );

                // NamedCommands.registerCommand("IntakeWiggle",
                //         new RunCommand(() -> 
                //                 intake.intakeWiggle()
                //         , intake)
                // );
                
                NamedCommands.registerCommand("IntakeAgitate",
                        getShakeyCommand()
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
                Driver1.rightBumper().toggleOnTrue(Commands.run(() -> {
                        if (arm.isArmDownReached()) {
                                roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE);
                                arm.intakeArmTest();
                        } else {
                                roller.setVolts(0);
                                arm.setArm(0);
                        }
                }, roller, arm).repeatedly());
                Driver1.rightTrigger().whileTrue(
                        new ParallelCommandGroup(
                                new CMD_AimBotMove(
                                        drivetrain, 
                                        photonVision, 
                                        shooter, 
                                        index
                                ),
                                NamedCommands.getCommand("IntakeAgitate")
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
                        index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
                }, index));
                Driver2.y().onTrue(new InstantCommand(() -> targetRPM += 25));
                Driver2.a().onTrue(new InstantCommand(() -> targetRPM -= 25));
                Driver2.leftBumper().whileTrue(new RunCommand(() -> {
                        index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS);
                        index.setMeteringVolts(-Constants.Index.kINDEX_METERING_MOTOR_VOLTS);
                        shooter.setVolts(-2.5);
                }, index, shooter));
                Driver2.povDown().onTrue(Commands.run(()->arm.intakeArmDown(),arm));
                Driver2.povUp().onTrue(Commands.run(()->arm.intakeArmUp(),arm));
                // Driver2.rightBumper().whileTrue(new RunCommand(() -> {
                //         arm.setArm(MathUtil.applyDeadband(Driver2.getLeftY(), Operator.kDriveDeadband) * Constants.Arm.kARM_MOTOR_SPEED);
                // }, arm));

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
                Driver2.leftStick().whileTrue(NamedCommands.getCommand("IntakeAgitate"));
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
                logDrivetrain();
                checkAlerts();
        }

        // Logs everything about the drivetrain
        public void logDrivetrain () {
                drivetrain.swerveModuleStatesPublisher.set(drivetrain.getState().ModuleStates);
                drivetrain.desiredSwerveModuleStatesPublisher.set(drivetrain.getState().ModuleTargets);
                for (int i = 0; i < drivetrain.getModules().length; i++) {
                        TalonFX driveMotor = drivetrain.getModule(i).getDriveMotor();
                        TalonFX steerMotor = drivetrain.getModule(i).getSteerMotor();
                        int driveMotorId = driveMotor.getDeviceID();
                        int steerMotorId = steerMotor.getDeviceID();
                        SmartDashboard.putNumber("Drivetrain/Motors/Current/Drive Motor ID " + driveMotorId + " Stator Current", driveMotor.getStatorCurrent().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Current/Steer Motor ID " + steerMotorId + " Stator Current", steerMotor.getStatorCurrent().getValueAsDouble());

                        SmartDashboard.putNumber("Drivetrain/Motors/Current/Drive Motor ID " + driveMotorId + " Supply Current", driveMotor.getSupplyCurrent().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Current/Steer Motor ID " + steerMotorId + " Supply Current", steerMotor.getSupplyCurrent().getValueAsDouble());
                        

                        SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Drive Motor ID " + driveMotorId + " Motor Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Steer Motor ID " + steerMotorId + " Motor Voltage", steerMotor.getMotorVoltage().getValueAsDouble());
                        
                        SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Drive Motor ID " + driveMotorId + " Supply Voltage", driveMotor.getSupplyVoltage().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Steer Motor ID " + steerMotorId + " Supply Voltage", steerMotor.getSupplyVoltage().getValueAsDouble());
                        

                        SmartDashboard.putNumber("Drivetrain/Motors/RPM/Drive Motor ID " + driveMotorId + " RPM", driveMotor.getVelocity().getValue().baseUnitMagnitude());
                        SmartDashboard.putNumber("Drivetrain/Motors/RPM/Steer Motor ID " + steerMotorId + " RPM", steerMotor.getVelocity().getValue().baseUnitMagnitude());
                        
                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Drive Motor ID " + driveMotorId + " Encoder Pos", driveMotor.getPosition().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Steer Motor ID " + steerMotorId + " Encoder Pos", steerMotor.getPosition().getValueAsDouble());

                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Drive Motor ID " + driveMotorId + " Torque Current", driveMotor.getTorqueCurrent().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Steer Motor ID " + steerMotorId + " Torque Current", steerMotor.getTorqueCurrent().getValueAsDouble());

                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Drive Motor ID " + driveMotorId + " Device Temp", driveMotor.getDeviceTemp().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Steer Motor ID " + steerMotorId + " Device Temp", steerMotor.getDeviceTemp().getValueAsDouble());

                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Drive Motor ID " + driveMotorId + " Processor Temp", driveMotor.getProcessorTemp().getValueAsDouble());
                        SmartDashboard.putNumber("Drivetrain/Motors/Pos/Steer Motor ID " + steerMotorId + " Processor Temp", steerMotor.getProcessorTemp().getValueAsDouble());
                        
                        SmartDashboard.putNumber("Drivetrain/Motors/AbsEncoder/Encoder ID " + drivetrain.getModule(i).getEncoder().getDeviceID() + " Position", drivetrain.getModule(i).getEncoder().getPosition().getValueAsDouble());
                }
        }

        private void checkAlerts () {
                for (int i = 0; i < drivetrain.getModules().length; i++) {
                        Alert.alertKraken(drivetrain.getModule(i).getDriveMotor());
                        Alert.alertKraken(drivetrain.getModule(i).getSteerMotor());
                }
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
                isTeleop = false;
                photonPoseUpdate();
        }


        public void testInit() {
        }

        public void testPeriodic() {
                isTeleop = false;
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
                isTeleop = true;
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
                isTeleop = false;
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

        private Command getShakeyCommand () {
                
                Command c = new ParallelCommandGroup(
                                new InstantCommand(()->{isShaking=true;}),
                                new RunCommand(()->roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE), roller),
                                new SequentialCommandGroup(
                                        new RunCommand(()->arm.setArm(.15), arm).withTimeout(.4),
                                        new RunCommand(()->arm.setArm(-.1), arm).withTimeout(.4)
                                ).repeatedly()
                        ).finallyDo(()->{isShaking=false;});
                return c;
        }

}
