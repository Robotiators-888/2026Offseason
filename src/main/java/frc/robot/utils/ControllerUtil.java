package frc.robot.utils;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.CMD_AimBot;
import frc.robot.commands.CMD_Shuttle;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Linear;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Shooter;

public class ControllerUtil {
    



    private CommandSwerveDrivetrain drivetrain;
    private SUB_Linear linear;
    private SUB_Roller roller;
    private SUB_Index index;
    private SUB_PhotonVision photonVision;
    private SUB_Shooter shooter;
    private SUB_Metering metering;
    private SUB_Hood hood;
    private CommandXboxController Driver1;
    private CommandXboxController Driver2;


    /** Target flywheel RPM setpoint in revolutions per minute. */
    public double targetRPM = 1000;

    /** Scheduled command for automatic trench alignment. */
    private Command trenchAlign = Commands.none();

    /** State flag indicating active trench alignment status. */
    private boolean trenchAligning = false;


    /** PathPlanner path definitions for trench alignment and navigation. */
    private PathPlannerPath pathLeftToNeutral;
    private PathPlannerPath pathNeutralToLeft;
    private PathPlannerPath pathRightToNeutral;
    private PathPlannerPath pathNeutralToRight;

    /** Toggle flag for field-relative vs. robot-centric driving mode. */
    public boolean fieldRelative = true;

    /**
     * Constructs a new CommandUtil instance with subsystem references.
     *
     * @param drivetrain Swerve drivetrain subsystem instance.
     * @param linear Linear intake deploy subsystem instance.
     * @param roller Intake roller subsystem instance.
     * @param index Spindexer and feeder subsystem instance.
     * @param photonVision Vision subsystem instance.
     * @param shooter Shooter flywheel subsystem instance.
     */
    public ControllerUtil(CommandSwerveDrivetrain drivetrain, SUB_Linear linear, SUB_Roller roller,
        SUB_Index index, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Hood hood, SUB_Metering metering, CommandXboxController Driver1, CommandXboxController Driver2) {
        this.drivetrain = drivetrain;
        this.linear = linear;
        this.roller = roller;
        this.index = index;
        this.photonVision = photonVision;
        this.shooter = shooter;
        this.hood = hood;
        this.metering = metering;
        this.Driver1 = Driver1;
        this.Driver2 = Driver2;

        try {
                this.pathLeftToNeutral =
                    PathPlannerPath.fromPathFile("LeftTrough-LeftTroughCenter");
                this.pathNeutralToLeft =
                    PathPlannerPath.fromPathFile("LeftTroughCenter-LeftTroughTA");
                this.pathRightToNeutral =
                    PathPlannerPath.fromPathFile("RightTrough-RightTroughCenter");
                this.pathNeutralToRight =
                    PathPlannerPath.fromPathFile("RightTroughCenter-RightTroughTA");
        } catch (Exception e) {
                Alert.registerError("Failed to load trench paths: " + e.getMessage());
        }
    }

    public void configureBindings() {
            configureDriver1Bindings();
            configureDriver2Bindings();
    }

    private void configureDriver1Bindings() {
        // Driver 1 Bindings

        // Left Bumper: Initiate trench alignment sequence
        Driver1.leftBumper()
            .onTrue(Commands.runOnce(() -> {
                trenchAligning = true;
                Pose2d currentPose = drivetrain.getPose();

                Pose2d p1 = AllianceFlipUtil.apply(pathLeftToNeutral != null
                        ? pathLeftToNeutral.getStartingHolonomicPose().orElse(
                                new Pose2d())
                        : new Pose2d());
                Pose2d p2 = AllianceFlipUtil.apply(pathNeutralToLeft != null
                        ? pathNeutralToLeft.getStartingHolonomicPose().orElse(
                                new Pose2d())
                        : new Pose2d());
                Pose2d p3 = AllianceFlipUtil.apply(pathRightToNeutral != null
                        ? pathRightToNeutral.getStartingHolonomicPose().orElse(
                                new Pose2d())
                        : new Pose2d());
                Pose2d p4 = AllianceFlipUtil.apply(pathNeutralToRight != null
                        ? pathNeutralToRight.getStartingHolonomicPose().orElse(
                                new Pose2d())
                        : new Pose2d());

                double d1 =
                    currentPose.getTranslation().getDistance(p1.getTranslation());
                double d2 =
                    currentPose.getTranslation().getDistance(p2.getTranslation());
                double d3 =
                    currentPose.getTranslation().getDistance(p3.getTranslation());
                double d4 =
                    currentPose.getTranslation().getDistance(p4.getTranslation());

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
                        trenchAlign =
                            AutoBuilder
                                .pathfindThenFollowPath(selectedPath, constraints)
                                .until(() -> { return !trenchAligning; });
                        trenchAlign.schedule();
                } catch (Exception e) {
                        Alert.registerError(
                            "Failed to retrieve trench command: " + e.getMessage());
                }
            }))
            .onFalse(new InstantCommand(() -> { trenchAligning = false; }));
        // Right Bumper: Run intake roller 
        Driver1.rightBumper().whileTrue(Commands.run(() -> {
                roller.setRPM(Constants.Roller.kROLLER_MOTOR_RPM);
        }, roller, linear));
        // Right Trigger: AimBot while held
        Driver1.rightTrigger().whileTrue(
            new CMD_AimBot(drivetrain, photonVision, index, hood, metering, shooter, linear,
                () -> - (Driver1.getLeftY()), () -> - (Driver1.getLeftX()), Constants.Linear.kLinearAgitatePeriodics));
        // Left Stick Button: Toggle field-relative driving mode
        Driver1.leftStick().onTrue(
            new InstantCommand(() -> { fieldRelative = !fieldRelative; }));

            
    }
    private void configureDriver2Bindings() {
        // Driver 2 Bindings

        // Left Trigger: Manual Shooter RPM control while held
        Driver2.leftTrigger().whileTrue(
            new RunCommand(() -> shooter.setRPM(targetRPM), shooter));
        // Right Trigger: Manual Indexer control while held
        Driver2.rightTrigger().whileTrue(new RunCommand(() -> {
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
        }, index));
        // Y Button: Increase target RPM by 25
        Driver2.y().onTrue(new InstantCommand(() -> targetRPM += 25));
        // A Button: Decrease target RPM by 25
        Driver2.a().onTrue(new InstantCommand(() -> targetRPM -= 25));
        // B Button: Shuttle
        Driver2.b().whileTrue(new CMD_Shuttle(drivetrain, photonVision, index, shooter,
            () -> - (Driver1.getLeftY()), () -> - (Driver1.getLeftX())));
        // X Button: Set RPM to distance-based value #TODO: Auto hood angle logic
            Driver2.x().onTrue(new InstantCommand(
            ()
                -> targetRPM = shooter.getDistanceRPM(
                        drivetrain.getPose().getTranslation().getDistance(
                            SUB_PhotonVision.getInstance()
                                .at_field
                                .getTagPose(DriverStation.getAlliance().orElse(Alliance.Blue)
                                            == Alliance.Red
                                        ? 10
                                        : 26)
                                .map(pose
                                    -> pose.toPose2d().getTranslation().plus(
                                        new Translation2d(
                                            Units.inchesToMeters(DriverStation.getAlliance()
                                                                    .orElse(Alliance.Blue)
                                                        == Alliance.Red
                                                    ? -23.5
                                                    : 23.5),
                                            0)))
                                .orElse(drivetrain.getPose().getTranslation())))));
    
        // Left Bumper: Unclog indexer and shooter 
        Driver2.leftBumper().whileTrue(new RunCommand(() -> {
            index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS);
            metering.setRPM(-500);
            shooter.setRPM(-500);
        }, index, metering, shooter));
        // POV Up/Down: Manual linear intake control
        Driver2.povDown().onTrue(Commands.run(
            () -> linear.forward(), linear));
        Driver2.povUp().onTrue(Commands.run(
            () -> linear.backward(), linear));
        // POV Right: Manual hood reset
        Driver2.povRight()
            .whileTrue(new RunCommand(() -> hood.set(-.05), hood))
            .onFalse(new InstantCommand(() -> hood.resetEncoder(), hood));
    }

    @Deprecated
    public void updateTrenchAlign() {
        Pose2d currentPose = drivetrain.getPose();

        Pose2d p1 = AllianceFlipUtil.apply(pathLeftToNeutral != null
            ? pathLeftToNeutral.getStartingHolonomicPose().orElse(new Pose2d())
            : new Pose2d());
        Pose2d p2 = AllianceFlipUtil.apply(pathNeutralToLeft != null
            ? pathNeutralToLeft.getStartingHolonomicPose().orElse(new Pose2d())
            : new Pose2d());
        Pose2d p3 = AllianceFlipUtil.apply(pathRightToNeutral != null
            ? pathRightToNeutral.getStartingHolonomicPose().orElse(new Pose2d())
            : new Pose2d());
        Pose2d p4 = AllianceFlipUtil.apply(pathNeutralToRight != null
            ? pathNeutralToRight.getStartingHolonomicPose().orElse(new Pose2d())
            : new Pose2d());

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
    }
}
