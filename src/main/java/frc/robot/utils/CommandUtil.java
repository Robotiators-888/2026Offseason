package frc.robot.utils;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.CMD_AimBotAuto;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Linear;
import frc.robot.subsystems.SUB_Roller;

public class CommandUtil {
    /** Longest a shot may spend waiting for the flywheel before the auto gives up and moves on. */
    private static final double kSpinUpTimeoutSeconds = 2.0;

    /** How long to hold the feed open once the flywheel is at speed. */
    private static final double kFeedSeconds = 1.5;

    /** How long the intake runs when an auto asks for it. */
    private static final double kIntakeSeconds = 2.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final SUB_Linear linear;
    private final SUB_Roller roller;
    private final SUB_Index index;
    private final SUB_PhotonVision photonVision;
    private final SUB_Shooter shooter;
    private final SUB_Hood hood;
    private final SUB_Metering metering;

    public CommandUtil (CommandSwerveDrivetrain drivetrain, SUB_Linear linear, SUB_Roller roller, SUB_Index index, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Hood hood, SUB_Metering metering){
        this.drivetrain = drivetrain;
        this.linear = linear;
        this.roller = roller;
        this.index = index;
        this.photonVision = photonVision;
        this.shooter = shooter;
        this.hood = hood;
        this.metering = metering;
    }

    public void registerAllNamedCommands(){
                // Intake

                NamedCommands.registerCommand("Intake",
                        new RunCommand(() -> {
                                linear.forward();
                                roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE);
                        }

                        ,linear,roller).withTimeout(kIntakeSeconds)
                );


                // The arm is deliberately left where it is; only the rollers stop.
                NamedCommands.registerCommand("StopIntake",
                                new InstantCommand(() -> roller.set(0), roller));

                NamedCommands.registerCommand("DeployIntakeEncoder", new InstantCommand(() -> linear.forward(), linear));

                // Shooter and Indexer
                NamedCommands.registerCommand("ShootAutoAim",
                        new CMD_AimBotAuto(
                                drivetrain,
                                photonVision,
                                shooter,
                                index,
                                hood,
                                metering
                        ).withTimeout(kSpinUpTimeoutSeconds + kFeedSeconds)
                );

                NamedCommands.registerCommand("IntakeAgitate",
                        getLinearCompress().withTimeout(kIntakeSeconds)
                );

                // Shoot at the range implied by the current pose without taking the drivetrain, so
                // the path keeps running underneath it.
                NamedCommands.registerCommand("ShootAndMove", buildRangedShot());

                NamedCommands.registerCommand("StopShooting", Commands.parallel(
                                new InstantCommand(() -> {
                                        index.set(0);

                                }, index),
                                new InstantCommand(() -> {
                                        shooter.stop();
                                        shooter.setHighPower(false);
                                }, shooter)));
    }

        /**
         * A self-terminating shot that does not require the drivetrain: spin up to the
         * distance-appropriate RPM, then hold the feed open for a fixed window.
         *
         * <p>Both stages are bounded, so an auto sequencing this can never hang on it.
         */
        private Command buildRangedShot () {
                return Commands.sequence(
                        Commands.run(() -> {
                                shooter.setHighPower(true);
                                shootAtCurrentRange();
                        }, shooter)
                                .until(() -> shooter.atDesiredRPM())
                                .withTimeout(kSpinUpTimeoutSeconds),
                        Commands.run(() -> {
                                shootAtCurrentRange();
                                index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                        }, shooter, index)
                                .withTimeout(kFeedSeconds)
                ).finallyDo(() -> {
                        index.set(0);
                        shooter.setHighPower(false);
                });
        }

        /** Commands the flywheel for the robot's current distance to the hub, if that is known. */
        private void shootAtCurrentRange () {
                Hub.getDistanceToGoal(drivetrain.getPose()).ifPresent(shooter::shootMeters);
        }

        public Command getLinearCompress () {
                return new RunCommand(() -> linear.backward(), linear);
        }
}
