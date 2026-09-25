package frc.robot.utils;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.CMD_AimBotAuto;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Linear;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Roller;
import frc.robot.subsystems.SUB_Shooter;

/**
 * Utility helper class for building and registering named commands for PathPlanner auto routines.
 */
public class CommandUtil {
        private CommandSwerveDrivetrain drivetrain;
        private SUB_Linear linear;
        private SUB_Roller roller;
        private SUB_Index index;
        private SUB_PhotonVision photonVision;
        private SUB_Shooter shooter;
        private SUB_Metering metering;
        private SUB_Hood hood;

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
        public CommandUtil(CommandSwerveDrivetrain drivetrain, SUB_Linear linear, SUB_Roller roller,
            SUB_Index index, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Hood hood,
            SUB_Metering metering) {
                this.drivetrain = drivetrain;
                this.linear = linear;
                this.roller = roller;
                this.index = index;
                this.photonVision = photonVision;
                this.shooter = shooter;
                this.hood = hood;
                this.metering = metering;
        }

        /**
         * Deploys the intake linear mechanism and runs intake rollers.
         *
         * @return Command representing deploy and intake action.
         */
        public Command deployAndIntake() {
                return Commands.sequence(
                    Commands.runOnce(() -> linear.forward(), linear),
                    Commands.run(() -> roller.setRPM(Constants.Roller.kROLLER_MOTOR_RPM), roller)
                ).withName("CommandUtil.deployAndIntake");
        }

        /**
         * Stops the intake rollers.
         *
         * @return Command to stop intake rollers.
         */
        public Command stopIntake() {
                return Commands.runOnce(() -> roller.stop(), roller)
                    .withName("CommandUtil.stopIntake");
        }

        /**
         * Waits until shooter flywheel and hood are at tolerance, then feeds game piece into shooter.
         *
         * @return Command that waits for tolerances and feeds indexer and metering wheels.
         */
        public Command checkAndFeed() {
                return Commands.sequence(
                    Commands.waitUntil(() -> shooter.atDesiredRPM() && hood.atDesiredAngle())
                        .withTimeout(2.0),
                    Commands.parallel(
                        Commands.run(() -> metering.setRPM(Constants.Metering.kMETERING_MOTOR_RPM), metering),
                        Commands.run(() -> index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS), index)
                    )
                ).withName("CommandUtil.checkAndFeed");
        }

        /**
         * Stops shooter flywheel, metering, and indexer motors.
         *
         * @return Command stopping feed mechanisms.
         */
        public Command stopShooterIndexer() {
                return Commands.parallel(
                    Commands.runOnce(() -> shooter.stop(), shooter),
                    Commands.runOnce(() -> metering.stop(), metering),
                    Commands.runOnce(() -> index.stop(), index)
                ).withName("CommandUtil.stopShooterIndexer");
        }

        /**
         * Reverses all intake and indexer motors to clear jams.
         *
         * @return Command running unjam sequence.
         */
        public Command reverseAll() {
                return Commands.parallel(
                    Commands.run(() -> roller.setVolts(-6.0), roller),
                    Commands.run(() -> index.setVolts(-Constants.Index.kINDEX_MOTOR_VOLTS), index),
                    Commands.run(() -> metering.setRPM(-500), metering)
                ).withName("CommandUtil.reverseAll");
        }

        /**
         * Stops all mechanism motors safely.
         *
         * @return Command stopping all mechanisms.
         */
        public Command stopAll() {
                return Commands.parallel(
                    Commands.runOnce(() -> shooter.stop(), shooter),
                    Commands.runOnce(() -> roller.stop(), roller),
                    Commands.runOnce(() -> index.stop(), index),
                    Commands.runOnce(() -> metering.stop(), metering)
                ).withName("CommandUtil.stopAll");
        }

        /**
         * Retracts linear intake and pulses rollers briefly to clear pieces.
         *
         * @return Command for tucking intake.
         */
        public Command tuckAndClear() {
                return Commands.sequence(
                    Commands.runOnce(() -> roller.setVolts(2.0), roller),
                    new WaitCommand(0.5),
                    Commands.runOnce(() -> linear.backward(), linear),
                    Commands.runOnce(() -> roller.stop(), roller)
                ).withName("CommandUtil.tuckAndClear");
        }

        /**
         * Registers all named commands with PathPlanner {@link NamedCommands} for autonomous
         * routines.
         */
        public void registerAllNamedCommands() {
                NamedCommands.registerCommand("ReachedTarget",
                    new InstantCommand(() -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                    new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                // Intake
                NamedCommands.registerCommand("Intake", deployAndIntake());
                NamedCommands.registerCommand("StopIntake", stopIntake());
                NamedCommands.registerCommand("TuckAndClear", tuckAndClear());

                // Feeding and unjamming
                NamedCommands.registerCommand("CheckAndFeed", checkAndFeed());
                NamedCommands.registerCommand("ReverseAll", reverseAll());
                NamedCommands.registerCommand("StopAll", stopAll());

                // Shooter and Indexer
                NamedCommands.registerCommand("ShootAutoAim",
                    new CMD_AimBotAuto(
                        drivetrain, photonVision, index, hood, metering, shooter, linear));

                NamedCommands.registerCommand("StopShooting", stopShooterIndexer());
        }
}
