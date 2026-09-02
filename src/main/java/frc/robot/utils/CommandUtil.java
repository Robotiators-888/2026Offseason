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
            SUB_Index index, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Hood hood, SUB_Metering metering) {
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
         * Registers all named commands with PathPlanner {@link NamedCommands} for autonomous routines.
         */
        public void registerAllNamedCommands() {
                NamedCommands.registerCommand("ReachedTarget",
                    new InstantCommand(

                        () -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                    new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                // Intake
                NamedCommands.registerCommand("Intake",
                    new RunCommand(
                        () -> roller.setRPM(Constants.Roller.kROLLER_MOTOR_RPM), roller));

                // NamedCommands.registerCommand("StopIntake",
                //     new InstantCommand(() -> roller.setRPM(0), roller));

                // Shooter and Indexer
                NamedCommands.registerCommand(
                    "ShootAutoAim", new CMD_AimBotAuto( drivetrain,  photonVision,
             index,  hood,  metering,  shooter)
                );

                NamedCommands.registerCommand(
                    "StopShooting", Commands.parallel(new InstantCommand(() -> {
                            index.set(0);
                    }, index), new InstantCommand(() -> shooter.stop(), shooter)));
        }

}
