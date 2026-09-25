package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

/**
 * Coordination layer combining multiple individual subsystems into coherent robot-level actions.
 * Follows the Team 449 RobotActions pattern for clean command composition.
 */
public class RobotActions {
    private final SUB_Shooter shooter;
    private final SUB_Hood hood;
    private final SUB_Metering metering;
    private final SUB_Index index;
    private final SUB_Roller roller;
    private final SUB_Linear linear;

    /**
     * Constructs a RobotActions instance.
     *
     * @param shooter Shooter subsystem.
     * @param hood Hood subsystem.
     * @param metering Metering subsystem.
     * @param index Index subsystem.
     * @param roller Roller intake subsystem.
     * @param linear Linear deployment subsystem.
     */
    public RobotActions(SUB_Shooter shooter, SUB_Hood hood, SUB_Metering metering,
                        SUB_Index index, SUB_Roller roller, SUB_Linear linear) {
        this.shooter = shooter;
        this.hood = hood;
        this.metering = metering;
        this.index = index;
        this.roller = roller;
        this.linear = linear;
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
        ).withName("RobotActions.deployAndIntake");
    }

    /**
     * Stops the intake rollers.
     *
     * @return Command to stop intake rollers.
     */
    public Command stopIntake() {
        return Commands.runOnce(() -> roller.stop(), roller)
            .withName("RobotActions.stopIntake");
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
        ).withName("RobotActions.checkAndFeed");
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
        ).withName("RobotActions.stopShooterIndexer");
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
        ).withName("RobotActions.reverseAll");
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
        ).withName("RobotActions.stopAll");
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
        ).withName("RobotActions.tuckAndClear");
    }
}
