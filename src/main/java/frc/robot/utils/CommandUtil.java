package frc.robot.utils;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.CMD_AimBotAuto;
import frc.robot.commands.CMD_PredictiveAimAuto;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Linear;
import frc.robot.subsystems.SUB_Roller;

public class CommandUtil {
    private CommandSwerveDrivetrain drivetrain;
    private SUB_Linear linear;
    private SUB_Roller roller;
    private SUB_Index index;
    private SUB_PhotonVision photonVision;
    private SUB_Shooter shooter;
    public CommandUtil (CommandSwerveDrivetrain drivetrain, SUB_Linear linear, SUB_Roller roller, SUB_Index index, SUB_PhotonVision photonVision, SUB_Shooter shooter){
        this.drivetrain = drivetrain;
        this.linear = linear;
        this.roller = roller;
        this.index = index;
        this.photonVision = photonVision;
        this.shooter = shooter;
    }
    public void registerAllNamedCommands(){
        NamedCommands.registerCommand("ReachedTarget", new InstantCommand(

                                () -> drivetrain.setReachedTarget(true)));

                NamedCommands.registerCommand("ResetReachedTarget",
                                new InstantCommand(() -> drivetrain.setReachedTarget(false)));

                
                // Intake

                NamedCommands.registerCommand("Intake",
                        new RunCommand(() -> {
                                linear.forward(Constants.Linear.kLINEAR_FAST_PID_CONTROLLER);
                                roller.setVolts(Constants.Roller.kROLLER_MOTOR_VOLTAGE);
                        }
                                
                        ,linear,roller)
                );


                NamedCommands.registerCommand("StopIntake",
                                Commands.parallel(
                                        new InstantCommand(() -> roller.set(0), roller)
                                        // TODO: Check?
                                        // Not needed?
                                        // new InstantCommand(() -> linear.set(0), linear)
                                )
                );

                // TODO: Check
                NamedCommands.registerCommand("DeployIntakeEncoder", new InstantCommand(() -> linear.forward(Constants.Linear.kLINEAR_FAST_PID_CONTROLLER), linear));

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

                NamedCommands.registerCommand("ShootAndMove", 
                        new CMD_PredictiveAimAuto(
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
                        getLinearCompress()
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
    }
        public Command getLinearCompress () {
                // TODO: check this
                return new RunCommand(() -> linear.backward(Constants.Linear.kLINEAR_SLOW_PID_CONTROLLER), linear);
        }
}
