package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shuttle extends RunCommand{
    SUB_Index index;
    SUB_Shooter shooter;
    SUB_Intake intake;
    SUB_PhotonVision photonVision;
    CommandSwerveDrivetrain drivetrain;
    public CMD_Shuttle (CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Index index, SUB_Shooter shooter, SUB_Intake intake) {
        super(()->{});
        this.index = index;
        this.shooter = shooter;
        this.intake = intake;
        this.photonVision = photonVision;
        this.drivetrain = drivetrain;
        addRequirements(photonVision, drivetrain, index, shooter, intake);
    }

    @Override
    public void initialize () {

    }

    @Override
    public void execute () {

    }
}
