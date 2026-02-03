package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.*;

public class SUB_Intake extends SubsystemBase {
    private IntakeSimulation intakeSim;
    private final SUB_Drivetrain drivetrain;
    private final int MAX_CAPACITY = 30;

    public SUB_Intake(SUB_Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        if (Robot.isSimulation()) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                drivetrain.getDriveSimulation(),
                Inches.of(25), 
                Inches.of(6), 
                IntakeSide.BACK,
                MAX_CAPACITY
            );
            intakeSim.register();
        }
    }

    public void runIntake() {
        if (intakeSim != null) {
            intakeSim.startIntake();
        }
    }

    public void stopIntake() {
        if (intakeSim != null) {
            intakeSim.stopIntake();
        }
    }
    
    public int getStoredFuelCount() {
        if (intakeSim != null) {
            return intakeSim.getGamePiecesAmount();
        }
        return 0;
    }
    
    public boolean takeBall() {
        if (intakeSim != null) {
            return intakeSim.obtainGamePieceFromIntake();
        }
        return false;
    }
}
