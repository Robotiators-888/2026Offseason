package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.*;

public class SUB_Intake extends SubsystemBase {
    private IntakeSimulation intakeSim;
    private final SUB_Drivetrain drivetrain;
    private final int MAX_CAPACITY = 40;
    private int artificialFuelCount = 0;

    public SUB_Intake(SUB_Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        if (Robot.isSimulation()) {
            intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                drivetrain.getDriveSimulation(),
                Inches.of(25), 
                Inches.of(6), 
                IntakeSide.FRONT,
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
            return intakeSim.getGamePiecesAmount() + artificialFuelCount;
        }
        return artificialFuelCount;
    }
    
    public void addFuel(int amount) {
        artificialFuelCount += amount;
        if (artificialFuelCount > MAX_CAPACITY) {
            artificialFuelCount = MAX_CAPACITY;
        }
    }
    
    public void setFuel(int amount) {
        if (intakeSim != null) {
            // Clear physics simulation count
            while (intakeSim.getGamePiecesAmount() > 0) {
                intakeSim.obtainGamePieceFromIntake();
            }
        }
        artificialFuelCount = amount;
        if (artificialFuelCount > MAX_CAPACITY) {
            artificialFuelCount = MAX_CAPACITY;
        }
    }

    public boolean takeBall() {
        if (artificialFuelCount > 0) {
            artificialFuelCount--;
            return true;
        }
        if (intakeSim != null) {
            return intakeSim.obtainGamePieceFromIntake();
        }
        return false;
    }
}
