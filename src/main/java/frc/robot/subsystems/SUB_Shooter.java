package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.SimulatedArena;
import org.dyn4j.geometry.Circle;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.*;

public class SUB_Shooter extends SubsystemBase {
    private final SUB_Intake intake;
    private final SUB_Drivetrain drivetrain;
    
    // Define "Fuel" properties
    public static final GamePieceInfo FUEL_INFO = new GamePieceInfo(
        "Fuel",
        new Circle(0.075), // Radius in meters
        Meters.of(0.075),
        Kilograms.of(0.25),
        0.1, // Linear damping
        0.1, // Angular damping
        0.7 // Restitution
    );

    public SUB_Shooter(SUB_Intake intake, SUB_Drivetrain drivetrain) {
        this.intake = intake;
        this.drivetrain = drivetrain;
    }

    public void shoot() {
        if (Robot.isSimulation()) {
            if (intake.takeBall()) {
                // Calculate launch parameters
                var robotPose = drivetrain.getPose();
                var robotSpeeds = drivetrain.getChassisSpeeds();
                
                // Launch position: 0.5m in front of robot center
                Translation2d launchPos = robotPose.getTranslation().plus(
                    new Translation2d(0.5, robotPose.getRotation())
                );
                
                // Launch velocity: 2.5 m/s forward + robot velocity
                double shotSpeed = 2.5;
                Translation2d shotVel = new Translation2d(shotSpeed, robotPose.getRotation())
                    .plus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond));

                GamePieceProjectile projectile = new GamePieceProjectile(
                    FUEL_INFO,
                    launchPos,
                    shotVel,
                    0.5, // Initial height (meters)
                    6.0, // Initial vertical speed (m/s)
                    new Rotation3d()
                );
                
                SimulatedArena.getInstance().addGamePieceProjectile(projectile);
            }
        }
    }
}
