package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

/**
 * Characterization command to calculate effective swerve wheel radius by rotating the robot in place.
 * Accounts for wheel tread wear over competitions.
 */
public class WheelRadiusCharacterizationCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private double accumWheelDistance = 0.0;
    private double gyroDelta = 0.0;

    private Rotation2d lastAngle = new Rotation2d();
    private final double[] lastPositions = new double[4];
    private final Timer timer = new Timer();

    private final SwerveRequest.RobotCentric request =
        new SwerveRequest.RobotCentric()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(1.5);

    // Drive base radius: distance from center of robot to each swerve module (sqrt(11.25^2 + 11.25^2) in meters)
    private static final double DRIVE_BASE_RADIUS_METERS = Units.inchesToMeters(11.25 * Math.sqrt(2.0));
    private static final double NOMINAL_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    /**
     * Constructs a new WheelRadiusCharacterizationCommand.
     *
     * @param drivetrain Swerve drivetrain subsystem.
     */
    public WheelRadiusCharacterizationCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
        accumWheelDistance = 0.0;
        gyroDelta = 0.0;
        lastAngle = drivetrain.getPose().getRotation();

        SwerveModulePosition[] currentPositions = drivetrain.getState().ModulePositions;
        for (int i = 0; i < 4 && i < currentPositions.length; i++) {
            lastPositions[i] = currentPositions[i].distanceMeters;
        }
    }

    @Override
    public void execute() {
        drivetrain.setControl(request);

        SwerveModulePosition[] currentPositions = drivetrain.getState().ModulePositions;
        Rotation2d currentAngle = drivetrain.getPose().getRotation();

        // Allow 1 second for wheels to align before accumulating distance
        if (timer.hasElapsed(1.0)) {
            for (int i = 0; i < 4 && i < currentPositions.length; i++) {
                accumWheelDistance += Math.abs(currentPositions[i].distanceMeters - lastPositions[i]);
            }
            gyroDelta += Math.abs(currentAngle.minus(lastAngle).getRadians());
        }

        for (int i = 0; i < 4 && i < currentPositions.length; i++) {
            lastPositions[i] = currentPositions[i].distanceMeters;
        }
        lastAngle = currentAngle;
    }

    @Override
    public boolean isFinished() {
        // Run for 3 full rotations (6 * PI radians)
        return gyroDelta >= Math.PI * 6;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());

        if (accumWheelDistance > 0.001) {
            double averageWheelDistance = accumWheelDistance / 4.0;
            double effectiveRadius = NOMINAL_WHEEL_RADIUS_METERS * (gyroDelta * DRIVE_BASE_RADIUS_METERS) / averageWheelDistance;
            double effectiveRadiusInches = Units.metersToInches(effectiveRadius);

            Logger.recordOutput("WheelRadius/WheelDeltaMeters", averageWheelDistance);
            Logger.recordOutput("WheelRadius/GyroDeltaRadians", gyroDelta);
            Logger.recordOutput("WheelRadius/EffectiveRadiusMeters", effectiveRadius);
            Logger.recordOutput("WheelRadius/EffectiveRadiusInches", effectiveRadiusInches);

            SmartDashboard.putNumber("WheelRadius/EffectiveRadiusInches", effectiveRadiusInches);
            System.out.printf("[WheelRadius] Characterized effective wheel radius: %.4f inches (%.4f m)%n",
                effectiveRadiusInches, effectiveRadius);
        }
    }
}
