package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.commands.CMD_AimBotBase;

public class CMD_Shuttle extends CMD_AimBotBase {
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0);
    private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0);

    /**
     * Constructs a new CMD_Shuttle command for long-range scoring or passing.
     * @param drivetrain The swerve drivetrain subsystem
     * @param photonVision The vision subsystem
     * @param index The indexer subsystem
     * @param shooter The shooter subsystem
     * @param translationXSupplier Supplier for X translation input
     * @param translationYSupplier Supplier for Y translation input
     */
    public CMD_Shuttle (CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Index index, SUB_Shooter shooter, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        super(drivetrain, photonVision, index, shooter);
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
    }

    @Override
    protected Translation2d getTargetTranslation (Pose2d currentPose) {
        // Calculate a target pose shifted away from the hub for shuttling/passing
        Pose2d tempPose = photonVision.at_field.getTagPose(
                    DriverStation.getAlliance().orElse(
                        Alliance.Blue
                    ) == Alliance.Red ? 10 : 26
            ).map(
                pose -> pose.toPose2d().relativeTo(
                    new Pose2d(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -Units.inchesToMeters(100) : Units.inchesToMeters(100),
                        (drivetrain.getPose().getY()<4) ? Units.inchesToMeters(100): -Units.inchesToMeters(100),
                        Rotation2d.fromDegrees(0)
                    )
                )).orElse(
                    drivetrain.getPose()
                );
        
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getCurrentRobotChassisSpeeds(), 
            currentPose.getRotation()
        );
        
        // 1. Calculate base distance and initial Time of Flight (TOF)
        double distanceToHub = currentPose.getTranslation().getDistance(tempPose.getTranslation());
        double tof = shooter.getExpectedTOF(distanceToHub);

        // 2. Calculate Virtual Target: PhysicalTarget - (RobotVelocity * TOF)
        // Moving towards the goal (positive velocity) makes the virtual target closer.
        return new Translation2d(
            tempPose.getX() - (fieldSpeeds.vxMetersPerSecond * tof),
            tempPose.getY() - (fieldSpeeds.vyMetersPerSecond * tof)
        );
    }

    @Override
    protected double getDistanceFromTarget (Translation2d shooterFieldPosition, Translation2d targetTranslation) {
      return shooterFieldPosition.getDistance(targetTranslation);
    }
    
    @Override
    protected SwerveRequest getDriveRequest (double omegaSpeed) {
        double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
        double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));

        // Return swerve drive request with PID-calculated rotation
        return drive
            .withVelocityX(xInput * MaxSpeed)
            .withVelocityY(yInput * MaxSpeed)
            .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate));
 
    }

    @Override
    protected boolean getBrakeRequestConditions () {
        return false;
    }

    @Override
    protected void doBrakeLogic (Pose2d currentPose, Translation2d targetTranslation) {}
}
