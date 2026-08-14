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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.TrajectorySolver;

public class CMD_Shuttle extends Command {
    /** Physical offsets for targeting calibration */
    private final Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    
    /** Subsystems and state variables for shuttle targeting */
    private final SUB_Index index;
    private final SUB_Shooter shooter;
    private final SUB_PhotonVision photonVision;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    
    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withRotationalDeadband(0) 
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withCenterOfRotation(shooterOffset);

    /** Motion profiling constraints for rotation */
    private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
        RotationsPerSecond.of(0.75).in(RadiansPerSecond), 
        RotationsPerSecond.of(1.5).in(RadiansPerSecond)   
    );
    private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0, -8.0, 0.0);
    private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0, -8.0, 0.0);

    /** PID controller for robot heading alignment during shuttle */
    private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
        5.0, 0, 0.2,
        thetaConstraints
    );
    private Pose2d targetPose = new Pose2d();
    private boolean isThetaErrorCorrect;

    /**
     * Constructs a new CMD_Shuttle command for long-range scoring or passing.
     */
    public CMD_Shuttle(
        CommandSwerveDrivetrain drivetrain, 
        SUB_PhotonVision photonVision, 
        SUB_Index index, 
        SUB_Shooter shooter,
        DoubleSupplier translationXSupplier, 
        DoubleSupplier translationYSupplier
    ) {
        this.index = index;
        this.shooter = shooter;
        this.photonVision = photonVision;
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
        isThetaErrorCorrect = false;
        
        addRequirements(drivetrain, index, shooter);
    }

    @Override
    public void initialize() {
        robotAngleController.setTolerance(Units.degreesToRadians(0.0));
        robotAngleController.reset(
            drivetrain.getPose().getRotation().getRadians(),
            drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
        );
        SUB_Shooter.isShooting = true;
    }

    @Override
    public void execute() {
        // Calculate shuttle target with field offset
        int tagId = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26;
        Pose2d tagPose = photonVision.at_field.getTagPose(tagId).orElse(new Pose3d()).toPose2d();

        double offsetX = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) 
            ? -Units.inchesToMeters(100) 
            : Units.inchesToMeters(100);
        double offsetY = (drivetrain.getPose().getY() < 4.0) 
            ? Units.inchesToMeters(100) 
            : -Units.inchesToMeters(100);

        Translation2d shuttleTarget = tagPose.getTranslation().plus(new Translation2d(offsetX, offsetY));
        targetPose = new Pose2d(shuttleTarget, new Rotation2d());
        
        Pose2d currentPose = drivetrain.getPose();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getCurrentRobotChassisSpeeds(), 
            currentPose.getRotation()
        );

        Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
            shooterOffset.rotateBy(currentPose.getRotation())
        );

        double distanceToTarget = shooterFieldPosition.getDistance(targetPose.getTranslation());
        double tof = shooter.getExpectedTOF(distanceToTarget);

        Translation2d virtualTargetTranslation = new Translation2d(
            targetPose.getX() - (fieldSpeeds.vxMetersPerSecond * tof),
            targetPose.getY() - (fieldSpeeds.vyMetersPerSecond * tof)
        );

        drivetrain.publisher2.set(new Pose2d(virtualTargetTranslation, new Rotation2d()));
        
        // Calculate required heading to face shuttle target
        Rotation2d targetRotation = new Rotation2d(
            virtualTargetTranslation.getX() - shooterFieldPosition.getX(),
            virtualTargetTranslation.getY() - shooterFieldPosition.getY()
        );

        double omegaSpeed = robotAngleController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );

        // Check if rotation error is within acceptable threshold
        double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
        isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(14) 
            && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 40;
        
        double distance = shooterFieldPosition.getDistance(virtualTargetTranslation);
        TrajectorySolver.TrajectoryResult trajectory = TrajectorySolver.calculateTrajectory(
            shooter.flywheelRPM() > 500 ? shooter.flywheelRPM() : Constants.Shooter.kSHOOTER_FLYWHEEL_RPM,
            distance
        );
        if (trajectory.flywheelAdjusted() || shooter.flywheelRPM() < 500) {
            shooter.setRPM(trajectory.targetFlywheelRPM());
        }
        
        boolean isShooterReady = shooter.atDesiredRPM();
        if (isThetaErrorCorrect && isShooterReady) {
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
        } else {
            index.setVolts(0);
        }

        double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
        double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
        
        double clampedOmega = MathUtil.clamp(omegaSpeed, -MaxAngularRate, MaxAngularRate);
        double feedforward = Math.copySign(Units.degreesToRadians(9), clampedOmega);
        drivetrain.setControl(
            drive.withVelocityX(xInput * MaxSpeed)
                 .withVelocityY(yInput * MaxSpeed)
                 .withRotationalRate(clampedOmega + (Math.abs(clampedOmega) > 1e-4 ? feedforward : 0))
        );
    }

    @Override
    public void end(boolean interrupted) {
        SUB_Shooter.isShooting = false;
        index.setVolts(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
