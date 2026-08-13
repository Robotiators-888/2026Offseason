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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shuttle extends RunCommand{
    /** Physical offsets for targeting calibration */
    Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    /** Subsystems and state variables for shuttle targeting */
    private SUB_Index index;
    private SUB_Shooter shooter;
    private SUB_PhotonVision photonVision;
    private CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withRotationalDeadband(0) 
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withCenterOfRotation(shooterOffset);

    /** Motion profiling constraints for rotation */
    private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
        RotationsPerSecond.of(0.75).in(RadiansPerSecond), 
        RotationsPerSecond.of(1.5).in(RadiansPerSecond)   
    );
    private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0);
    private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0);

    /** PID controller for robot heading alignment during shuttle */
    private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
        5.0, 0, 0.2,
        thetaConstraints
    );
    private Pose2d targetPose = new Pose2d();
    private boolean isThetaErrorCorrect;

    /**
     * Constructs a new CMD_Shuttle command for long-range scoring or passing.
     * @param drivetrain The swerve drivetrain subsystem
     * @param photonVision The vision subsystem
     * @param index The indexer subsystem
     * @param shooter The shooter subsystem
     * @param translationXSupplier Supplier for X translation input
     * @param translationYSupplier Supplier for Y translation input
     */
    public CMD_Shuttle (CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Index index, SUB_Shooter shooter,DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        super(()->{});
        this.index = index;
        this.shooter = shooter;
        this.photonVision = photonVision;
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
        isThetaErrorCorrect = false;
        addRequirements(photonVision, drivetrain, index, shooter);
    }

    @Override
    public void initialize () {
        robotAngleController.setTolerance(Units.degreesToRadians(0.0));
        // Reset the PID controller to the current state of the robot
        robotAngleController.reset(
            drivetrain.getPose().getRotation().getRadians(),
            drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
        );
    }

    @Override
    public void execute () {
        // Calculate a target pose shifted away from the hub for shuttling/passing
        targetPose = photonVision.at_field.getTagPose(
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
        
        Translation2d targetTranslation = virtualTargetTranslation;
        
        // Calculate the required heading to face the shuttle target
        Rotation2d targetRotation = new Rotation2d(
            targetTranslation.getX() - shooterFieldPosition.getX(),
            targetTranslation.getY() - shooterFieldPosition.getY()
        );

        double omegaSpeed = robotAngleController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );

        // Check if the rotation error is within an acceptable threshold for firing
        double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
        isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(14) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 40;
        
        double distance = shooterFieldPosition.getDistance(virtualTargetTranslation);
        shooter.shootMeters(distance);
        
        
        // Automated firing when aligned and up to speed
        boolean isShooterReady = shooter.atDesiredRPM();
        
        if (isThetaErrorCorrect && isShooterReady) {
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
        } else if (!isThetaErrorCorrect) {
            index.setVolts(0);
        }

        double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
        double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
        
        // Apply swerve drive request with PID-calculated rotation
        drivetrain.setControl(
        drive.withVelocityX(xInput * MaxSpeed)
        .withVelocityY(yInput * MaxSpeed)
        .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate)));
 
    }
}
