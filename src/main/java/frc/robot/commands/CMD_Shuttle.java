package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Shuttle extends RunCommand{
    private SUB_Index index;
    private SUB_Shooter shooter;
    private SUB_PhotonVision photonVision;
    private CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withRotationalDeadband(0) 
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
        RotationsPerSecond.of(0.75).in(RadiansPerSecond), 
        RotationsPerSecond.of(1.5).in(RadiansPerSecond)   
    );

    private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
        5.0, 0, 0.2, // P=5.0 is aggressive but safe with a Profile
        thetaConstraints
    );    private Pose2d targetPose = new Pose2d();
    private boolean isThetaErrorCorrect;
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
        robotAngleController.reset(
            drivetrain.getPose().getRotation().getRadians(),
            drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
        );
    }

    @Override
    public void execute () {
        targetPose = photonVision.at_field.getTagPose(
                    DriverStation.getAlliance().orElse(
                        Alliance.Blue
                    ) == Alliance.Red ? 10 : 26
            ).map(
                pose -> pose.toPose2d().relativeTo(
                    new Pose2d(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -Units.inchesToMeters(100) : Units.inchesToMeters(100),(drivetrain.getPose().getY()<4) ? Units.inchesToMeters(100): -Units.inchesToMeters(100),Rotation2d.fromDegrees(0)
                        )
                    )).orElse(
                        drivetrain.getPose()
                    );
        drivetrain.publisher2.set(targetPose);
        SmartDashboard.putNumber("Y Test",drivetrain.getPose().getY());
        Pose2d currentPose = drivetrain.getPose();
        // Set targetPose to be correct
        Translation2d targetTranslation = targetPose.getTranslation();
        Rotation2d targetRotation = new Rotation2d(
            targetTranslation.getX() - currentPose.getX(),
            targetTranslation.getY() - currentPose.getY()
        );
        double omegaSpeed = robotAngleController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );
        double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
        isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(14) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 40;
        double distance = drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation());
        shooter.shootMeters(distance);
        index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM); // Keep metering wheel spinning
        boolean isShooterReady = shooter.atDesiredRPM();
        boolean isMeteringReady = Math.abs(index.intakeMeteringRPM() - Constants.Index.kINDEX_METERING_MOTOR_RPM) < 100;
        if (isThetaErrorCorrect && isShooterReady && isMeteringReady) {
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
        }
        double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband);
        double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband);
        drivetrain.setControl(
        drive.withVelocityX(xInput * MaxSpeed)
        .withVelocityY(yInput * MaxSpeed)
        .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate)));
 
    }
}
