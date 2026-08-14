// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.TrajectorySolver;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBot extends Command {
  /** Subsystems and state variables used for targeting and control */
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose = new Pose2d();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;
  private final SUB_Index index;
  private final SUB_Hood hood;
  private final SUB_Metering metering;
  private boolean isLocked;

  /** Physical offsets for targeting calibration */
  private final Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
  private final Rotation2d shooterThetaOffset = new Rotation2d(Units.degreesToRadians(0)); // CounterClockwise Positive
  
  /** Motion profiling constraints for rotation */
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(1.6).in(RadiansPerSecond), 
      RotationsPerSecond.of(12).in(RadiansPerSecond)   
  );

  /** PID controller for robot heading alignment */
  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      5.0, 0, 0.2,
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final double MaxSpeed = 2.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0, -8.0, 0.0);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0, -8.0, 0.0);
  
  /**
   * Constructs a new AimBot command.
   */
  public CMD_AimBot(
      CommandSwerveDrivetrain drivetrain, 
      SUB_PhotonVision photonVision, 
      SUB_Index index, 
      SUB_Hood hood, 
      SUB_Metering metering, 
      DoubleSupplier translationXSupplier, 
      DoubleSupplier translationYSupplier
  ) {
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.index = index;
    this.hood = hood;
    this.metering = metering;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrain, metering, index, hood);
  }

  @Override
  public void initialize() {
    robotAngleController.setTolerance(Units.degreesToRadians(0.0));
    
    int targetTag = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26;
    Pose2d tagPose = photonVision.at_field.getTagPose(targetTag).orElse(new Pose3d()).toPose2d();
      
    double hubOffsetX = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) 
        ? Units.inchesToMeters(-23.5) 
        : Units.inchesToMeters(23.5);
    Translation2d hubCenterTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    
    targetPose = new Pose2d(hubCenterTranslation, new Rotation2d());
    
    // Reset the PID controller to current state
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    isLocked = false;
    running = true;
    SUB_Shooter.isShooting = true;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    Translation2d targetTranslation = targetPose.getTranslation();
    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // Calculate heading from shooter to target
    Rotation2d targetRotation = new Rotation2d(
        targetTranslation.getX() - shooterFieldPosition.getX(),
        targetTranslation.getY() - shooterFieldPosition.getY()
    ).plus(shooterThetaOffset);

    // Update telemetry
    drivetrain.publisher1.set(new Pose2d(targetTranslation, targetRotation));

    // Calculate rotational velocity (omega) using the profiled PID controller
    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Calculate error for threshold checking
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(5) 
        && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 20;
    SmartDashboard.putBoolean("CMD_AimBot/isThetaErrorCorrect", isThetaErrorCorrect);
    
    double distance = currentPose.getTranslation().getDistance(targetTranslation);
    TrajectorySolver.TrajectoryResult trajectory = TrajectorySolver.calculateTrajectory(
        SUB_Shooter.getInstance().flywheelRPM() > 500 ? SUB_Shooter.getInstance().flywheelRPM() : Constants.Shooter.kSHOOTER_FLYWHEEL_RPM,
        distance
    );
    hood.setToPosition(trajectory.desiredHoodAngleRad());
    if (trajectory.flywheelAdjusted() || SUB_Shooter.getInstance().flywheelRPM() < 500) {
        SUB_Shooter.getInstance().setRPM(trajectory.targetFlywheelRPM());
    }
    metering.set(1.0);
    
    if (isThetaErrorCorrect) {
      index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else {
      index.setVolts(0);
    }
    
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));

    // Wheel locking logic
    if (!isLocked && thetaErrorRads <= Units.degreesToRadians(2)) {
      isLocked = true;
    } else if (isLocked && thetaErrorRads >= Units.degreesToRadians(5)) {
      isLocked = false;
    }

    // Lock wheels or drive
    if (xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked) {
        drivetrain.setControl(brakeRequest);
    } else {
        double clampedOmega = MathUtil.clamp(omegaSpeed, -MaxAngularRate, MaxAngularRate);
        double feedforward = Math.copySign(Units.degreesToRadians(9), clampedOmega);
        drivetrain.setControl(
          drive.withVelocityX(xInput * MaxSpeed)
               .withVelocityY(yInput * MaxSpeed)
               .withRotationalRate(clampedOmega + (Math.abs(clampedOmega) > 1e-4 ? feedforward : 0))
        );
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    SUB_Shooter.isShooting = false;
    index.setVolts(0);
    metering.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning() {
    return running;
  }
}
