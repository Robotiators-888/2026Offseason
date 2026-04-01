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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Advanced targeting command that compensates for robot movement using time-of-flight prediction.
 * Instead of reacting to error, it calculates a "virtual target" based on where the hub 
 * will be relative to the robot when the ball arrives.
 */
public class CMD_PredictiveAim extends RunCommand {
  /** Subsystems and state variables for predictive targeting */
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d staticTargetPose = new Pose2d();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;
  private final SUB_Shooter shooter;
  private final SUB_Index index;

  /** Physical offsets for targeting calibration */
  Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-5));
  
  /** Motion profiling constraints for rotation */
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(1.6).in(RadiansPerSecond), 
      RotationsPerSecond.of(5).in(RadiansPerSecond)   
  );

  /** 
   * PID controller for robot heading alignment.
   * In predictive mode, this handles fine stability while feed-forward handles the bulk of the "lead".
   */
  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      1.5, 0, 0.0, 
      thetaConstraints
  );
  
  public static boolean isThetaErrorCorrect = false;
  private double MaxSpeed = 2.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.10; // Limit max speed to 30% for better control while aiming
  private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withCenterOfRotation(shooterOffset); 
  
  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0, -3.0, 0.0);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0, -3.0, 0.0);
  
  /**
   * Constructs a new PredictiveAim command.
   * @param drivetrain The swerve drivetrain subsystem
   * @param photonVision The vision subsystem
   * @param shooter The shooter subsystem
   * @param index The indexer subsystem
   * @param translationXSupplier Supplier for X translation input
   * @param translationYSupplier Supplier for Y translation input
   */
  public CMD_PredictiveAim(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {    
    super(() -> {});
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain, shooter, index);
  }

  @Override
  public void initialize() {
    robotAngleController.setTolerance(Units.degreesToRadians(0.0));
    
    Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
      
    double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
    Translation2d hubCenterTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    
    staticTargetPose = new Pose2d(hubCenterTranslation, new Rotation2d());
    
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    running = true;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    
    // Convert robot-centric speeds to field-centric for accurate target shifting
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getCurrentRobotChassisSpeeds(), 
        currentPose.getRotation()
    );

    // 1. Calculate base distance and initial Time of Flight (TOF)
    double distanceToHub = currentPose.getTranslation().getDistance(staticTargetPose.getTranslation());
    double tof = shooter.getExpectedTOF(distanceToHub);

    // 2. Calculate Virtual Target: PhysicalTarget - (RobotVelocity * TOF)
    // Moving towards the goal (positive velocity) makes the virtual target closer.
    Translation2d virtualTargetTranslation = new Translation2d(
        staticTargetPose.getX() - (fieldSpeeds.vxMetersPerSecond * tof),
        staticTargetPose.getY() - (fieldSpeeds.vyMetersPerSecond * tof)
    );

    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // 3. Calculate target rotation toward the virtual target
    Rotation2d targetRotation = new Rotation2d(
        virtualTargetTranslation.getX() - shooterFieldPosition.getX(),
        virtualTargetTranslation.getY() - shooterFieldPosition.getY()
    );

    // Telemetry: Show both physical and virtual targets
    drivetrain.publisher1.set(new Pose2d(staticTargetPose.getTranslation(), targetRotation)); // Physical
    drivetrain.publisher2.set(new Pose2d(virtualTargetTranslation, targetRotation)); // Virtual

    // 4. Calculate rotation velocity
    // Primary rotation comes from PID toward the predictive target
    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Alignment checking for firing (Removed angular velocity check)
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("PredictiveAim/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(5);
    boolean isReadyToShoot = isThetaErrorCorrect; //if we want to add shooter RPM requirements, we can add it here as well (shooter.atDesiredRPM())
    
    SmartDashboard.putBoolean("PredictiveAim/isThetaErrorCorrect", isThetaErrorCorrect);
    SmartDashboard.putBoolean("PredictiveAim/ReadyToShoot", isReadyToShoot);

    // 5. Update shooter RPM based on predicted virtual distance
    double predictiveDistance = shooterFieldPosition.getDistance(virtualTargetTranslation);
    shooter.shootMeters(predictiveDistance);
    
    // Automatic indexing when ready
    if (isReadyToShoot) {
      index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
      index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else {
      index.setVolts(0);
      index.setMeteringRPM(-1000);
    }

    // double xInput = 0.0;//xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double magnitude = Math.sqrt(Math.pow(xInput,2)+Math.pow(yInput,2));
    if (magnitude == 0.0) {
      magnitude = 1.0; //No divivde by zero, and if we're not commanding movement, we don't need to limit it
    }
    
    
    // We add a small constant "kick" to overcome friction when moving proactively
    double rotationOutput = omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed);
    
    drivetrain.setControl(
      drive.withVelocityX(xInput * MaxSpeed * (1/magnitude))
      .withVelocityY(yInput * MaxSpeed * (1/magnitude))
      .withRotationalRate(rotationOutput));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning () {
    return running;
  }
}
