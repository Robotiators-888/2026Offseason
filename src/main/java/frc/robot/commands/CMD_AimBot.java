// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Hub;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBot extends RunCommand {
  /** Subsystems and state variables used for targeting and control */
  private final CommandSwerveDrivetrain drivetrain;
  private Optional<Translation2d> targetTranslation = Optional.empty();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;
  private final SUB_Index index;
  private final SUB_Hood hood;
  private final SUB_Metering metering;
  private final SUB_Shooter shooter;
  private boolean isLocked;

  /** Physical offsets for targeting calibration */
  Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
  Rotation2d shooterThetaOffset = new Rotation2d(Units.degreesToRadians(0)); // CounterClockwise Positive
  
  /** Motion profiling constraints for rotation */
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(1.6).in(RadiansPerSecond), 
      RotationsPerSecond.of(12).in(RadiansPerSecond)   
  );

  /** PID controller for robot heading alignment */
  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      5.0, 0, 0.2, // P=5.0 is aggressive but safe with a Profile
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;

  /** Inside this error the heading output is zeroed so the robot can actually settle. */
  private static final double kHeadingToleranceRads = Units.degreesToRadians(0.75);

  /** Error band within which the shot is considered on-target. */
  private static final double kAlignedToleranceRads = Units.degreesToRadians(5);

  /** Yaw rate below which the robot is considered settled enough to fire, in degrees per second. */
  private static final double kMaxYawRateDegPerSec = 20;

  /** Constant term that breaks drivetrain stiction, applied only outside the tolerance band. */
  private static final double kStictionFeedforwardRadsPerSec = Units.degreesToRadians(9);

  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in X direction
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in Y direction
  
  /**
   * Constructs a new AimBot command.
   * @param drivetrain The swerve drivetrain subsystem
   * @param photonVision The vision subsystem for target tracking
   * @param shooter The shooter subsystem
   * @param index The indexer subsystem
   * @param translationXSupplier Supplier for X translation input
   * @param translationYSupplier Supplier for Y translation input
   */
  public CMD_AimBot(CommandSwerveDrivetrain drivetrain, SUB_Index index, SUB_Hood hood, SUB_Metering metering, SUB_Shooter shooter, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {    super(() -> {});
    this.drivetrain = drivetrain;
    this.index = index;
    this.hood = hood;
    this.metering = metering;
    this.shooter = shooter;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrain, metering, index, hood);
  }

  @Override
  public void initialize() {
    // Determine the correct target tag based on the current alliance
    robotAngleController.setTolerance(kHeadingToleranceRads);

    // Latch the hub location once, here, rather than recomputing it every execute().
    targetTranslation = Hub.getGoalTranslation();

    // Reset the PID controller to the current state of the robot
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    isLocked = false;
    running = true;
    isThetaErrorCorrect = false;
    xSlewRateLimiter.reset(0.0);
    ySlewRateLimiter.reset(0.0);
    shooter.setHighPower(true);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    // Without a hub location there is nothing to aim at. Hold still rather than aiming at a
    // fabricated target, which is what the old orElse(currentPose) fallback silently did.
    if (targetTranslation.isEmpty()) {
      isThetaErrorCorrect = false;
      index.setVolts(0);
      metering.set(0);
      drivetrain.setControl(brakeRequest);
      return;
    }
    Translation2d goal = targetTranslation.get();

    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // 2. Calculate the angle directly from the SHOOTER to the target
    Rotation2d targetRotation = new Rotation2d(
        goal.getX() - shooterFieldPosition.getX(),
        goal.getY() - shooterFieldPosition.getY()
    );

    targetRotation = targetRotation.plus(shooterThetaOffset);

    // Update telemetry
    drivetrain.publisher1.set(new Pose2d(goal, targetRotation));

    // 3. Calculate rotational velocity (omega) using the PID controller
    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Calculate error for deadband checking
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));

    isThetaErrorCorrect = thetaErrorRads <= kAlignedToleranceRads && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= kMaxYawRateDegPerSec;
    SmartDashboard.putBoolean("CMD_AimBot/isThetaErrorCorrect",isThetaErrorCorrect);

    double distance = shooterFieldPosition.getDistance(goal);

    // The flywheel holds a standing band (see SUB_Shooter.holdReadyBand), so the hood is what
    // actually aims. Solve the angle against the RPM currently commanded; fall back to the
    // minimum-energy angle if that RPM cannot reach, which is also the angle the band floor
    // corresponds to.
    hood.setAngle(SUB_Hood.angleForRPM(distance, shooter.getTargetRPM(), shooter.tunedRPM(distance))
        .orElseGet(() -> SUB_Hood.findoptimalangle(distance)));
    SmartDashboard.putNumber("CMD_AimBot/Distance (m)", distance);
    metering.set(1);

    // Only feed once the robot is pointed AND the flywheel is up to speed — feeding into a
    // spinning-up flywheel is what the auto and shuttle paths already guard against.
    if (isThetaErrorCorrect && shooter.atDesiredRPM()) {
      index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else {
      index.setVolts(0);
    }
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));

    // Wheel locking logic
    if (!isLocked && thetaErrorRads <= Units.degreesToRadians(2)) {
      isLocked = true;
    }
    else if (isLocked && thetaErrorRads >= kAlignedToleranceRads) {
      isLocked = false;
    }

    // Lock wheels or drive
    if (xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked) {
        drivetrain.setControl(brakeRequest);
    } else {
        drivetrain.setControl(
          drive.withVelocityX(xInput * MaxSpeed)
          .withVelocityY(yInput * MaxSpeed)
          .withRotationalRate(rotationalRate(omegaSpeed, thetaErrorRads)));
    }
  }

  /**
   * Turns the heading controller's output into a rotational rate command.
   *
   * <p>The controller already produces radians per second, so it is clamped rather than scaled by
   * MaxAngularRate — multiplying by it, as this used to, inflated the command by ~4.7x on top of
   * the proportional gain.
   *
   * <p>The static term breaks stiction, but it is gated on the error being outside the alignment
   * band. Applying it unconditionally meant that at zero error {@code Math.copySign(x, 0.0)}
   * returned +x and the robot kept creeping, so it could never settle.
   */
  private double rotationalRate(final double omegaSpeed, final double thetaErrorRads) {
    final double clamped = MathUtil.clamp(omegaSpeed, -MaxAngularRate, MaxAngularRate);
    if (thetaErrorRads <= kHeadingToleranceRads) {
      return 0.0;
    }
    return clamped + Math.copySign(kStictionFeedforwardRadsPerSec, clamped);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    isThetaErrorCorrect = false;
    shooter.setHighPower(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning () {
    return running;
  }
}
