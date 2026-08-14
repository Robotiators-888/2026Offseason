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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.utils.AimUtil;
import frc.robot.utils.FeedGate;
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
  
  /**
   * Motion profiling constraints for rotation. The profile velocity must not exceed the
   * MaxAngularRate output clamp below — a profile planning 1.6 rot/s against a 0.75 rot/s clamp
   * ran its setpoint away from the plant and lagged every aim.
   */
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(0.75).in(RadiansPerSecond),
      RotationsPerSecond.of(1.5).in(RadiansPerSecond)
  );

  /** PID controller for robot heading alignment */
  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      Constants.Shooter.kAIM_HEADING_kP, 0, Constants.Shooter.kAIM_HEADING_kD,
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;

  /** Inside this error the heading output is zeroed so the robot can actually settle. */
  private static final double kHeadingToleranceRads = Constants.Shooter.kAIM_HEADING_TOLERANCE_RADS;

  /** Yaw rate below which the robot is considered settled enough to fire, in degrees per second. */
  private static final double kMaxYawRateDegPerSec = 20;

  /** Debounced, latching fire gate — see {@link FeedGate}. */
  private final FeedGate feedGate =
      new FeedGate(Constants.Shooter.kFEED_ARM_DEBOUNCE_SECONDS, Timer::getFPGATimestamp);

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

    // The shooter is required because this command commands its RPM every loop. Leaving it to
    // the shooter's default command meant a manual-RPM binding could interrupt that default
    // mid-aim, and the hood silently solved against a setpoint nobody was holding. The flip
    // side is deliberate: pressing manual RPM now cancels this command outright.
    addRequirements(drivetrain, metering, index, hood, shooter);
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
    feedGate.reset();
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
    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // Motion compensation, ported from CMD_Shuttle: aim at a virtual target shifted opposite
    // the robot's field-relative velocity by the shot's time of flight, so shooting while
    // strafing leads the hub instead of trailing it. At standstill this degrades to the hub.
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getCurrentRobotChassisSpeeds(), currentPose.getRotation());
    Translation2d goal = AimUtil.virtualTarget(targetTranslation.get(), shooterFieldPosition,
        fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond,
        shooter::getExpectedTOF);

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

    double distance = shooterFieldPosition.getDistance(goal);

    // Tolerance scales with range — a flat band lets the lateral miss grow with distance.
    final double alignedToleranceRads = AimUtil.alignedToleranceRads(distance,
        Constants.Shooter.kAIM_HALF_WIDTH_METERS, Constants.Shooter.kAIM_SAFETY_FACTOR_TELEOP);

    isThetaErrorCorrect = thetaErrorRads <= alignedToleranceRads && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= kMaxYawRateDegPerSec;
    SmartDashboard.putBoolean("CMD_AimBot/isThetaErrorCorrect",isThetaErrorCorrect);

    // The flywheel holds a standing band (see SUB_Shooter.holdReadyBand), so the hood is what
    // actually aims. This command owns the RPM now — leaning on the shooter's default command
    // also measured distance from robot center while the hood used the shooter position, so the
    // two could solve for different ranges. Solve the hood against the RPM just commanded; fall
    // back to the minimum-energy angle if that RPM cannot reach, which is also the angle the
    // band floor corresponds to.
    shooter.holdReadyBand(distance);
    hood.setAngle(SUB_Hood.angleForRPM(distance, shooter.getTargetRPM(), shooter.tunedRPM(distance))
        .orElseGet(() -> SUB_Hood.findoptimalangle(distance)));
    SmartDashboard.putNumber("CMD_AimBot/Distance (m)", distance);
    metering.set(1);

    final boolean inRange = shooter.canReach(distance);
    SmartDashboard.putBoolean("Shooter/In Range", inRange);
    // Advisory only — during shifts the hub may be inactive, but transition/endgame rules and
    // driver judgment make a hard firing interlock riskier than the wasted shot it prevents.
    SmartDashboard.putBoolean("CMD_AimBot/Hub Inactive",
        Hub.isAllianceHubActive().map(active -> !active).orElse(false));

    // Feed only once every arm condition — pointed, at speed, hood arrived, physically in
    // range — has held for a beat, then keep feeding through the RPM droop of a ball passing
    // the wheels (see FeedGate). The hood check matters most: the hood is the aiming axis, and
    // this gate used to fire while it was still slewing.
    final boolean armReady =
        isThetaErrorCorrect && shooter.atDesiredRPM() && hood.atAngle() && inRange;
    final boolean sustainReady = isThetaErrorCorrect
        && Math.abs(shooter.rpmError()) < Constants.Shooter.kRPM_SUSTAIN_TOLERANCE;
    if (feedGate.update(armReady, sustainReady)) {
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
    else if (isLocked && thetaErrorRads >= alignedToleranceRads) {
      isLocked = false;
    }

    // Lock wheels or drive
    if (xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked) {
        drivetrain.setControl(brakeRequest);
    } else {
        drivetrain.setControl(
          drive.withVelocityX(xInput * MaxSpeed)
          .withVelocityY(yInput * MaxSpeed)
          .withRotationalRate(AimUtil.rotationalRate(omegaSpeed, thetaErrorRads,
              MaxAngularRate, kHeadingToleranceRads,
              Constants.Shooter.kAIM_STICTION_RADS_PER_SEC,
              Constants.Shooter.kAIM_STICTION_RAMP_WIDTH_RADS)));
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    isThetaErrorCorrect = false;
    // Zero the feed path explicitly rather than trusting the default commands to do it a loop
    // later — the auto and shuttle commands already do this in their end().
    index.setVolts(0);
    metering.set(0);
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
