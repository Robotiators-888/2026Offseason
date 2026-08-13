// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Hub;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBotAuto extends RunCommand {
  /** Subsystems and state variables for autonomous targeting */
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Optional<Translation2d> targetTranslation = Optional.empty();
  private static boolean running;
  private final SUB_Shooter shooter;
  private final SUB_Index index;
  private final SUB_Hood hood;
  private final SUB_Metering metering;

  /** Physical offset from robot center to shooter exit */
  Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
  
  /** Motion profiling constraints for rotation (narrower for auto) */
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(1.6).in(RadiansPerSecond), 
      RotationsPerSecond.of(12).in(RadiansPerSecond)   
  );

  /** PID controller for robot heading alignment during autonomous */
  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      3.0, 0, 0.2, // P=5.0 is aggressive but safe with a Profile
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;

  /** Inside this error the heading output is zeroed so the robot can settle. */
  private static final double kHeadingToleranceRads = Units.degreesToRadians(0.75);

  /** Tighter than teleop's band, since auto has no driver to judge the shot. */
  private static final double kAlignedToleranceRads = Units.degreesToRadians(3);

  private static final double kMaxYawRateDegPerSec = 20;
  private static final double kStictionFeedforwardRadsPerSec = Units.degreesToRadians(9);

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withRotationalDeadband(0) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  /**
   * Constructs a new autonomous AimBot command.
   * @param drivetrain The swerve drivetrain subsystem
   * @param photonVision The vision subsystem
   * @param shooter The shooter subsystem
   * @param index The indexer subsystem
   * @param hood The hood subsystem
   * @param metering The metering subsystem
   */
  public CMD_AimBotAuto(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, SUB_Hood hood, SUB_Metering metering) {    super(() -> {});
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    this.hood = hood;
    this.metering = metering;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // hood and metering are required so the hood's default command cannot drive it back to stow
    // while this command is trying to aim with it.
    addRequirements(drivetrain, shooter, index, hood, metering);
  }

  @Override
  public void initialize() {
    robotAngleController.setTolerance(kHeadingToleranceRads);

    targetTranslation = Hub.getGoalTranslation();

    // Reset PID controller to the current state of the robot
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    running = true;
    isThetaErrorCorrect = false;
    shooter.setHighPower(true);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    // No hub in the tag layout means no shot. Stand down rather than aiming at a made-up point.
    if (targetTranslation.isEmpty()) {
      isThetaErrorCorrect = false;
      index.setVolts(0);
      metering.set(0);
      return;
    }
    Translation2d goal = targetTranslation.get();

    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // Calculate target heading directly from shooter position to the hub
    Rotation2d targetRotation = new Rotation2d(
        goal.getX() - shooterFieldPosition.getX(),
        goal.getY() - shooterFieldPosition.getY()
    );

    drivetrain.publisher2.set(new Pose2d(shooterFieldPosition,targetRotation));
    drivetrain.publisher1.set(new Pose2d(goal, targetRotation));

    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Update alignment status for automated firing
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    
    isThetaErrorCorrect = thetaErrorRads <= kAlignedToleranceRads && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= kMaxYawRateDegPerSec;

    double distance = shooterFieldPosition.getDistance(goal);
    shooter.holdReadyBand(distance);
    hood.setAngle(SUB_Hood.angleForRPM(distance, shooter.getTargetRPM(), shooter.tunedRPM(distance))
        .orElseGet(() -> SUB_Hood.findoptimalangle(distance)));
    metering.set(1);

    // Automated firing trigger
    boolean isShooterReady = shooter.atDesiredRPM();

    if (isThetaErrorCorrect && isShooterReady) {
        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else {
        index.setVolts(0);
    }

    // Drive request with PID rotation (no translation in static auto aim)
    drivetrain.setControl(
      drive.withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(rotationalRate(omegaSpeed, thetaErrorRads)));
  }

  /**
   * See {@link CMD_AimBot#rotationalRate}. The controller output is already radians per second, so
   * it is clamped rather than multiplied by MaxAngularRate, and the stiction term is suppressed
   * inside the tolerance band so the robot can settle.
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
    index.setVolts(0);
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
