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
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Base class for all Aimbot Commands
// Even though the class has no abstract methods it is abstract becuase the methods defined don't make sense for a telop or auto command
// It has a brake request (doesn't make sense for auto) but has no teleop drive controls (doesn't make sense for teleop)
public abstract class CMD_AimBotBase extends RunCommand {
  /** Subsystems and state variables used for targeting and control */
  protected final SUB_PhotonVision photonVision;
  protected final CommandSwerveDrivetrain drivetrain;
  protected Pose2d targetPose = new Pose2d();
  // Shouldn't be protected because its behavior shouldn't change
  private static boolean running;
  protected final SUB_Shooter shooter;
  protected final SUB_Index index;
  protected boolean isLocked;

  /** Physical offsets for targeting calibration */
  protected Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-5));
  protected Rotation2d shooterThetaOffset = new Rotation2d(Units.degreesToRadians(0)); // CounterClockwise Positive
  
  /** Motion profiling constraints for rotation */
  protected final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(1.6).in(RadiansPerSecond), 
      RotationsPerSecond.of(12).in(RadiansPerSecond)   
  );

  /** PID controller for robot heading alignment */
  protected final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      5.0, 0, 0.2, // P=5.0 is aggressive but safe with a Profile
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;
  protected final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  protected double MaxSpeed = 2.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  protected double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  protected final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  
  /**
   * Constructs the AimBotBase class.
   * @param drivetrain The swerve drivetrain subsystem
   * @param photonVision The vision subsystem for target tracking
   * @param shooter The shooter subsystem
   * @param index The indexer subsystem
   */
  public CMD_AimBotBase(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index) {
    super(() -> {});
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    // Could be put in initialize
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain, shooter, index);
  }

  // This depricated is here to tell you that this constructor should only be called if you know what you are doing
  // This constructor doesn't require the drive subsystem
  @Depricated
  public CMD_AimBotBase(SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, CommandSwerveDrivetrain drivetrain) {
    super(() -> {});
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    this.drivetrain = drivetrain;
    // Could be put in initialize
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Doesn't require the drivetrain be careful
    addRequirements(shooter, index);
  }

  // The idea is that initialize and execute shouldn't need to be overrided again
  @Override
  public void initialize() {
    robotAngleController.setTolerance(Units.degreesToRadians(0.0));

    targetPose = getTargetPose();

    // Reset the PID controller to the current state of the robot
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    isLocked = false;
    running = true;
  }

  @Override
  public void execute() {
    // Set up poses
    Pose2d currentPose = drivetrain.getPose();
    Translation2d targetTranslation = getTargetTranslation();
    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // 2. Calculate the angle directly from the SHOOTER to the target
    Rotation2d targetRotation = new Rotation2d(
        targetTranslation.getX() - shooterFieldPosition.getX(),
        targetTranslation.getY() - shooterFieldPosition.getY()
    );

    // SOTM doesn't have this for some reason
    targetRotation = targetRotation.plus(shooterThetaOffset);

    // Update telemetry
    // Static pose (Always the hub)
    drivetrain.publisher1.set(new Pose2d(targetPose.getTranslation(), targetPose.getRotation()));
    // Virtual pose (Real target for SOTM but same as publisher1 for everything else)
    drivetrain.publisher2.set(new Pose2d(targetTranslation, targetRotation));

    // 3. Calculate rotational velocity (omega) using the PID controller
    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Calculate error for deadband checking
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));

    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(5) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 20;
    SmartDashboard.putBoolean("CMD_AimBot/isThetaErrorCorrect", isThetaErrorCorrect);
    double distance = getDistanceFromTarget();
    SmartDashboard.putNumber("CMD_AimBot/Distance (m)", distance);
    shooter.shootMeters(distance);

    // Keep metering wheel spinning
    boolean isShooterReady = shooter.atDesiredRPM();
    if (isThetaErrorCorrect && isShooterReady) {
      index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
      index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else if (!isThetaErrorCorrect) {
        index.setVolts(0);
        index.setMeteringRPM(0);
    }

    // Wheel locking logic
    if (!isLocked && thetaErrorRads <= Units.degreesToRadians(2)) {
      isLocked = true;
    }
    else if (isLocked && thetaErrorRads >= Units.degreesToRadians(5)) {
      isLocked = false;
    }

    // Lock wheels or drive
    if (getBrakeRequestConditions())
      doBrakeLogic(currentPose, targetTranslation);
    else
      drivetrain.setControl(getDriveRequest(omegaSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  // The command keeps running until told to stop it has no natural finishing condition
  @Override
  public boolean isFinished() {
    return false;
  }

  //**************************************//
  // Methods that can/should be overriden
  //**************************************//

  // Override this method to get a different target pose
  // Plus this is also good for code organization
  protected Pose2d getTargetPose () {
    // Determine the correct target tag based on the current alliance
    Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
    double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
    Translation2d hubCenterTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    return new Pose2d(hubCenterTranslation, new Rotation2d());
  }

  // Override this for a different target translation which is used for SOTM
  protected Translation2d getTargetTranslation () {
    return targetPose.getTranslation();
  }

  // Override this to return false for an auto command
  protected boolean getBrakeRequestConditions () {
    return isThetaErrorCorrect && isLocked;
  }

  // Override this for special brake request
  protected void doBrakeLogic (Pose2d currentPose, Translation2d targetTranslation) {
    drivetrain.setControl(brakeRequest);
  }

  // Get a drive request, override this for controller inputs
  protected SwerveRequest getDriveRequest(double omegaSpeed) {
    return drive
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate));
  }

  // Override this in case the target isn't the hub
  protected double getDistanceFromTarget () {
    return drivetrain.getPose().getTranslation().getDistance(
            SUB_PhotonVision.getInstance().at_field.getTagPose(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
            ).map(pose -> pose.toPose2d().getTranslation().plus(
                    new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 0)
            )).orElse(drivetrain.getPose().getTranslation())
    );
  }

  //******************//
  // Static method(s)
  //******************//

  public static boolean isRunning () {
    return running;
  }
}
