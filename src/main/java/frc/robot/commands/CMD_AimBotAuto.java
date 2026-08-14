// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.TrajectorySolver;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBotAuto extends Command {
  /** Subsystems and state variables for autonomous targeting */
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose = new Pose2d();
  private static boolean running;
  private final SUB_Shooter shooter;
  private final SUB_Index index;

  /** Physical offset from robot center to shooter exit */
  private final Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
  
  /** Motion profiling constraints for rotation (narrower for auto) */
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(1.6).in(RadiansPerSecond), 
      RotationsPerSecond.of(12).in(RadiansPerSecond)   
  );

  /** PID controller for robot heading alignment during autonomous */
  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      3.0, 0, 0.2,
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withRotationalDeadband(0) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  /**
   * Constructs a new autonomous AimBot command.
   */
  public CMD_AimBotAuto(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index) {
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrain, shooter, index);
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
    
    // Reset PID controller to current state
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
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

    // Calculate target heading directly from shooter position to hub
    Rotation2d targetRotation = new Rotation2d(
        targetTranslation.getX() - shooterFieldPosition.getX(),
        targetTranslation.getY() - shooterFieldPosition.getY()
    );

    drivetrain.publisher2.set(new Pose2d(shooterFieldPosition, targetRotation));
    drivetrain.publisher1.set(new Pose2d(targetTranslation, targetRotation));

    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Update alignment status for automated firing
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(3) 
        && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 20;
    
    double distance = currentPose.getTranslation().getDistance(targetTranslation);
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
    
    double clampedOmega = MathUtil.clamp(omegaSpeed, -MaxAngularRate, MaxAngularRate);
    double feedforward = Math.copySign(Units.degreesToRadians(9), clampedOmega);
    drivetrain.setControl(
      drive.withVelocityX(0)
           .withVelocityY(0)
           .withRotationalRate(clampedOmega + (Math.abs(clampedOmega) > 1e-4 ? feedforward : 0))
    );
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    SUB_Shooter.isShooting = false;
    index.setVolts(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning() {
    return running;
  }
}
