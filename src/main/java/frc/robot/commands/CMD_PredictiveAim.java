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
import frc.robot.commands.CMD_AimBotBase;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Advanced targeting command that compensates for robot movement using time-of-flight prediction.
 * Instead of reacting to error, it calculates a "virtual target" based on where the hub 
 * will be relative to the robot when the ball arrives.
 */
public class CMD_PredictiveAim extends CMD_AimBotBase {
  /** Subsystems and state variables for predictive targeting */
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier; 
  
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
    super(drivetrain, photonVision, shooter, index);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
  }

  @Override
  protected Translation2d getTargetTranslation (Pose2d currentPose) {
    // Convert robot-centric speeds to field-centric for accurate target shifting
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getCurrentRobotChassisSpeeds(), 
        currentPose.getRotation()
    );

    // 1. Calculate base distance and initial Time of Flight (TOF)
    double distanceToHub = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double tof = shooter.getExpectedTOF(distanceToHub);

    // 2. Calculate Virtual Target: PhysicalTarget - (RobotVelocity * TOF)
    // Moving towards the goal (positive velocity) makes the virtual target closer.
    return new Translation2d(
        targetPose.getX() - (fieldSpeeds.vxMetersPerSecond * tof),
        targetPose.getY() - (fieldSpeeds.vyMetersPerSecond * tof)
    );
  }

  @Override
  protected double getDistanceFromTarget (Translation2d shooterFieldPosition, Translation2d targetTranslation) {
    return shooterFieldPosition.getDistance(targetTranslation);
  }

  @Override
  protected SwerveRequest getDriveRequest (double omegaSpeed) {
    // double xInput = 0.0;//xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double magnitude = Math.sqrt(Math.pow(xInput,2)+Math.pow(yInput,2));

    if (magnitude == 0.0) {
      magnitude = 1.0; // No divivde by zero, and if we're not commanding movement, we don't need to limit it
    }
    
    // We add a small constant "kick" to overcome friction when moving proactively
    double rotationOutput = omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed);
    
    return drive
      .withVelocityX(xInput * MaxSpeed * (1/magnitude))
      .withVelocityY(yInput * MaxSpeed * (1/magnitude))
      .withRotationalRate(rotationOutput);
  }
}
