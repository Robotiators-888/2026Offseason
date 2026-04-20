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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.StatusCode;

public class CMD_AimBotSpecialLock extends CMD_AimBotBase {
  /**  State variables used for targeting and control */
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in X direction
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in Y direction

  // --- CUSTOM SWERVE REQUEST FOR EXPLICIT STATES ---
  private class ApplyModuleStates implements SwerveRequest {
      public SwerveModuleState[] targetStates = new SwerveModuleState[4];
      
      public ApplyModuleStates withStates(SwerveModuleState[] states) {
          this.targetStates = states;
          return this;
      }

      @Override
      public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?,?,?>... modulesToApply) {
          for (int i = 0; i < modulesToApply.length; ++i) {
              if (targetStates[i] != null) {
                  modulesToApply[i].apply(new SwerveModule.ModuleRequest()
                      .withState(targetStates[i])
                      .withDriveRequest(DriveRequestType.OpenLoopVoltage));
              }
          }
          return StatusCode.OK;
      }
  }
  private final ApplyModuleStates moduleStatesRequest = new ApplyModuleStates();

  /**
   * Constructs a new AimBot command.
   */
  public CMD_AimBotSpecialLock(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
    super(drivetrain, photonVision, shooter, index);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
  }

  @Override
  private boolean getBrakeRequestConditions () {
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));   
    return xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked;
  }

  @Override
  private SwerveRequest getDriveRequest (double omegaSpeed) {
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
    return drive
      .withVelocityX(xInput * MaxSpeed)
      .withVelocityY(yInput * MaxSpeed)
      .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate));
  }

  @Override
  private void doBrakeLogic (Pose2d currentPose, Translation2d targetTranslation) {
    // 1. Find the vector from the Robot to the Target in FIELD coordinates
    Translation2d fieldRelativeVectorToTarget = targetTranslation.minus(currentPose.getTranslation());

    // 2. Convert that vector to ROBOT coordinates
    Translation2d robotRelativeCenterOfRotation = fieldRelativeVectorToTarget.rotateBy(currentPose.getRotation().unaryMinus());

    // 3. Calculating the perfect perpendicular wheel angles
    ChassisSpeeds fakeOrbitSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0); // 1 rad/s rotation
    SwerveModuleState[] orbitStates = drivetrain.getKinematics().toSwerveModuleStates(fakeOrbitSpeeds, robotRelativeCenterOfRotation);

    // 4. Force speeds to zero so the robot stays perfectly still
    for (SwerveModuleState state : orbitStates) {
      state.speedMetersPerSecond = 0.0;
    }

    // 5. Apply the raw states explicitly using our custom Phoenix 6 ModuleRequest class
    drivetrain.setControl(moduleStatesRequest.withStates(orbitStates));
  }
}
