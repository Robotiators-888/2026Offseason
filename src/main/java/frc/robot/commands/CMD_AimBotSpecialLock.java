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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CMD_AimBotSpecialLock extends RunCommand {
  /** Subsystems and state variables used for targeting and control */
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose = new Pose2d();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;
  private final SUB_Shooter shooter;
  private final SUB_Index index;
  private boolean isLocked;
  private boolean wasLocked; // Tracks state for Coast/Brake transitions

  /** Physical offsets for targeting calibration */
  Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-5));
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
  private double MaxSpeed = 2.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
            
  // --- CUSTOM SWERVE REQUEST FOR EXPLICIT STATES ---
  // This satisfies CTRE's architecture while giving us bare-metal control of the angles
  private class ApplyModuleStates implements SwerveRequest {
      public SwerveModuleState[] targetStates = new SwerveModuleState[4];
      
      public ApplyModuleStates withStates(SwerveModuleState[] states) {
          this.targetStates = states;
          return this;
      }

      @Override
      public com.ctre.phoenix6.StatusCode apply(com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters parameters, com.ctre.phoenix6.swerve.SwerveModule<?,?,?>... modulesToApply) {
          for (int i = 0; i < modulesToApply.length; ++i) {
              if (targetStates[i] != null) {
                  // This is the ModuleRequest from the Phoenix 6 docs!
                  modulesToApply[i].apply(new com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest()
                      .withState(targetStates[i])
                      .withDriveRequest(DriveRequestType.OpenLoopVoltage));
              }
          }
          return com.ctre.phoenix6.StatusCode.OK;
      }
  }
  private final ApplyModuleStates moduleStatesRequest = new ApplyModuleStates();
  // --------------------------------------------------

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in X direction
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in Y direction
  
  /**
   * Constructs a new AimBot command.
   */
  public CMD_AimBotSpecialLock(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {    
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
    // Determine the correct target tag based on the current alliance
    robotAngleController.setTolerance(Units.degreesToRadians(0.0));
    
    Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
      
    double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
    Translation2d hubCenterTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    
    targetPose = new Pose2d(hubCenterTranslation, new Rotation2d());
    
    // Reset the PID controller to the current state of the robot
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    isLocked = false;
    wasLocked = false;
    running = true;
  }

  @Override
  public void execute() {
    // Set up poses
    Pose2d currentPose = drivetrain.getPose();

    Translation2d targetTranslation = targetPose.getTranslation();
    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
        shooterOffset.rotateBy(currentPose.getRotation())
    );

    // 2. Calculate the angle directly from the SHOOTER to the target
    Rotation2d targetRotation = new Rotation2d(
        targetTranslation.getX() - shooterFieldPosition.getX(),
        targetTranslation.getY() - shooterFieldPosition.getY()
    );

    targetRotation = targetRotation.plus(shooterThetaOffset);

    // Update telemetry
    drivetrain.publisher1.set(new Pose2d(targetTranslation, targetRotation));

    // 3. Calculate rotational velocity (omega) using the PID controller
    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Calculate error for deadband checking
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(5) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 20;
    SmartDashboard.putBoolean("CMD_AimBot/isThetaErrorCorrect",isThetaErrorCorrect);
    double distance = drivetrain.getPose().getTranslation().getDistance(
            SUB_PhotonVision.getInstance().at_field.getTagPose(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
            ).map(pose -> pose.toPose2d().getTranslation().plus(
                    new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 0)
            )).orElse(drivetrain.getPose().getTranslation())
    );
    shooter.shootMeters(distance);
    
     // Keep metering wheel spinning
    boolean isShooterReady = shooter.atDesiredRPM();
    if (isThetaErrorCorrect && isShooterReady) {
      index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
      index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else if (!isThetaErrorCorrect) {
        index.setVolts(0);
        index.setMeteringRPM(-1000);
    }
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));

    // Wheel locking logic
    if (!isLocked && thetaErrorRads <= Units.degreesToRadians(1)) {
      isLocked = true;
    }
    else if (isLocked && thetaErrorRads >= Units.degreesToRadians(5)) {
      isLocked = false;
    }

    // --- NEUTRAL MODE TRANSITIONS ---
    if (isLocked && !wasLocked) {
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            drivetrain.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        }
    } else if (!isLocked && wasLocked) {
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            drivetrain.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
        }
    }
    wasLocked = isLocked;

    // Lock wheels or drive
    if (xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked) {
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
        
    } else {
        drivetrain.setControl(
          drive.withVelocityX(xInput * MaxSpeed)
          .withVelocityY(yInput * MaxSpeed)
          .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate))); 
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    
    // Safety check
    if (wasLocked) {
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            drivetrain.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
        }
        wasLocked = false;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning () {
    return running;
  }
}