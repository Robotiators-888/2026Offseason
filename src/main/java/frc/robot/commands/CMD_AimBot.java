// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

public class CMD_AimBot extends RunCommand {
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private final SUB_Shooter shooter;
  private final SUB_Index index;
  private Pose2d targetPose = new Pose2d();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;

  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(0.75).in(RadiansPerSecond), 
      RotationsPerSecond.of(1.5).in(RadiansPerSecond)   
  );

  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      5.0, 0, 0.2, // P=5.0 is aggressive but safe with a Profile
      thetaConstraints
  );

  public static boolean isThetaErrorCorrect = false;
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private double MaxSpeed = 1.0; // Slow down for shoot on the move
  private final Translation2d shooterOffset = new Translation2d(
        Units.inchesToMeters(-10), 
        Units.inchesToMeters(-5)
    );
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Operator.kDriveDeadband).withRotationalDeadband(0) 
            .withDriveRequestType(DriveRequestType.Velocity).withCenterOfRotation(shooterOffset); 
  public CMD_AimBot(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {    super(() -> {});
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    
    addRequirements(drivetrain,shooter,index);
  }

  @Override
  public void initialize() {
    robotAngleController.setTolerance(Units.degreesToRadians(5.0));
    
    Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
      
    double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
    Translation2d hubCenterTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    
    targetPose = new Pose2d(hubCenterTranslation, new Rotation2d());
    
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    running = true;
  }

  @Override
  public void execute() {
    // Robot Pose and Math for SoTM
    Pose2d currentPose = drivetrain.getPose();
    Translation2d shooterFieldPosition = currentPose.getTranslation()
            .plus(shooterOffset.rotateBy(currentPose.getRotation()));
    double distanceToRealTarget = shooterFieldPosition.getDistance(targetPose.getTranslation()); // Clsoe enough, we don't need ICBM level precision for a 2-3 meter shot
    double tof = shooter.getExpectedTOF(distanceToRealTarget);
    var chassisSpeeds = drivetrain.getCurrentRobotChassisSpeeds(); 
    Translation2d robotVelocity = new Translation2d(
        chassisSpeeds.vxMetersPerSecond, 
        chassisSpeeds.vyMetersPerSecond
    ).rotateBy(currentPose.getRotation());
    Translation2d virtualTarget = targetPose.getTranslation().minus(robotVelocity.times(tof)); 
    Rotation2d targetRotation = new Rotation2d(
        virtualTarget.getX() - shooterFieldPosition.getX(),
        virtualTarget.getY() - shooterFieldPosition.getY()
    );
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(
        currentPose.getRotation().getRadians() - targetRotation.getRadians()
    ));    
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(2) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 5;
    boolean isPerfectlyAligned = thetaErrorRads <= Units.degreesToRadians(1) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 5;

    // Logging for debugging and tuning
    drivetrain.publisher1.set(new Pose2d(virtualTarget, targetRotation));
    drivetrain.publisher2.set(new Pose2d(shooterFieldPosition, targetRotation)); 
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));

    // Feed and shoot control
    double distanceToVirtualTarget = shooterFieldPosition.getDistance(virtualTarget);
    shooter.shootMeters(distanceToVirtualTarget); 
    
    index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM); // Keep metering wheel spinning
    boolean isShooterReady = shooter.atDesiredRPM();
    boolean isMeteringReady = Math.abs(index.intakeMeteringRPM() - Constants.Index.kINDEX_METERING_MOTOR_RPM) < 100;
    if (isThetaErrorCorrect && isShooterReady && isMeteringReady) {
        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else {
        index.setVolts(-1.0);
    }   


    // Rotational Control
    double pidOutput = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );
    
    double xInput = translationXSupplier.getAsDouble();
    double yInput = translationYSupplier.getAsDouble();
    
    if (Math.abs(xInput) < Operator.kDriveDeadband && Math.abs(yInput) < Operator.kDriveDeadband && isPerfectlyAligned) {
        drivetrain.setControl(brakeRequest);
    } else {
        drivetrain.setControl(
          drive.withVelocityX(xInput * MaxSpeed)
          .withVelocityY(yInput * MaxSpeed)
          .withRotationalRate((pidOutput)));
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    shooter.stop();
    index.set(0);
    index.stopMetering();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning () {
    return running;
  }
}