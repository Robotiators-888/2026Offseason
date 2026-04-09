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
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBotMove extends RunCommand {
  // Holds constructor arguments and sets up variables
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose = new Pose2d();
  private static boolean running;
  private final SUB_Shooter shooter;
  private final SUB_Index index;
  Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-5));
  
  private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
      RotationsPerSecond.of(0.75).in(RadiansPerSecond), 
      RotationsPerSecond.of(1.5).in(RadiansPerSecond)   
  );

  private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
      5.0, 0, 0.2, // P=5.0 is aggressive but safe with a Profile
      thetaConstraints
  );
  public static boolean isThetaErrorCorrect = false;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  public CMD_AimBotMove(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index) {    super(() -> {});
    // Hold constructor arguments
    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.shooter = shooter;
    this.index = index;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    
    // Require subsystems
    addRequirements(drivetrain, shooter, index);
  }

  @Override
  public void initialize() {
    // Gets hub and tag positions
    robotAngleController.setTolerance(Units.degreesToRadians(0.0));
    
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
    shooter.shootMeters(distance-(drivetrain.getState().Speeds.vxMetersPerSecond*shooter.getExpectedTOF(distance)));
    SmartDashboard.putNumber("CMD_AimBot/TOF", distance-(drivetrain.getState().Speeds.vxMetersPerSecond*shooter.getExpectedTOF(distance)));
     // Keep metering wheel spinning
    boolean isShooterReady = shooter.atDesiredRPM();
    if (isThetaErrorCorrect && isShooterReady) {
      index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
      index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    } else if (!isThetaErrorCorrect) {
        index.setVolts(0);
    }
    double xInput = 1.3;

    
    drivetrain.setControl(
      drive.withVelocityX((!isThetaErrorCorrect||distance<1.5) ? 0:xInput)
      .withVelocityY(0)
      .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate)));

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
