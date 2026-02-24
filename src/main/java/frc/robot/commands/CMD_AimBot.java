// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.SUB_PhotonVision;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBot extends RunCommand {
  private final SUB_PhotonVision photonVision;
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose = new Pose2d();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;

  private final PIDController robotAngleController = new PIDController(1.5, 0, 0.05);
  public static boolean isThetaErrorCorrect = false;
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();


  public CMD_AimBot(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
    super(() -> {});


    this.drivetrain = drivetrain;
    this.photonVision = photonVision;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
    
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    robotAngleController.setTolerance(Units.degreesToRadians(5.0));
    Pose2d currentPose = drivetrain.getPose();

    Pose2d tPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
    Rotation2d targetRotation = new Rotation2d(tPose.getX()-currentPose.getX(),tPose.getY()-currentPose.getY());
    targetPose = new Pose2d(tPose.getX()+((DriverStation.getAlliance().equals(Optional.of(Alliance.Red))) ?  Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5)), tPose.getY(),
        targetRotation);
    robotAngleController.reset();
    drivetrain.publisher2.set(targetPose);
    running = true;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();

    drivetrain.publisher1.set(targetPose);


    Rotation2d targetRotation = new Rotation2d(targetPose.getX()-currentPose.getX(),targetPose.getY()-currentPose.getY());
    double omegaSpeed = robotAngleController.calculate(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        MathUtil.angleModulus(targetRotation.getRadians())
    );

    double thetaErrorRads = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    SmartDashboard.putNumber("Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(5);
    double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), 0.05);
    double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), 0.05);
    if (xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect) {
        drivetrain.setControl(brakeRequest);
    } else {
        drivetrain.drive(xInput, yInput, omegaSpeed, true, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    // No specific actions on end
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning () {
    return running;
  }
}
