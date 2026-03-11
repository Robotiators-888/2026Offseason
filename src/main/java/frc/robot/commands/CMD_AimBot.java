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
  private Pose2d targetPose = new Pose2d();
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private static boolean running;
  private final SUB_Shooter shooter;
  private final SUB_Index index;
  private boolean isLocked;
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
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withRotationalDeadband(0) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  public CMD_AimBot(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {    super(() -> {});
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
    robotAngleController.setTolerance(Units.degreesToRadians(0.0));
    
    Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
      ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
      : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
      
    double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
    Translation2d hubCenterTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());
    
    // Save as targetPose for visualization (Rotation doesn't matter yet, it gets calculated in execute)
    targetPose = new Pose2d(hubCenterTranslation, new Rotation2d());
    
    robotAngleController.reset(
        drivetrain.getPose().getRotation().getRadians(),
        drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
    );
    isLocked = false;
    running = true;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose();
    
    // 1. Define the shooter's physical offset relative to the robot's center
    // Back Right translates to negative X and negative Y
    Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-5));
    
    // 2. Rotate this offset by the robot's current heading to orient it to the field
    Translation2d rotatedShooterOffset = shooterOffset.rotateBy(currentPose.getRotation());
    
    // 3. Add the rotated offset to the robot's center field position to get the shooter's field position
    Translation2d shooterFieldPosition = currentPose.getTranslation().plus(rotatedShooterOffset);

    // 4. Calculate the angle required for the *shooter* to face the *hub center*
    Translation2d targetTranslation = targetPose.getTranslation();
    Rotation2d targetRotation = new Rotation2d(
        targetTranslation.getX() - shooterFieldPosition.getX(),
        targetTranslation.getY() - shooterFieldPosition.getY()
    );

    // Update telemetry so you can see where the robot is trying to aim in AdvantageScope/Glass
    drivetrain.publisher1.set(new Pose2d(targetTranslation, targetRotation));
    // drivetrain.publisher2.set(new Pose2d(shooterFieldPosition, targetRotation)); // Visualizing the shooter position

    // 5. Calculate rotational velocity (omega) using the PID controller
    double omegaSpeed = robotAngleController.calculate(
        currentPose.getRotation().getRadians(),
        targetRotation.getRadians()
    );

    // Calculate error for deadband checking
    double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
    SmartDashboard.putNumber("CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));
    
    isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(2) && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= 5;
    double distance = drivetrain.getPose().getTranslation().getDistance(
            SUB_PhotonVision.getInstance().at_field.getTagPose(
                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
            ).map(pose -> pose.toPose2d().getTranslation().plus(
                    new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 0)
            )).orElse(drivetrain.getPose().getTranslation())
    );
    shooter.shootMeters(distance);
    
    index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM); // Keep metering wheel spinning
    boolean isShooterReady = shooter.atDesiredRPM();
    boolean isMeteringReady = Math.abs(index.intakeMeteringRPM() - Constants.Index.kINDEX_METERING_MOTOR_RPM) < 100;
    if (isThetaErrorCorrect && isShooterReady && isMeteringReady) {
        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
    }
    double xInput = MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband);
    double yInput = MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband);

    if (!isLocked && thetaErrorRads <= Units.degreesToRadians(1)) {
      isLocked = true;
    }
    else if (isLocked && thetaErrorRads >= Units.degreesToRadians(2)) {
      isLocked = false;
    }

    if (xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked) {
        drivetrain.setControl(brakeRequest);
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
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean isRunning () {
    return running;
  }
}
