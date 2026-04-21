// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.Operator;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CMD_AimBot extends CMD_AimBotBase {
  /** State variables used for targeting and control */
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in X direction
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0); // Limit acceleration to 3 m/s^2 in Y direction

  /**
   * Constructs a new AimBot command.
   * @param drivetrain The swerve drivetrain subsystem
   * @param photonVision The vision subsystem for target tracking
   * @param shooter The shooter subsystem
   * @param index The indexer subsystem
   * @param translationXSupplier Supplier for X translation input
   * @param translationYSupplier Supplier for Y translation input
   */
  public CMD_AimBot(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
    // Call the parent class constructor with the require subsystems
    super(drivetrain, photonVision, shooter, index);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
  }

  @Override
  protected boolean getBrakeRequestConditions () {
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
    return xInput == 0.0 && yInput == 0.0 && isThetaErrorCorrect && isLocked;
  }

  // Make sure to be able to drive with joysticks
  @Override
  protected SwerveRequest getDriveRequest (double omegaSpeed) {
    double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
    double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));
    return drive
      .withVelocityX(xInput * MaxSpeed)
      .withVelocityY(yInput * MaxSpeed)
      .withRotationalRate(omegaSpeed * MaxAngularRate + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate));
  }
}
