// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_AimBotAuto extends CMD_AimBotBase {
  public CMD_AimBotAuto(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index) {
    super(drivetrain, photonVision, shooter, index); 
  }

  @Override
  protected boolean getBrakeRequestConditions () {
    return false;
  }

  @Override
  protected void doBrakeLogic (Pose2d currentPose, Translation2d targetTranslation) {}
}
