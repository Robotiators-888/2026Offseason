// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Main entry-point class for the FRC robot application.
 *
 * <p>Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main {
        private Main() {}

        /**
         * Main initialization function called by Java runtime.
         *
         * @param args Command line arguments passed on startup.
         */
        public static void main(String... args) {
                RobotBase.startRobot(Robot::new);
        }
}
