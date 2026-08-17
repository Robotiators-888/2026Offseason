// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Alert;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * Main Robot class extending AdvantageKit's {@link LoggedRobot}.
 *
 * <p>Manages the lifecycle of the robot across different match phases (Disabled, Autonomous,
 * Teleoperated, Test, and Simulation) and ensures periodic execution of the {@link CommandScheduler}.
 */
public class Robot extends LoggedRobot {
        private Command m_autonomousCommand;

        private final RobotContainer m_robotContainer;

        /**
         * Initializes data logging (AdvantageKit, WPILib DataLog, Phoenix SignalLogger),
         * instantiates the {@link RobotContainer}, and sets up alert mechanisms.
         */
        public Robot() {
                DataLogManager.start();
                DriverStation.startDataLog(DataLogManager.getLog());
                Logger.start();
                SignalLogger.start();

                // Instantiate our RobotContainer. This will perform all our button bindings, and
                // put our autonomous chooser on the dashboard.
                m_robotContainer = new RobotContainer();
                m_robotContainer.robotInit();
                Alert.setup();
        }

        /**
         * This function is called every 20 ms, no matter the mode. Use this for items like
         * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
         *
         * <p>This runs after the mode specific periodic functions, but before LiveWindow and
         * SmartDashboard integrated updating.
         */
        @Override
        public void robotPeriodic() {
                // Runs the Scheduler. This is responsible for polling buttons, adding
                // newly-scheduled commands, running already-scheduled commands, removing finished
                // or interrupted commands, and running subsystem periodic() methods. This must be
                // called from the robot's periodic block in order for anything in the Command-based
                // framework to work.
                CommandScheduler.getInstance().run();
                m_robotContainer.robotPeriodic();
        }

        /** This function is called once each time the robot enters Disabled mode. */
        @Override
        public void disabledInit() {}

        /**
         * Periodic method called while the robot is disabled.
         * delegates tasks to {@link RobotContainer#disabledPeriodic()}.
         */
        @Override
        public void disabledPeriodic() {
                m_robotContainer.disabledPeriodic();
        }

        /**
         * Runs once when autonomous mode is enabled. Retrieves and schedules the command
         * selected in {@link RobotContainer#getAutonomousCommand()}.
         */
        @Override
        public void autonomousInit() {
                m_autonomousCommand = m_robotContainer.getAutonomousCommand();

                // schedule the autonomous command (example)
                if (m_autonomousCommand != null) {
                        m_autonomousCommand.schedule();
                }
                m_robotContainer.autonomousInit();
        }

        /** Periodic method called during autonomous mode (every 20ms). */
        @Override
        public void autonomousPeriodic() {
                m_robotContainer.autonomousPeriodic();
        }

        /** Runs once when teleoperated mode is enabled. Cancels any active autonomous command. */
        @Override
        public void teleopInit() {
                // This makes sure that the autonomous stops running when
                // teleop starts running. If you want the autonomous to
                // continue until interrupted by another command, remove
                // this line or comment it out.
                if (m_autonomousCommand != null) {
                        m_autonomousCommand.cancel();
                }
                m_robotContainer.teleopInit();
        }

        /** Periodic method called during operator control (every 20ms). */
        @Override
        public void teleopPeriodic() {
                m_robotContainer.teleopPeriodic();
        }

        /** Runs once when test mode is enabled. Cancels all currently running commands. */
        @Override
        public void testInit() {
                // Cancels all running commands at the start of test mode.
                CommandScheduler.getInstance().cancelAll();
                m_robotContainer.testInit();
        }

        /** Periodic method called during test mode (every 20ms). */
        @Override
        public void testPeriodic() {
                m_robotContainer.testPeriodic();
        }

        /** This function is called once when the robot is first started up in simulation. */
        @Override
        public void simulationInit() {}

        /** This function is called periodically whilst in simulation (every 20ms). */
        @Override
        public void simulationPeriodic() {}
}
