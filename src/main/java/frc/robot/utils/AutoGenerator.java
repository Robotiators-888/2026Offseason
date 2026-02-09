package frc.robot.utils;

import frc.robot.subsystems.SUB_Drivetrain;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;


public class AutoGenerator extends SubsystemBase {
    public static SUB_Drivetrain drivetrain = SUB_Drivetrain.getInstance();
    private static AutoGenerator INSTANCE = null;
    public static boolean reachedAutoTarget;
    public static boolean intakecomplete = true;

    public AutoGenerator() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
        AutoBuilder.configure(drivetrain::getPose, // Robot pose supplier
                drivetrain::resetPose, // Method to reset odometry (will be called if your auto has
                                       // a starting pose)
                drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds), // Method that will
                                                                                 // drive the robot
                                                                                 // given ROBOT
                                                                                 // RELATIVE
                                                                                 // ChassisSpeeds.
                                                                                 // Also optionally
                                                                                 // outputs
                                                                                 // individual
                                                                                 // module
                                                                                 // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                                                // following controller for holonomic drive trains
                        new PIDConstants(3, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(2, 0.0, 0.0) // Rotation PID constants
                ), config, // The robot configuration
                () -> {
                    return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
                }, drivetrain // Reference to this subsystem to set requirements
        );

        registerAllCommands();
    }

    public void registerAllCommands() {}

    public static AutoGenerator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoGenerator();
        }

        return INSTANCE;
    }

    public void setreachedtarget(boolean value) {
        reachedAutoTarget = value;
        SmartDashboard.putBoolean("ReachedAutoTarget", reachedAutoTarget);
    }

    public boolean getreachedtarget() {
        return reachedAutoTarget;

    }

    public void setintakecomplete(boolean value) {
        intakecomplete = value;
        SmartDashboard.putBoolean("IntakeComplete", intakecomplete);
    }

    public boolean getintakecomplete() {
        return intakecomplete;
    }

    public void periodic() {
    }
}
