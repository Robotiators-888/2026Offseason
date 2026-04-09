package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class CMD_PredictiveAimAuto extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SUB_PhotonVision photonVision;
    private final SUB_Shooter shooter;
    private final SUB_Index index;
    
    private Translation2d staticTargetTranslation;
    private final Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-5));
    
    // The maximum allowed sideways drift of the shooter during the ball's flight (in meters)
    private final double MAX_ALLOWED_TANGENTIAL_DISPLACEMENT = 0.15; // 15 cm threshold
    
    // Custom PID specifically for Auto Aiming
    private final ProfiledPIDController autoAimAngleController = new ProfiledPIDController(
        1.5, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(
            Units.rotationsToRadians(1.6), 
            Units.rotationsToRadians(5)
        )
    );
    
    // The dynamic rotational feedback (rad/s) we will feed to PathPlanner
    private double currentOmegaOverride = 0.0;

    public CMD_PredictiveAimAuto(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision, SUB_Shooter shooter, SUB_Index index) {
        this.drivetrain = drivetrain;
        this.photonVision = photonVision;
        this.shooter = shooter;
        this.index = index;
        
        autoAimAngleController.enableContinuousInput(-Math.PI, Math.PI);

        // Drivetrain is NOT required here because PathPlanner's active path command "owns" it.
        addRequirements(shooter, index); 
    }

    @Override
    public void initialize() {
        // Determine the correct target tag based on current alliance
        Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
            ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
            : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();
        
        double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? Units.inchesToMeters(-23.5) : Units.inchesToMeters(23.5);
        staticTargetTranslation = new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());

        // Reset the PID to prevent initial jerking
        autoAimAngleController.reset(
            drivetrain.getPose().getRotation().getRadians(),
            drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
        );
        
        // --- NEW PATHPLANNER API ---
        // Inject our custom PID output directly into the Holonomic Drive Controller
        PPHolonomicDriveController.overrideRotationFeedback(() -> currentOmegaOverride);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getCurrentRobotChassisSpeeds(), 
            currentPose.getRotation()
        );

        // 1. Calculate Base TOF
        double distanceToHub = currentPose.getTranslation().getDistance(staticTargetTranslation);
        double tof = shooter.getExpectedTOF(distanceToHub);

        // 2. Virtual Target
        Translation2d virtualTarget = new Translation2d(
            staticTargetTranslation.getX() - (fieldSpeeds.vxMetersPerSecond * tof),
            staticTargetTranslation.getY() - (fieldSpeeds.vyMetersPerSecond * tof)
        );

        Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
            shooterOffset.rotateBy(currentPose.getRotation())
        );

        Rotation2d targetRotation = new Rotation2d(
            virtualTarget.getX() - shooterFieldPosition.getX(),
            virtualTarget.getY() - shooterFieldPosition.getY()
        );

        // 3. Calculate Rotational Velocity (Omega) via our own PID
        // Update the class variable that PathPlanner is continuously reading
        currentOmegaOverride = autoAimAngleController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );

        // Add stiction/friction feedforward if necessary (mirrored from your teleop logic)
        currentOmegaOverride += Math.copySign(Units.degreesToRadians(9), currentOmegaOverride);

        // 4. Alignment Check
        double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
        boolean isThetaCorrect = thetaErrorRads <= Units.degreesToRadians(4);

        // 5. The Math: Rotational Displacement Limit
        // Hypotenuse (radius) of the shooter offset triangle
        double shooterRadius = shooterOffset.getNorm(); 
        double angularVelocity = Math.abs(drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        
        // Tangential displacement = Tangential Velocity (omega * radius) * TimeOfFlight
        double tangentialDisplacement = (angularVelocity * shooterRadius) * tof;
        boolean isRotationalDisplacementSafe = tangentialDisplacement <= MAX_ALLOWED_TANGENTIAL_DISPLACEMENT;

        // Telemetry
        SmartDashboard.putNumber("AutoAim/Tangential Displacement (m)", tangentialDisplacement);
        SmartDashboard.putBoolean("AutoAim/Rotational Safe", isRotationalDisplacementSafe);
        SmartDashboard.putBoolean("AutoAim/Theta Correct", isThetaCorrect);

        // 6. Set Shooter RPM
        shooter.shootMeters(currentPose.getTranslation().getDistance(virtualTarget));

        // 7. Auto-Fire Logic
        if (isThetaCorrect && isRotationalDisplacementSafe) {
            index.setMeteringRPM(Constants.Index.kINDEX_METERING_MOTOR_RPM);
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
        } else {
            index.setVolts(0);
            index.setMeteringRPM(-1000); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        // --- NEW PATHPLANNER API ---
        // Give rotation control back to the PathPlanner trajectory when finished
        PPHolonomicDriveController.clearRotationFeedbackOverride();
        index.setVolts(0);
    }
}