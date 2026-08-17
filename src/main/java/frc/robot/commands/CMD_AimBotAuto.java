// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Thanks Omar for the name AimBot, it is a very good name for this command
package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
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
import java.util.Optional;

/**
 * Autonomous command for vision-guided target heading alignment and automatic firing.
 *
 * <p>Requires {@link CommandSwerveDrivetrain}, {@link SUB_Shooter}, and {@link SUB_Index}.
 * Controls rotation towards the alliance Hub target while stationary, setting shooter RPM based
 * on distance and triggering indexer feed when rotation tolerance and flywheel speeds are satisfied.
 */
public class CMD_AimBotAuto extends RunCommand {
        /** Subsystems and state variables for autonomous targeting */
        private final SUB_PhotonVision photonVision;
        private final CommandSwerveDrivetrain drivetrain;
        private Pose2d targetPose = new Pose2d();
        private static boolean running;
        private final SUB_Shooter shooter;
        private final SUB_Index index;

        /** Physical offset from robot center to shooter exit */
        Translation2d shooterOffset =
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));

        /** Motion profiling constraints for rotation (1.6 rot/s max velocity, 12 rot/s^2 max acceleration). */
        private final TrapezoidProfile.Constraints thetaConstraints =
            new TrapezoidProfile.Constraints(RotationsPerSecond.of(1.6).in(RadiansPerSecond),
                RotationsPerSecond.of(12).in(RadiansPerSecond));

        /** Profiled PID controller for robot heading alignment during autonomous (P=3.0, I=0.0, D=0.2). */
        private final ProfiledPIDController robotAngleController =
            new ProfiledPIDController(3.0, 0, 0.2,
                thetaConstraints);

        /** Status flag indicating whether heading error is within 3 degrees tolerance during autonomous. */
        public static boolean isThetaErrorCorrect = false;

        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric().withRotationalDeadband(0).withDriveRequestType(
                DriveRequestType.OpenLoopVoltage);

        /**
         * Constructs a new autonomous AimBot command.
         *
         * @param drivetrain The swerve drivetrain subsystem.
         * @param photonVision The vision subsystem.
         * @param shooter The shooter subsystem.
         * @param index The indexer subsystem.
         */
        public CMD_AimBotAuto(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision,
            SUB_Shooter shooter, SUB_Index index) {
                super(() -> {});
                this.drivetrain = drivetrain;
                this.photonVision = photonVision;
                this.shooter = shooter;
                this.index = index;
                robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

                addRequirements(drivetrain, shooter, index);
        }

        /**
         * Command initialization. Resolves target AprilTag pose based on alliance color and resets heading PID controller.
         */
        @Override
        public void initialize() {
                robotAngleController.setTolerance(Units.degreesToRadians(0.0));

                // Determine the correct target tag based on current alliance
                Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
                    ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
                    : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();

                double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
                    ? Units.inchesToMeters(-23.5)
                    : Units.inchesToMeters(23.5);
                Translation2d hubCenterTranslation =
                    new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());

                targetPose = new Pose2d(hubCenterTranslation, new Rotation2d());

                // Reset PID controller to the current state of the robot
                robotAngleController.reset(drivetrain.getPose().getRotation().getRadians(),
                    drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
                running = true;
        }

        /**
         * Command execution loop (20ms). Calculates rotational velocity, sets distance-interpolated shooter RPM,
         * and activates indexer feed when rotational error is within 3 degrees and flywheels are at target speed.
         */
        @Override
        public void execute() {
                Pose2d currentPose = drivetrain.getPose();

                Translation2d targetTranslation = targetPose.getTranslation();
                Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
                    shooterOffset.rotateBy(currentPose.getRotation()));

                // Calculate target heading directly from shooter position to the hub
                Rotation2d targetRotation =
                    new Rotation2d(targetTranslation.getX() - shooterFieldPosition.getX(),
                        targetTranslation.getY() - shooterFieldPosition.getY());

                drivetrain.publisher2.set(new Pose2d(shooterFieldPosition, targetRotation));
                drivetrain.publisher1.set(new Pose2d(targetTranslation, targetRotation));

                double omegaSpeed = robotAngleController.calculate(
                    currentPose.getRotation().getRadians(), targetRotation.getRadians());

                // Update alignment status for automated firing
                double thetaErrorRads = Math.abs(MathUtil.angleModulus(
                    currentPose.getRotation().getRadians() - targetRotation.getRadians()));
                SmartDashboard.putNumber(
                    "CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));

                isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(3)
                    && Math.abs(
                           drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble())
                        <= 20;

                double distance = drivetrain.getPose().getTranslation().getDistance(
                    SUB_PhotonVision.getInstance()
                        .at_field
                        .getTagPose(
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10
                                                                                              : 26)
                        .map(pose
                            -> pose.toPose2d().getTranslation().plus(new Translation2d(
                                Units.inchesToMeters(
                                    DriverStation.getAlliance().orElse(Alliance.Blue)
                                            == Alliance.Red
                                        ? -23.5
                                        : 23.5),
                                0)))
                        .orElse(drivetrain.getPose().getTranslation()));
                shooter.shootMeters(distance);

                // Automated firing trigger
                boolean isShooterReady = shooter.atDesiredRPM();

                if (isThetaErrorCorrect && isShooterReady) {
                        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                } else if (!isThetaErrorCorrect) {
                        index.setVolts(0);
                }

                // Drive request with PID rotation (no translation in static auto aim)
                drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(
                    omegaSpeed * MaxAngularRate
                    + Math.copySign(Units.degreesToRadians(9), omegaSpeed * MaxAngularRate)));
        }

        /**
         * Resets running state when ended or interrupted.
         *
         * @param interrupted True if command was interrupted.
         */
        @Override
        public void end(boolean interrupted) {
                running = false;
        }

        /**
         * Returns whether command is complete. Always returns false (runs until canceled).
         *
         * @return False.
         */
        @Override
        public boolean isFinished() {
                return false;
        }

        /**
         * Returns whether the autonomous AimBot command is currently active.
         *
         * @return True if running, false otherwise.
         */
        public static boolean isRunning() {
                return running;
        }
}
