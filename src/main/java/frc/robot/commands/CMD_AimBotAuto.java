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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;
import java.util.Optional;

/**
 * Command for automated vision-guided target alignment, hood positioning, and shooting feed control.
 *
 * <p>Requires {@link CommandSwerveDrivetrain}, {@link SUB_Metering}, {@link SUB_Index}, and {@link SUB_Hood}.
 * Allows driver translation via joystick inputs while automatically calculating and maintaining chassis rotation
 * towards the alliance Hub target center.
 */
public class CMD_AimBotAuto extends RunCommand {
        /** Subsystems and state variables used for targeting and control */
        private final SUB_PhotonVision photonVision;
        private final CommandSwerveDrivetrain drivetrain;
        private Pose2d targetPose = new Pose2d();
        private static boolean running;
        private final SUB_Index index;
        private final SUB_Hood hood;
        private final SUB_Metering metering;
        private final SUB_Shooter shooter;
        private boolean isLocked;

        /** Physical offsets for targeting calibration */
        Translation2d shooterOffset =
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        Rotation2d shooterThetaOffset =
            new Rotation2d(Units.degreesToRadians(0)); // CounterClockwise Positive

        /** Motion profiling constraints for rotation (1.6 rot/s max velocity, 12 rot/s^2 max acceleration). */
        private final TrapezoidProfile.Constraints thetaConstraints =
            new TrapezoidProfile.Constraints(RotationsPerSecond.of(1.6).in(RadiansPerSecond),
                RotationsPerSecond.of(12).in(RadiansPerSecond));

        /** Profiled PID controller for heading alignment (P=5.0, I=0.0, D=0.2). */
        private final ProfiledPIDController robotAngleController =
            new ProfiledPIDController(5.0, 0, 0.2,
                thetaConstraints);

        /** Status flag indicating whether heading error is within 5 degrees tolerance. */
        public static boolean isThetaErrorCorrect = false;

        private final SwerveRequest.SwerveDriveBrake brakeRequest =
            new SwerveRequest.SwerveDriveBrake();
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        /**
         * Constructs a new AimBot command.
         *
         * @param drivetrain The swerve drivetrain subsystem.
         * @param photonVision The vision subsystem for target tracking.
         * @param index The indexer subsystem.
         * @param hood The hood subsystem.
         * @param metering The metering subsystem.
         * @param translationXSupplier Supplier for X translation input (-1.0 to 1.0).
         * @param translationYSupplier Supplier for Y translation input (-1.0 to 1.0).
         */
        public CMD_AimBotAuto(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision,
            SUB_Index index, SUB_Hood hood, SUB_Metering metering, SUB_Shooter shooter) {
                super(() -> {});
                this.drivetrain = drivetrain;
                this.photonVision = photonVision;
                this.index = index;
                this.hood = hood;
                this.shooter = shooter;
                this.metering = metering;
                robotAngleController.enableContinuousInput(-Math.PI, Math.PI);

                addRequirements(drivetrain, metering, index, hood, shooter);
        }

        /**
         * Command initialization. Resolves target AprilTag pose based on alliance color, resets
         * heading PID controller, and sets high current limit shooting mode.
         */
        @Override
        public void initialize() {
                // Determine the correct target tag based on the current alliance
                robotAngleController.setTolerance(Units.degreesToRadians(0.0));

                Pose2d tagPose = (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)))
                    ? photonVision.at_field.getTagPose(10).orElse(new Pose3d()).toPose2d()
                    : photonVision.at_field.getTagPose(26).orElse(new Pose3d()).toPose2d();

                double hubOffsetX = DriverStation.getAlliance().equals(Optional.of(Alliance.Red))
                    ? Units.inchesToMeters(-23.5)
                    : Units.inchesToMeters(23.5);
                Translation2d hubCenterTranslation =
                    new Translation2d(tagPose.getX() + hubOffsetX, tagPose.getY());

                targetPose = new Pose2d(hubCenterTranslation, new Rotation2d());

                // Reset the PID controller to the current state of the robot
                robotAngleController.reset(drivetrain.getPose().getRotation().getRadians(),
                    drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
                isLocked = false;
                running = true;
                SUB_Shooter.isShooting = true;
        }

        /**
         * Command execution loop (20ms). Calculates angle to target, computes rotational velocity,
         * updates hood angle and metering speed, and feeds indexer when alignment error is within 5 degrees.
         */
        @Override
        public void execute() {
                // Set up poses
                Pose2d currentPose = drivetrain.getPose();

                Translation2d targetTranslation = targetPose.getTranslation();
                Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
                    shooterOffset.rotateBy(currentPose.getRotation()));

                // 2. Calculate the angle directly from the SHOOTER to the target
                Rotation2d targetRotation =
                    new Rotation2d(targetTranslation.getX() - shooterFieldPosition.getX(),
                        targetTranslation.getY() - shooterFieldPosition.getY());

                targetRotation = targetRotation.plus(shooterThetaOffset);

                // Update telemetry
                drivetrain.publisher1.set(new Pose2d(targetTranslation, targetRotation));

                // 3. Calculate rotational velocity (omega) using the PID controller
                double omegaSpeed = robotAngleController.calculate(
                    currentPose.getRotation().getRadians(), targetRotation.getRadians());

                // Calculate error for deadband checking
                double thetaErrorRads = Math.abs(MathUtil.angleModulus(
                    currentPose.getRotation().getRadians() - targetRotation.getRadians()));
                SmartDashboard.putNumber(
                    "CMD_AimBot/Theta Error (Deg)", Units.radiansToDegrees(thetaErrorRads));

                isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(5)
                    && Math.abs(
                           drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble())
                        <= 20;
                SmartDashboard.putBoolean("CMD_AimBot/isThetaErrorCorrect", isThetaErrorCorrect);
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
                double targetFlywheelRPM = shooter.getZonedRPM(distance);
                shooter.setRPM(targetFlywheelRPM);
                double exitVelocity = (Constants.Shooter.kSHOOTER_COMPRESSION_RATIO * Math.PI * Constants.Shooter.ShooterDiameter * targetFlywheelRPM)/(720  * 3.281);
                hood.setPosition(Units.radiansToDegrees(SUB_Hood.calculateLaunchAngle(distance,exitVelocity,true)));
                metering.setRPM(100);

                if (isThetaErrorCorrect && shooter.atDesiredRPM() && hood.atDesiredAngle()) {
                        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                } else {
                        index.setVolts(0);
                }


                // Wheel locking logic
                if (!isLocked && thetaErrorRads <= Units.degreesToRadians(1)) {
                        isLocked = true;
                } else if (isLocked && thetaErrorRads >= Units.degreesToRadians(5)) {
                        isLocked = false;
                }

                // Lock wheels or drive
                if (isThetaErrorCorrect && isLocked) {
                        drivetrain.setControl(brakeRequest);
                } else {
                        drivetrain.setControl(drive
                                .withRotationalRate(omegaSpeed * MaxAngularRate
                                    + Math.copySign(Units.degreesToRadians(9),
                                        omegaSpeed * MaxAngularRate)));
                }
        }

        /**
         * Clears running flags and disables high current limit shooting mode when ended or interrupted.
         *
         * @param interrupted True if command was interrupted.
         */
        @Override
        public void end(boolean interrupted) {
                running = false;
                SUB_Shooter.isShooting = false;
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
         * Returns whether the AimBot command is currently active.
         *
         * @return True if running, false otherwise.
         */
        public static boolean isRunning() {
                return running;
        }
}
