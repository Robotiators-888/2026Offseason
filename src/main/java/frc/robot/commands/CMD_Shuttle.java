package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_PhotonVision;
import frc.robot.subsystems.SUB_Shooter;

/**
 * Command for shuttling game pieces across the field with velocity feedforward motion compensation.
 *
 * <p>Requires {@link SUB_PhotonVision}, {@link CommandSwerveDrivetrain}, {@link SUB_Index}, and
 * {@link SUB_Shooter}. Calculates a virtual target position compensating for robot translation
 * velocity and time-of-flight (TOF).
 */
public class CMD_Shuttle extends RunCommand {
        /** Physical offsets for targeting calibration */
        Translation2d shooterOffset =
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        /** Subsystems and state variables for shuttle targeting */
        private SUB_Index index;
        private SUB_Shooter shooter;
        private SUB_Hood hood;
        private SUB_Metering metering;
        private SUB_PhotonVision photonVision;
        private CommandSwerveDrivetrain drivetrain;

        private final SwerveRequest.SwerveDriveBrake brakeRequest =
            new SwerveRequest.SwerveDriveBrake();
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                .withRotationalDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withCenterOfRotation(shooterOffset);

        /**
         * Motion profiling constraints for rotation (0.75 rot/s max velocity, 1.5 rot/s^2 max
         * acceleration).
         */
        private final TrapezoidProfile.Constraints thetaConstraints =
            new TrapezoidProfile.Constraints(RotationsPerSecond.of(0.75).in(RadiansPerSecond),
                RotationsPerSecond.of(1.5).in(RadiansPerSecond));

        /**
         * Profiled PID controller for robot heading alignment during shuttle (P=5.0, I=0.0,
         * D=0.2).
         */
        private final ProfiledPIDController robotAngleController =
            new ProfiledPIDController(5.0, 0, 0.2, thetaConstraints);
        private Pose2d targetPose = new Pose2d();
        private boolean isThetaErrorCorrect;
        private boolean isLocked = false;

        /**
         * Constructs a new CMD_Shuttle command for long-range scoring or passing.
         *
         * @param drivetrain The swerve drivetrain subsystem.
         * @param photonVision The vision subsystem.
         * @param index The indexer subsystem.
         * @param shooter The shooter subsystem.
         * @param hood The hood subsystem.
         * @param metering The metering subsystem.
         */
        public CMD_Shuttle(CommandSwerveDrivetrain drivetrain, SUB_PhotonVision photonVision,
            SUB_Index index, SUB_Shooter shooter, SUB_Hood hood, SUB_Metering metering) {
                super(() -> {});
                this.index = index;
                this.shooter = shooter;
                this.hood = hood;
                this.metering = metering;
                this.photonVision = photonVision;
                this.drivetrain = drivetrain;
                robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
                isThetaErrorCorrect = false;
                addRequirements(photonVision, drivetrain, index, shooter, hood, metering);
        }

        /**
         * Command initialization. Resets heading PID controller.
         */
        @Override
        public void initialize() {
                isLocked = false;
                robotAngleController.setTolerance(Units.degreesToRadians(0.0));
                // Reset the PID controller to the current state of the robot
                robotAngleController.reset(drivetrain.getPose().getRotation().getRadians(),
                    drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
        }

        /**
         * Command execution loop (20ms). Computes virtual shuttle target using time-of-flight
         * compensation, aligns drivetrain heading towards virtual target, and feeds game piece when
         * rotational error is within 14 degrees.
         */
        @Override
        public void execute() {
                targetPose =
                    photonVision.at_field
                        .getTagPose(
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10
                                                                                              : 26)
                        .map(pose
                            -> pose.toPose2d().relativeTo(new Pose2d(
                                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                    ? -Units.inchesToMeters(100)
                                    : Units.inchesToMeters(100),
                                (drivetrain.getPose().getY() < 4) ? Units.inchesToMeters(100)
                                                                  : -Units.inchesToMeters(100),
                                Rotation2d.fromDegrees(0))))
                        .orElse(drivetrain.getPose());

                Pose2d currentPose = drivetrain.getPose();
                Translation2d shooterPosition = currentPose.getTranslation().plus(
                    shooterOffset.rotateBy(currentPose.getRotation()));

                Translation2d targetTranslation = targetPose.getTranslation();
                Rotation2d targetRotation =
                    new Rotation2d(shooterPosition.getX() - targetTranslation.getX(),
                        shooterPosition.getY() - targetTranslation.getY());

                double omegaSpeed = robotAngleController.calculate(
                    currentPose.getRotation().getRadians(), targetRotation.getRadians());

                double thetaErrorRads = Math.abs(MathUtil.angleModulus(
                    currentPose.getRotation().getRadians() - targetRotation.getRadians()));
                isThetaErrorCorrect = thetaErrorRads <= Units.degreesToRadians(14)
                    && Math.abs(
                           drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble())
                        <= 40;

                double distance = shooterPosition.getDistance(targetTranslation);
                double targetFlywheelRPM = shooter.getZonedRPM(distance);
                shooter.setRPM(targetFlywheelRPM);
                metering.setRPM(Constants.Metering.kMETERING_MOTOR_RPM);
                double exitVelocity = (Constants.Shooter.kSHOOTER_COMPRESSION_RATIO * Math.PI
                                          * Constants.Shooter.ShooterDiameter * targetFlywheelRPM)
                    / (720 * 3.281);
                hood.setPosition(Units.radiansToDegrees(
                    SUB_Hood.calculateLaunchAngle(distance, 0.0, exitVelocity, false)));

                boolean isShooterReady = shooter.atDesiredRPM();

                if (isThetaErrorCorrect && isShooterReady && hood.atDesiredAngle()) {
                        index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
                } else if (!isThetaErrorCorrect) {
                        index.setVolts(0);
                }

                if (!isLocked && thetaErrorRads <= Units.degreesToRadians(5)) {
                        isLocked = true;
                } else if (isLocked && thetaErrorRads >= Units.degreesToRadians(10)) {
                        isLocked = false;
                }

                if (isThetaErrorCorrect && isLocked) {
                        drivetrain.setControl(brakeRequest);
                } else {
                        drivetrain.setControl(drive.withRotationalRate(omegaSpeed * MaxAngularRate
                            + Math.copySign(
                                Units.degreesToRadians(9), omegaSpeed * MaxAngularRate)));
                }
        }

        @Override
        public void end(boolean interrupted) {
                index.setVolts(0);
                metering.setRPM(0);
        }
}
