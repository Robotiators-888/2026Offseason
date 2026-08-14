package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Operator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Index;
import frc.robot.subsystems.SUB_Metering;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.AimUtil;
import frc.robot.utils.FeedGate;
import frc.robot.utils.Hub;

public class CMD_Shuttle extends RunCommand{
    /** Physical offsets for targeting calibration */
    Translation2d shooterOffset = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
    /** Subsystems and state variables for shuttle targeting */
    private final SUB_Index index;
    private final SUB_Shooter shooter;
    private final SUB_Hood hood;
    private final SUB_Metering metering;
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withRotationalDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withCenterOfRotation(shooterOffset);

    /** Motion profiling constraints for rotation */
    private final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(
        RotationsPerSecond.of(0.75).in(RadiansPerSecond),
        RotationsPerSecond.of(1.5).in(RadiansPerSecond)
    );
    private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0);
    private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3.0,-8.0,0.0);

    /** PID controller for robot heading alignment during shuttle */
    private final ProfiledPIDController robotAngleController = new ProfiledPIDController(
        Constants.Shooter.kAIM_HEADING_kP, 0, Constants.Shooter.kAIM_HEADING_kD,
        thetaConstraints
    );
    private Pose2d targetPose = new Pose2d();
    private boolean isThetaErrorCorrect;

    /**
     * Which sideline the shuttle is lobbing toward, held with hysteresis by
     * {@link AimUtil#sidelineSign}. 0 means unset, so the first execute() picks the nearest.
     */
    private int lastSidelineSign = 0;

    /** Inside this error the heading output is zeroed so the robot can settle. */
    private static final double kHeadingToleranceRads = Constants.Shooter.kAIM_HEADING_TOLERANCE_RADS;

    /** Shuttling is a lob, so it tolerates far more heading error than an aimed shot. */
    private static final double kAlignedToleranceRads = Units.degreesToRadians(14);

    private static final double kMaxYawRateDegPerSec = 40;

    /** How far from the hub, along each axis, the shuttle aim point sits. */
    private static final double kShuttleOffsetMeters = Units.inchesToMeters(100);

    /** How far past the field centerline the robot must travel before the aim point swaps sides. */
    private static final double kSidelineHysteresisMeters = 0.5;

    /** Debounced, latching fire gate — see {@link FeedGate}. */
    private final FeedGate feedGate =
        new FeedGate(Constants.Shooter.kFEED_ARM_DEBOUNCE_SECONDS, Timer::getFPGATimestamp);

    /**
     * Constructs a new CMD_Shuttle command for long-range scoring or passing.
     * @param drivetrain The swerve drivetrain subsystem
     * @param index The indexer subsystem
     * @param shooter The shooter subsystem
     * @param hood The hood subsystem, which holds the lob angle
     * @param metering The metering wheel that feeds the shooter
     * @param translationXSupplier Supplier for X translation input
     * @param translationYSupplier Supplier for Y translation input
     */
    public CMD_Shuttle (CommandSwerveDrivetrain drivetrain, SUB_Index index, SUB_Shooter shooter, SUB_Hood hood, SUB_Metering metering, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        super(()->{});
        this.index = index;
        this.shooter = shooter;
        this.hood = hood;
        this.metering = metering;
        this.drivetrain = drivetrain;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        robotAngleController.enableContinuousInput(-Math.PI, Math.PI);
        isThetaErrorCorrect = false;
        // Hood and metering must be required: without them their default commands keep stowing
        // the hood and zeroing the feed for the whole shuttle, so no ball can ever leave.
        addRequirements(drivetrain, index, shooter, hood, metering);
    }

    @Override
    public void initialize () {
        robotAngleController.setTolerance(kHeadingToleranceRads);
        // Reset the PID controller to the current state of the robot
        robotAngleController.reset(
            drivetrain.getPose().getRotation().getRadians(),
            drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond
        );
        isThetaErrorCorrect = false;
        lastSidelineSign = 0;
        feedGate.reset();
        xSlewRateLimiter.reset(0.0);
        ySlewRateLimiter.reset(0.0);
        shooter.setHighPower(true);
    }

    @Override
    public void end (boolean interrupted) {
        isThetaErrorCorrect = false;
        index.setVolts(0);
        metering.set(0);
        shooter.setHighPower(false);
    }

    @Override
    public void execute () {
        Pose2d currentPose = drivetrain.getPose();

        // A shuttle is a lob onto the floor, so the hood holds a fixed lob angle instead of
        // running the hub-height ballistic solve.
        hood.setAngle(Constants.Shooter.kSHUTTLE_HOOD_ANGLE_RADS);

        lastSidelineSign = AimUtil.sidelineSign(
            currentPose.getY(), Constants.Field.fieldWidth, lastSidelineSign, kSidelineHysteresisMeters);
        final boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

        // Shuttle target: the hub, shifted toward our side of the field and toward whichever
        // sideline the robot is nearer. This used to use Pose2d.relativeTo(), which re-expresses
        // the tag in the offset's frame — with a zero-rotation offset that degenerates to a
        // subtraction, so the target moved the opposite way from what the signs implied.
        Optional<Translation2d> goal = Hub.getGoalTranslation()
            .map(hub -> hub.plus(AimUtil.shuttleOffset(isRed, lastSidelineSign, kShuttleOffsetMeters)));
        if (goal.isEmpty()) {
            isThetaErrorCorrect = false;
            index.setVolts(0);
            metering.set(0);
            return;
        }
        targetPose = new Pose2d(goal.get(), new Rotation2d());

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getCurrentRobotChassisSpeeds(),
            currentPose.getRotation()
        );

        Translation2d shooterFieldPosition = currentPose.getTranslation().plus(
            shooterOffset.rotateBy(currentPose.getRotation())
        );

        Translation2d virtualTargetTranslation = AimUtil.virtualTarget(
            targetPose.getTranslation(), shooterFieldPosition,
            fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond,
            shooter::getExpectedTOF);

        drivetrain.publisher2.set(new Pose2d(virtualTargetTranslation, new Rotation2d()));

        Translation2d targetTranslation = virtualTargetTranslation;

        // Calculate the required heading to face the shuttle target
        Rotation2d targetRotation = new Rotation2d(
            targetTranslation.getX() - shooterFieldPosition.getX(),
            targetTranslation.getY() - shooterFieldPosition.getY()
        );

        double omegaSpeed = robotAngleController.calculate(
            currentPose.getRotation().getRadians(),
            targetRotation.getRadians()
        );

        // Check if the rotation error is within an acceptable threshold for firing
        double thetaErrorRads = Math.abs(MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetRotation.getRadians()));
        isThetaErrorCorrect = thetaErrorRads <= kAlignedToleranceRads && Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) <= kMaxYawRateDegPerSec;

        double distance = shooterFieldPosition.getDistance(virtualTargetTranslation);
        // Latch the flywheel setpoint instead of chasing the table every loop — otherwise the
        // setpoint moves with odometry jitter and atDesiredRPM() never holds long enough to feed.
        shooter.setRPM(AimUtil.latchedSetpoint(
            shooter.getTargetRPM(),
            shooter.tunedRPM(distance),
            Constants.Shooter.kSHUTTLE_RPM_LATCH_DEADBAND));

        // Same debounced, latching gate as the aim commands: aligned, at speed, and hood at the
        // lob angle to arm; keep feeding through the RPM droop of a ball passing the wheels.
        final boolean armReady = isThetaErrorCorrect && shooter.atDesiredRPM() && hood.atAngle();
        final boolean sustainReady = isThetaErrorCorrect
            && Math.abs(shooter.rpmError()) < Constants.Shooter.kRPM_SUSTAIN_TOLERANCE;

        if (feedGate.update(armReady, sustainReady)) {
            index.setVolts(Constants.Index.kINDEX_MOTOR_VOLTS);
            metering.set(1);
        } else {
            index.setVolts(0);
            metering.set(0);
        }

        double xInput = xSlewRateLimiter.calculate(MathUtil.applyDeadband(translationXSupplier.getAsDouble(), Operator.kDriveDeadband));
        double yInput = ySlewRateLimiter.calculate(MathUtil.applyDeadband(translationYSupplier.getAsDouble(), Operator.kDriveDeadband));

        // Apply swerve drive request with PID-calculated rotation
        drivetrain.setControl(
        drive.withVelocityX(xInput * MaxSpeed)
        .withVelocityY(yInput * MaxSpeed)
        .withRotationalRate(AimUtil.rotationalRate(omegaSpeed, thetaErrorRads,
            MaxAngularRate, kHeadingToleranceRads,
            Constants.Shooter.kAIM_STICTION_RADS_PER_SEC,
            Constants.Shooter.kAIM_STICTION_RAMP_WIDTH_RADS)));

    }
}
