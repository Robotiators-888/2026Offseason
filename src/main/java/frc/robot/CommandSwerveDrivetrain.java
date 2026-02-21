package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Alert;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private static CommandSwerveDrivetrain INSTANCE = null;
    public static CommandSwerveDrivetrain getInstance () {
        if (INSTANCE == null) {
            INSTANCE = TunerConstants.createDrivetrain();
        }
        return INSTANCE;
    }

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.ApplyRobotSpeeds chassisSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts);
    private DutyCycleOut dutyCycle = new DutyCycleOut(1); // Idk what to put for the output

    public final StructPublisher<Pose3d> publisher1 = NetworkTableInstance.getDefault()
        .getStructTopic("debugXPoint", Pose3d.struct).publish(); 
        public final StructPublisher<Pose3d> publisher2 = NetworkTableInstance.getDefault()
        .getStructTopic("debugYPoint", Pose3d.struct).publish(); 
    public final StructPublisher<Pose3d> publisher3 = NetworkTableInstance.getDefault()
        .getStructTopic("PhotonCam1Pose", Pose3d.struct).publish(); 
    public final StructPublisher<Pose3d> publisher4 = NetworkTableInstance.getDefault()
        .getStructTopic("PhotonCam2Pose", Pose3d.struct).publish(); 
    public final StructPublisher<Pose3d> selectPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SelectedPose", Pose3d.struct).publish(); 

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private boolean reachedAutoTarget = false;
    private boolean intakeComplete = true;

    public void setReachedTarget(boolean value) {
        reachedAutoTarget = value;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("ReachedAutoTarget", reachedAutoTarget);
    }

    public boolean getReachedTarget() {
        return reachedAutoTarget;
    }

    public void setIntakeComplete(boolean value) {
        intakeComplete = value;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("IntakeComplete", intakeComplete);
    }

    public boolean getIntakeComplete() {
        return intakeComplete;
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        registerTelemetry(logger::telemeterize);
        enableFOC();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveConstants, modules);
        configurePathPlanner();
        registerTelemetry(logger::telemeterize);
        enableFOC();
    }

    private void enableFOC () {
        dutyCycle.EnableFOC = true;
        var modules = this.getModules();
        for (var module : modules) {
            TalonFX driveMotor = module.getDriveMotor();
            if (driveMotor.getIsProLicensed().getValue()) {
                driveMotor.setControl(dutyCycle);
            }
            else {
                Alert.notifyWarning("Drive motor not liscensed");
            }
        }
    }

    private void configurePathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            () -> this.getState().Pose, // Supplier of current robot pose
            this::resetPose,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds, feedforwards) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new PPHolonomicDriveController(
                new PIDConstants(10, 0, 0),
                new PIDConstants(10, 0, 0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return this.getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void zeroHeading() {
        this.resetRotation(new Rotation2d());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        // Ignoring rateLimit for now as CTRE handles it via config/requests usually, 
        // or we'd need a SlewRateLimiter here.
        if (fieldRelative) {
            this.setControl(driveRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rot));
        } else {
            this.setControl(robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rot));
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(chassisSpeedsRequest.withSpeeds(speeds));
    }


    
    // Original addVisionMeasurement is already in SwerveDrivetrain, but might need checking signature.
    // SwerveDrivetrain has addVisionMeasurement(Pose2d visionRobotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs)
    // and addVisionMeasurement(Pose2d visionRobotPose, double timestamp)
    
    // We don't need to add it if it matches.
    // However, SwerveDrivetrain uses double timestamp, SUB_Drivetrain used double timestamp.
    // Let's check imports for Matrix.

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change during a match */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == DriverStation.Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
