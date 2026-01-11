// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Swerve;
import frc.robot.utils.*;
// import org.littletonrobotics.junction.Logger;

public class SUB_Drivetrain extends SubsystemBase {

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("AdvantageScopeOdometry", Pose2d.struct).publish();

  public StructPublisher<Pose2d> publisher1 = NetworkTableInstance.getDefault()
  .getStructTopic("debugXPoint", Pose2d.struct).publish(); 

  public StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault()
  .getStructTopic("debugYPoint", Pose2d.struct).publish(); 


  public StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault()
  .getStructTopic("PhotonCam1Pose", Pose2d.struct).publish(); 


  public StructPublisher<Pose2d> publisher4 = NetworkTableInstance.getDefault()
  .getStructTopic("PhotonCam2Pose", Pose2d.struct).publish(); 

  public StructPublisher<Pose2d> selectPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("SelectedPose", Pose2d.struct).publish(); 


  StructArrayPublisher<SwerveModuleState> currentStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Current States", SwerveModuleState.struct).publish();

StructArrayPublisher<SwerveModuleState> desiredStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();

  public final Field2d m_field = new Field2d();
  private static SUB_Drivetrain INSTANCE = null;
  /** Creates a new Drivetrain. */

  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(Constants.Drivetrain.kFRONT_LEFT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kFRONT_LEFT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(Constants.Drivetrain.kFRONT_RIGHT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kFRONT_RIGHT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule backLeft =
      new MAXSwerveModule(Constants.Drivetrain.kBACK_LEFT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kBACK_LEFT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule backRight =
      new MAXSwerveModule(Constants.Drivetrain.kBACK_RIGHT_DRIVE_MOTOR_CANID,
          Constants.Drivetrain.kBACK_RIGHT_STEER_MOTOR_CANID,
          Constants.Drivetrain.kBackRightChassisAngularOffset);

  private MAXSwerveModule[] modules =
      new MAXSwerveModule[] {frontLeft, frontRight, backLeft, backRight};
  private SwerveModuleState[] moduleStates = getModuleStates();
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  public AprilTagFieldLayout at_field;

  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();;

  private final Pigeon2 pigeon2 = new Pigeon2(Drivetrain.kPigeon_CANID);

  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter =
      new SlewRateLimiter(Constants.Drivetrain.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter =
      new SlewRateLimiter(Constants.Drivetrain.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  Pose2d pose = new Pose2d();
  // Odometry class for tracking robot pose

  public SwerveDrivePoseEstimator m_poseEstimator;

  public static SUB_Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SUB_Drivetrain();
    }

    return INSTANCE;
  }

  private SUB_Drivetrain() {
    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.Drivetrain.kDriveKinematics,
    Rotation2d.fromDegrees(getAngle()),
    new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
        backLeft.getPosition(), backRight.getPosition()},
    new Pose2d(0, 0, new Rotation2d(0)));
    zeroHeading();
    
  }

  @Override
  public void periodic() {

    m_poseEstimator.update(Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()});
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    modules = new MAXSwerveModule[] {frontLeft, frontRight, backLeft, backRight};

    m_field.setRobotPose(getPose());

    m_fieldRelVel = new FieldRelativeSpeed(
        Constants.Drivetrain.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
            frontRight.getState(), backLeft.getState(), backRight.getState()),
        Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble())); //TODO: Confirm sign and CCW vs CW
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, 0.02);
    m_lastFieldRelVel = m_fieldRelVel;

    publisher.set(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumberArray("Drive/PoseEstimator",
        new double[] {m_poseEstimator.getEstimatedPosition().getX(),
            m_poseEstimator.getEstimatedPosition().getY(),
            m_poseEstimator.getEstimatedPosition().getRotation().getDegrees()});

    SmartDashboard.putData("Drive/Field", m_field);
    SmartDashboard.putNumberArray("Odometry",
        new double[] {getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});

    SmartDashboard.putNumber("Drive/Robot Pose X meters", (getPose().getX()));
    SmartDashboard.putNumber("Drive/Robot Pose Y meters", (getPose().getY()));
    SmartDashboard.putNumber("Drive/rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drive/Robot Speed", modules[0].getVelocityDrive());

    SmartDashboard.putNumber("BACK RIGHT MODULE POSITION", backRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("BACK LEFT MODULE POSITION", backLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FRONT LEFT MODULE POSITION", frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FRONT RIGHT MODULE POSITION",
        frontRight.getPosition().distanceMeters);



    currentStatePublisher.set(getModuleStates());

    SmartDashboard.putNumber("Pigeon2 angle", Units.degreesToRadians(getAngle())); //TODO: Confirm accuracy

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    Pose2d pose =  m_poseEstimator.getEstimatedPosition();
    return pose;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()},
        pose);

    this.pose = pose;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
      boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate =
            Math.abs(Constants.Drivetrain.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively
                                   // instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
            inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors
                                              // with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
            inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.Drivetrain.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.Drivetrain.kMaxAngularSpeed;

    // Adjust the heading to be within the range of -180 to 180 degrees
    var swerveModuleStates =
        Constants.Drivetrain.kDriveKinematics.toSwerveModuleStates(fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
        Constants.Drivetrain.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    desiredStatePublisher.set(swerveModuleStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setAngle(double angle) {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        Constants.Drivetrain.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
      pigeon2.setYaw(0); //TODO: Confirm this works as intended
      m_poseEstimator.resetRotation(Rotation2d.fromDegrees(pigeon2.getYaw().getValueAsDouble()));
  }


  public double getAngle() {
    return -pigeon2.getYaw().getValueAsDouble(); // TODO: Confirm negative sign and CCW vs CW
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return pigeon2.getAngularVelocityZWorld().getValueAsDouble() * (Constants.Drivetrain.kGyroReversed ? -1.0 : 1.0); //TODO: Confirm sign/ CCW vs CW
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }


  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

    for (int i = 0; i < moduleStates.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Drivetrain.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getRotation2d(), getPositions(), pose);

    this.pose = pose;
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
      robotRelativeSpeeds.vxMetersPerSecond,
      robotRelativeSpeeds.vyMetersPerSecond,
      robotRelativeSpeeds.omegaRadiansPerSecond //TODO: Unstable, should we remove?
    );
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates =
        Drivetrain.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void driveVelocity(ChassisSpeeds speeds) {
    setpoint.vxMetersPerSecond = speeds.vxMetersPerSecond;
    setpoint.vyMetersPerSecond = speeds.vyMetersPerSecond;
  }

  public void driveVelocity(double omega) {
    setpoint.omegaRadiansPerSecond = omega;
  }

  public void stop() {
    driveVelocity(new ChassisSpeeds());
  }

  public Command pidControlledHeading(Supplier<Optional<Rotation2d>> headingSupplier) {
    var subsystem = this;
    return new Command() {
        private final PIDController headingPID = new PIDController(Swerve.kDrivingP, Swerve.kDrivingI, Swerve.kDrivingD);
        {
            addRequirements(subsystem);
            setName("PID Controlled Heading");
            headingPID.enableContinuousInput(-Math.PI, Math.PI); // Enable continuous input
            headingPID.setTolerance(Swerve.headingTolerance);
        }
        private Rotation2d desiredHeading;
        private boolean headingSet;

        @Override
        public void initialize() {
            desiredHeading = getPose().getRotation();
        }

        @Override
        public void execute() {
            var heading = headingSupplier.get();
            headingSet = heading.isPresent();
            heading.ifPresent((r) -> desiredHeading = r);
            double turnInput = headingPID.calculate(getPose().getRotation().getRadians(), desiredHeading.getRadians());
            turnInput = headingPID.atSetpoint() ? 0 : turnInput;
            turnInput = MathUtil.clamp(turnInput, -0.5, +0.5);
            driveVelocity(turnInput * Swerve.kMaxRotationalSpeed);
        }

        @Override
        public void end(boolean interrupted) {
            stop();
        }

        @Override
        public boolean isFinished() {
            return !headingSet && headingPID.atSetpoint();
        }
    };
}

  public Command fieldRelative(Supplier<ChassisSpeeds> speeds) {
    var subsystem = this;
    return new Command() {
      {
        addRequirements(subsystem);
        setName("Field Relative");
      }

      @Override
      public void execute() {
        driveVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getPose().getRotation()));
      }

      @Override
      public void end(boolean interrupted) {
        stop();
      }
    };
  }

  public Command pointTo(Supplier<Optional<Translation2d>> posToPointTo,
      Supplier<Rotation2d> forward) {
    return pidControlledHeading(() -> posToPointTo.get().map((pointTo) -> {
      var FORR = pointTo.minus(getPose().getTranslation());
      return new Rotation2d(FORR.getX(), FORR.getY()).minus(forward.get());
    }));
  }

  /**
   * Allows for vision measurements to be added to drive odometry.
   * 
   * @param visionPose The pose supplied by getPose() in SUB_Limelight
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    // visionPose.rotateBy();
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  public FieldRelativeSpeed getFieldRelativeSpeed() {
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel() {
    return m_fieldRelAccel;
  }

}