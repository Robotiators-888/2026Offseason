package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.opencv.core.Mat.Tuple2;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSparkFlex;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkFlex = new SparkFlex(drivingCANId, SparkLowLevel.MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.

    m_drivingEncoder = m_drivingSparkFlex.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingClosedLoopController = m_drivingSparkFlex.getClosedLoopController();
    m_turningClosedLoopController = m_turningSparkMax.getClosedLoopController();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingSparkFlex.configure(SwerveModuleConfigs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(SwerveModuleConfigs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);


  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public void setPosition(double posMeters) {
    m_drivingEncoder.setPosition(posMeters);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    @SuppressWarnings("deprecation")
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond,
        SparkFlex.ControlType.kVelocity);
    m_turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(),
        SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getVelocityDrive() {
    return m_drivingEncoder.getVelocity();
  }

  public double getVelocitySteer() {
    return m_turningEncoder.getVelocity();
  }

  public Tuple2<SparkBase.Faults> getFaults () {
    return new Tuple2<SparkBase.Faults>(m_drivingSparkFlex.getFaults(), m_turningSparkMax.getFaults());
  }
  public Tuple2<SparkBase.Warnings> getWarnings() {
    return new Tuple2<SparkBase.Warnings>(m_drivingSparkFlex.getWarnings(), m_turningSparkMax.getWarnings());
  }
}
