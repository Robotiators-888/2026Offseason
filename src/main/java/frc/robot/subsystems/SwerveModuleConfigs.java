package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.Swerve;

public final class SwerveModuleConfigs {
        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward
                        // gain.
                        double drivingFactor = Swerve.kWheelDiameterMeters * Math.PI
                                        / Swerve.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double drivingVelocityFeedForward = 1 / Swerve.kDriveWheelFreeSpeedRps;

                        drivingConfig.idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(Swerve.kDrivingMotorCurrentLimit);
                        drivingConfig.encoder.positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters
                                                                                         // per
                                                                                         // second
                        drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own
                                        // robot!
                                        .pid(Swerve.kDrivingP, Swerve.kDrivingI, Swerve.kDrivingD)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(Swerve.kDrivingMinOutput,
                                                        Swerve.kDrivingMaxOutput);

                        turningConfig.idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(Swerve.kTurningMotorCurrentLimit);
                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft
                                        // rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(Swerve.kTurningEncoderInverted)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians
                                                                                         // per
                                                                                         // second
                        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // These are example gains you may need to them for your own
                                        // robot!
                                        .pid(Swerve.kTurningP, Swerve.kTurningI, Swerve.kTurningD)
                                        .outputRange(Swerve.kTurningMinOutput,
                                                        Swerve.kTurningMaxOutput)
                                        // Enable PID wrap around for the turning motor. This will
                                        // allow the PID
                                        // controller to go through 0 to get to the setpoint i.e.
                                        // going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other
                                        // direction which is a
                                        // longer route.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
