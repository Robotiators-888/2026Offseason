package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

/**
 * Physics-based swerve drivetrain simulation implementation using IronMaple (MapleSim).
 * Features realistic contact physics, motor electrical simulation, and dynamic battery sag.
 */
public class DriveIOSim extends DriveIOHardware {
        private final SwerveDriveSimulation mapleSimDrive;
        private final Notifier simNotifier;

        private static SwerveModuleConstants<?, ?, ?>[] sanitizeConstantsForSim(
            SwerveModuleConstants<?, ?, ?>[] originalConstants) {
                SwerveModuleConstants<?, ?, ?>[] sanitized =
                    new SwerveModuleConstants<?, ?, ?>[originalConstants.length];
                for (int i = 0; i < originalConstants.length; i++) {
                        var mod = originalConstants[i];
                        sanitized[i] = mod.withEncoderOffset(0.0)
                                           .withDriveMotorInverted(false)
                                           .withSteerMotorInverted(false)
                                           .withEncoderInverted(false)
                                           .withSteerMotorGains(mod.SteerMotorGains.withKP(70.0).withKD(4.5))
                                           .withDriveFrictionVoltage(Volts.of(0.1))
                                           .withSteerFrictionVoltage(Volts.of(0.15))
                                           .withSteerInertia(KilogramSquareMeters.of(0.05));
                }
                return sanitized;
        }

        public DriveIOSim(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... moduleConstants) {
                super(drivetrainConstants, sanitizeConstantsForSim(moduleConstants));

                DriveTrainSimulationConfig simulationConfig =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
                        .withBumperSize(Inches.of(Constants.ROBOT_LENGTH_INCHES),
                            Inches.of(Constants.ROBOT_WIDTH_INCHES))
                        .withGyro(COTS.ofPigeon2())
                        .withTrackLengthTrackWidth(Inches.of(22.5), Inches.of(22.5))
                        .withSwerveModule(new SwerveModuleSimulationConfig(
                            DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1),
                            moduleConstants[0].DriveMotorGearRatio,
                            moduleConstants[0].SteerMotorGearRatio,
                            Volts.of(moduleConstants[0].DriveFrictionVoltage),
                            Volts.of(moduleConstants[0].SteerFrictionVoltage),
                            Meters.of(moduleConstants[0].WheelRadius),
                            KilogramSquareMeters.of(moduleConstants[0].SteerInertia), 1.2));

                Pose2d startingPose = new Pose2d(3.0, 3.0, new Rotation2d());
                this.mapleSimDrive = new SwerveDriveSimulation(simulationConfig, startingPose);

                SimulatedArena.overrideSimulationTimings(
                    Seconds.of(1.0 / Constants.ODOMETRY_LOOP_HZ), 1);
                SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);

                for (int i = 0; i < 4; i++) {
                        final var realModule = this.getModule(i);
                        final var simModule = mapleSimDrive.getModules()[i];

                        simModule.useDriveMotorController(new SimulatedMotorController() {
                                @Override
                                public Voltage updateControlSignal(Angle mechanismAngle,
                                    AngularVelocity mechanismVelocity, Angle encoderAngle,
                                    AngularVelocity encoderVelocity) {
                                        realModule.getDriveMotor().getSimState().setRawRotorPosition(
                                            encoderAngle);
                                        realModule.getDriveMotor().getSimState().setRotorVelocity(
                                            encoderVelocity);
                                        realModule.getDriveMotor().getSimState().setSupplyVoltage(
                                            SimulatedBattery.getBatteryVoltage());
                                        return realModule.getDriveMotor()
                                            .getSimState()
                                            .getMotorVoltageMeasure();
                                }
                        });

                        simModule.useSteerMotorController(new SimulatedMotorController() {
                                @Override
                                public Voltage updateControlSignal(Angle mechanismAngle,
                                    AngularVelocity mechanismVelocity, Angle encoderAngle,
                                    AngularVelocity encoderVelocity) {
                                        realModule.getSteerMotor().getSimState().setRawRotorPosition(
                                            encoderAngle);
                                        realModule.getSteerMotor().getSimState().setRotorVelocity(
                                            encoderVelocity);
                                        realModule.getSteerMotor().getSimState().setSupplyVoltage(
                                            SimulatedBattery.getBatteryVoltage());
                                        return realModule.getSteerMotor()
                                            .getSimState()
                                            .getMotorVoltageMeasure();
                                }
                        });
                }

                // Telemetry callback to reflect MapleSim physics pose
                Consumer<SwerveDriveState> simTelemetryConsumer = swerveDriveState -> {
                        swerveDriveState.Pose = mapleSimDrive.getSimulatedDriveTrainPose();
                        telemetryConsumer.accept(swerveDriveState);
                };
                registerTelemetry(simTelemetryConsumer);

                // Simulation physics loop running high-frequency
                this.simNotifier = new Notifier(() -> {
                        SimulatedArena.getInstance().simulationPeriodic();

                        getPigeon2().getSimState().setRawYaw(
                            mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
                        getPigeon2().getSimState().setAngularVelocityZ(RadiansPerSecond.of(
                            mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                                .omegaRadiansPerSecond));
                });
                simNotifier.startPeriodic(1.0 / Constants.ODOMETRY_LOOP_HZ);
        }

        public SwerveDriveSimulation getMapleSimDrive() {
                return mapleSimDrive;
        }

        @Override
        public void resetOdometry(Pose2d pose) {
                mapleSimDrive.setSimulationWorldPose(pose);
                super.resetOdometry(pose);
        }

        @Override
        public void updateInputs(DriveIOInputs inputs) {
                super.updateInputs(inputs);
                inputs.batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);

                Logger.recordOutput("Drive/SimulatedBatteryVoltage", inputs.batteryVoltage);
                Logger.recordOutput(
                    "Drive/SimulatedDrivetrainPose", mapleSimDrive.getSimulatedDriveTrainPose());
        }
}
