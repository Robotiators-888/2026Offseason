package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

/**
 * Subsystem controlling the high-velocity dual-flywheel shooter mechanism.
 *
 * <p>Hardware:
 * <ul>
 *   <li>Leader TalonFX motor controller on CAN ID 43 ({@link Constants.Shooter#kSHOOTER_LEADER_MOTOR_CANID})</li>
 *   <li>Follower TalonFX motor controller on CAN ID 44 ({@link Constants.Shooter#kSHOOTER_FOLLOWER_MOTOR_CANID})</li>
 * </ul>
 */
public class SUB_Shooter extends SubsystemBase {
        private static SUB_Shooter INSTANCE = null;

        /** Subsystem hardware and control state */
        private final TalonFX shooterLeader;
        private final TalonFX shooterFollower;
        private final VoltageOut voltageRequest = new VoltageOut(0);
        // Not used since it is replaced by velocityRequest
        // private final VelocityVoltage m_request = new VelocityVoltage(0);
        private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
        private double desiredSpeed = 0;
        private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        private final TalonFXConfiguration shooterLowConfig = new TalonFXConfiguration();

        /** Active shooting state flag used for dynamic current limit switching. */
        public static boolean isShooting;
        public static boolean wasShooting = false;
        /** Interpolation map for distance-based RPM calibration (distance in meters -> RPM). */
        private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

        private double currentZoneRPM = RPMIdle;
        public static final double RPMIdle = 1100.0;
        public static final double RPMZone1 = 1350.0;
        public static final double RPMZone2 = 1750.0;
        public static final double RPMZone3 = 2400.0;


        /**
         * Singleton pattern provider for the shooter subsystem.
         *
         * @return Single instance of {@link SUB_Shooter}.
         */
        public static SUB_Shooter getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Shooter();
                }

                return INSTANCE;
        }

        /**
         * Private constructor initializing leader and follower TalonFX motor controllers,
         * populating the distance-to-RPM look-up table, and configuring PID/current limit settings.
         */
        private SUB_Shooter() {
                // Initialize dual flywheel motors
                shooterLeader = new TalonFX(Constants.Shooter.kSHOOTER_LEADER_MOTOR_CANID);
                shooterFollower = new TalonFX(Constants.Shooter.kSHOOTER_FOLLOWER_MOTOR_CANID);

                // Populate distance-to-RPM look-up table (meters -> RPM)
                distanceToRPM.put(2.49493587092, 1250.0);
                distanceToRPM.put(3.03308176613, 1375.0 + 15);
                distanceToRPM.put(1.6346195276, 1075.0 - 25);
                distanceToRPM.put(4.10526503, 1575.0 + 25);
                distanceToRPM.put(5.34766117, 1750.0 + 40);
                distanceToRPM.put(10.5, 2400.0);

                configFlywheel();
        }

        /**
         * Configures high and low current limit configurations and closed-loop PID gains.
         */
        private void configFlywheel() {
                // Configure current limits and neutral mode
                shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                shooterConfig.CurrentLimits.StatorCurrentLimit = 100;
                shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                shooterConfig.CurrentLimits.SupplyCurrentLimit = 60;
                shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
                shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
                shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                // Configure PID control loop coefficients
                shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
                shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV;
                shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
                shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP;
                shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
                shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD;

                shooterLowConfig.CurrentLimits.StatorCurrentLimitEnable = true;
                shooterLowConfig.CurrentLimits.StatorCurrentLimit = 100;
                shooterLowConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                shooterLowConfig.CurrentLimits.SupplyCurrentLimit = 10;
                shooterLowConfig.CurrentLimits.SupplyCurrentLowerLimit = 5;
                shooterLowConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
                shooterLowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                shooterLowConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                // Configure PID control loop coefficients
                shooterLowConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
                shooterLowConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV;
                shooterLowConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
                shooterLowConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP;
                shooterLowConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
                shooterLowConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD;

                shooterLeader.getConfigurator().apply(shooterConfig);
                shooterFollower.getConfigurator().apply(shooterConfig);

                // Synchronize bottom flywheel to top flywheel
                shooterFollower.setControl(
                    new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
        }

        /**
         * Calculates required flywheel exit RPM using kinematic projectile equations based on distance and hood angle.
         *
         * @param distance Distance to target in meters.
         * @param angle Hood launch angle in radians.
         * @return Calculated target exit velocity in RPM.
         */
        public static double findoptimalRPM(final double distance, final double angle) {
                double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
                double exitvelocity = (1 / Math.cos(angle))
                    * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance)
                        / (2 * (distance * Math.tan(angle) - height)));
                double exitRPM = ((720 / Constants.Shooter.ShooterDiameter) * exitvelocity * 3.281)
                    / (Constants.Shooter.kSHOOTER_COMPRESSION_RATIO * Math.PI);
                return exitRPM;
        }

        /**
         * Deprecated open-loop set duty cycle method.
         *
         * @param speed Percent output duty cycle.
         * @deprecated Use {@link #setRPM(double)} for closed-loop velocity control.
         */
        @Deprecated
        public void set(final double speed) {
                shooterLeader.set(speed);
        }

        /**
         * Sets closed-loop target velocity for flywheel motors in RPM.
         *
         * @param rpm Target velocity for both flywheels in RPM.
         */
        public void setRPM(final double rpm) {
                this.desiredSpeed = rpm;
                shooterLeader.setControl(velocityRequest.withVelocity(AngularVelocity.ofRelativeUnits(rpm, RPM)));
        }

        /**
         * Calculates current average velocity of leader and follower flywheels in RPM.
         *
         * @return Average velocity of both flywheels in RPM.
         */
        public double flywheelRPM() {
                return (shooterLeader.getVelocity().getValue().in(RPM)
                           + shooterFollower.getVelocity().getValue().in(RPM))
                    / 2;
        }

        /**
         * Checks whether actual flywheel RPM is within tolerance (75 RPM) of target speed.
         *
         * @return True if flywheels are at target RPM, false otherwise.
         */
        public boolean atDesiredRPM() {
                return Math.abs(flywheelRPM() - desiredSpeed) < 75;
        }

        /**
         * Sets target RPM based on distance interpolation table lookup.
         *
         * @param meters Distance to target in meters.
         */
        public void shootMeters(final double meters) {
                double targetRPM = distanceToRPM.get(meters);
                setRPM(targetRPM);
        }

        /**
         * Returns required RPM from interpolation table for a given distance in meters.
         *
         * @param meters Distance to target in meters.
         * @return Interpolated target speed in RPM.
         */
        public double getDistanceRPM(final double meters) {
                return distanceToRPM.get(meters);
        }

        /**
         * Stops flywheel motors by outputting 0 volts.
         */
        public void stop() {
                this.desiredSpeed = 0;
                shooterLeader.setControl(voltageRequest.withOutput(0));
        }

        /**
         * Direct voltage output override for testing.
         *
         * @param volts Target voltage output in volts.
         */
        @Deprecated
        public void setVolts(final double volts) {
                shooterLeader.setControl(voltageRequest.withOutput(volts));
        }

        /**
         * Subsystem periodic loop (20ms). Telemeters flywheel RPM, motor current, supply/motor voltages,
         * temperatures, and encoder values for both motors to SmartDashboard, dynamically updating current limit profiles.
         */
        @Override
        public void periodic() {
                // Telemetry logging for dashboard and diagnostics
                SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
                SmartDashboard.putNumber("Shooter/Motor One Stator Current",
                    shooterLeader.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Stator Current",
                    shooterFollower.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor One Supply Current",
                    shooterLeader.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Supply Current",
                    shooterFollower.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor One Supply Voltage",
                    shooterLeader.getSupplyVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Supply Voltage",
                    shooterFollower.getSupplyVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor One Voltage",
                    shooterLeader.getMotorVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Voltage",
                    shooterFollower.getMotorVoltage().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor One Encoder Pos",
                    shooterLeader.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Encoder Pos",
                    shooterFollower.getPosition().getValueAsDouble());

                SmartDashboard.putNumber("Shooter/Motor One Torque Current",
                    shooterLeader.getTorqueCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Torque Current",
                    shooterFollower.getTorqueCurrent().getValueAsDouble());

                SmartDashboard.putNumber("Shooter/Motor One Device Temp",
                    shooterLeader.getDeviceTemp().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Device Temp",
                    shooterFollower.getDeviceTemp().getValueAsDouble());

                SmartDashboard.putNumber("Shooter/Motor One Processor Temp",
                    shooterLeader.getProcessorTemp().getValueAsDouble());
                SmartDashboard.putNumber("Shooter/Motor Two Processor Temp",
                    shooterFollower.getProcessorTemp().getValueAsDouble());

                SmartDashboard.putNumber(
                    "Shooter/FlywheelRPM (One)", shooterLeader.getVelocity().getValue().in(RPM));
                SmartDashboard.putNumber(
                    "Shooter/FlywheelRPM (Two)", shooterFollower.getVelocity().getValue().in(RPM));

                SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());

                Alert.alertKraken(shooterLeader);
                Alert.alertKraken(shooterFollower);

                if (SUB_Shooter.isShooting && !SUB_Shooter.wasShooting) {
                        shooterLeader.getConfigurator().apply(shooterConfig);
                        shooterFollower.getConfigurator().apply(shooterConfig);
                } else if (!SUB_Shooter.isShooting && SUB_Shooter.wasShooting) {
                        shooterLeader.getConfigurator().apply(shooterLowConfig);
                        shooterFollower.getConfigurator().apply(shooterLowConfig);
                }

                SUB_Shooter.wasShooting = SUB_Shooter.isShooting;
        }

        /**
         * Calculates expected game piece time-of-flight (TOF) in seconds based on linear distance curve.
         *
         * @param distanceMeters Distance to target in meters.
         * @return Estimated time of flight in seconds.
         */
        public double getExpectedTOF(final double distanceMeters) {
                return distanceMeters * 0.215298795 + 0.753755412;
        }

        public double getZonedRPM(double distanceMeters) {
            if (distanceMeters > 6.0) {
                currentZoneRPM = RPMZone3;
            } else if (currentZoneRPM == RPMIdle || currentZoneRPM == RPMZone3) {
                currentZoneRPM = (distanceMeters > 3.2) ? RPMZone2 : RPMZone1;
            } else if (currentZoneRPM == RPMZone1 && distanceMeters > 3.35) {
                    currentZoneRPM = RPMZone2;
            } else if (currentZoneRPM == RPMZone2 && distanceMeters < 3.05) {
                currentZoneRPM = RPMZone1;
            }
            return currentZoneRPM;
        }
}
