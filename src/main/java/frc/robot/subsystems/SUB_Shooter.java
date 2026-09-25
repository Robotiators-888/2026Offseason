package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the high-velocity dual-flywheel shooter mechanism with AdvantageKit IO abstraction.
 */
public class SUB_Shooter extends SubsystemBase {
        private static SUB_Shooter INSTANCE = null;

        private final ShooterIO io;
        private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

        private double desiredSpeed = 0;
        private final Debouncer atDesiredRPMDebouncer = new Debouncer(0.04);
        private final LoggedTunableNumber tunableRPM =
            new LoggedTunableNumber("Shooter/TunableRPM", Constants.Shooter.kSHOOTER_FLYWHEEL_RPM);

        public static boolean isShooting;
        public static boolean wasShooting = false;
        private double currentZoneRPM = RPMIdle;
        public static final double RPMZone1 = Constants.Shooter.kRPMZone1;
        public static final double RPMZone2 = Constants.Shooter.kRPMZone2;
        public static final double RPMZone3 = Constants.Shooter.kRPMZone3;
        public static final double RPMIdle = Constants.Shooter.kRPMIdle;

        public static SUB_Shooter getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Shooter();
                }
                return INSTANCE;
        }

        public SUB_Shooter(ShooterIO io) {
                this.io = io;
                INSTANCE = this;
        }

        public SUB_Shooter() {
                this(createIO());
        }

        private static ShooterIO createIO() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                return new ShooterIOHardware();
                        case SIM:
                                return new ShooterIOSim();
                        case REPLAY:
                        default:
                                return new ShooterIO() {};
                }
        }

        public void setRPM(final double rpm) {
                this.desiredSpeed = rpm;
                io.setVelocity(rpm);
        }

        public double flywheelRPM() {
                return (inputs.leaderVelocityRPM + inputs.followerVelocityRPM) / 2.0;
        }

        public boolean atDesiredRPM() {
                boolean inTolerance =
                    Math.abs(flywheelRPM() - desiredSpeed) < Constants.Shooter.kRPMTolerance;
                return atDesiredRPMDebouncer.calculate(inTolerance && desiredSpeed > 500);
        }

        public void stop() {
                this.desiredSpeed = 0;
                io.stop();
        }

        public void setVolts(final double volts) {
                io.setVoltage(volts);
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);

                SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
                SmartDashboard.putNumber("Shooter/FlywheelRPM (One)", inputs.leaderVelocityRPM);
                SmartDashboard.putNumber("Shooter/FlywheelRPM (Two)", inputs.followerVelocityRPM);
                SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());
                SmartDashboard.putBoolean("Shooter/AtDesiredRPM", atDesiredRPM());

                SmartDashboard.putNumber("Shooter/Leader Volts", inputs.leaderAppliedVolts);
                SmartDashboard.putNumber("Shooter/Follower Volts", inputs.followerAppliedVolts);
                SmartDashboard.putNumber("Shooter/Leader Current", inputs.leaderSupplyCurrentAmps);
                SmartDashboard.putNumber("Shooter/Follower Current", inputs.followerSupplyCurrentAmps);

                SUB_Shooter.wasShooting = SUB_Shooter.isShooting;
        }

        public double getInterpolatedRPM(double distanceMeters) {
                return Constants.Shooter.FLYWHEEL_RPM_MAP.get(distanceMeters);
        }

        public double getZonedRPM(double distanceMeters) {
                if (currentZoneRPM == RPMIdle) {
                        currentZoneRPM = (distanceMeters > Constants.Shooter.kZone3ThresholdMeters)
                            ? RPMZone3
                            : (distanceMeters > Constants.Shooter.kZone2InitialThresholdMeters)
                                ? RPMZone2
                                : RPMZone1;
                } else if (currentZoneRPM == RPMZone3
                    && distanceMeters < Constants.Shooter.kZone3To2HysteresisMeters) {
                        currentZoneRPM = RPMZone2;
                } else if (currentZoneRPM == RPMZone2) {
                        if (distanceMeters > Constants.Shooter.kZone2To3HysteresisMeters) {
                                currentZoneRPM = RPMZone3;
                        } else if (distanceMeters < Constants.Shooter.kZone2To1HysteresisMeters) {
                                currentZoneRPM = RPMZone1;
                        }
                } else if (currentZoneRPM == RPMZone1
                    && distanceMeters > Constants.Shooter.kZone1To2HysteresisMeters) {
                        currentZoneRPM = RPMZone2;
                }
                return currentZoneRPM;
        }
}
