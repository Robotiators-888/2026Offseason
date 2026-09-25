package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.metering.MeteringIO;
import frc.robot.subsystems.metering.MeteringIOHardware;
import frc.robot.subsystems.metering.MeteringIOInputsAutoLogged;
import frc.robot.subsystems.metering.MeteringIOSim;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the metering feed wheel system feeding game pieces into shooter flywheels.
 * Backed by AdvantageKit MeteringIO abstraction for hardware, simulation, and log replay.
 */
public class SUB_Metering extends SubsystemBase {
        private final MeteringIO io;
        private final MeteringIOInputsAutoLogged inputs = new MeteringIOInputsAutoLogged();

        private static SUB_Metering INSTANCE = null;

        /**
         * Singleton pattern provider for the metering subsystem.
         *
         * @return Single instance of {@link SUB_Metering}.
         */
        public static SUB_Metering getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Metering();
                }
                return INSTANCE;
        }

        public SUB_Metering(MeteringIO io) {
                this.io = io;
                INSTANCE = this;
        }

        public SUB_Metering() {
                this(createIO());
        }

        private static MeteringIO createIO() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                return new MeteringIOHardware();
                        case SIM:
                                return new MeteringIOSim();
                        case REPLAY:
                        default:
                                return new MeteringIO() {};
                }
        }

        /**
         * Sets open-loop percent output speed for the metering leader motor (-1.0 to 1.0).
         *
         * @param speed Percent duty cycle power.
         */
        @Deprecated
        public void set(final double speed) {
                io.setVoltage(speed * 12.0);
        }

        public void stop() {
                io.stop();
        }

        public void setRPM(final double rpm) {
                io.setRPM(rpm);
        }

        /**
         * Subsystem periodic loop (20ms).
         * Telemeters inputs through AdvantageKit and preserves SmartDashboard keys for backwards compatibility.
         */
        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs("Metering", inputs);

                SmartDashboard.putNumber("Metering/Leader Position", inputs.leaderPositionRotations);
                SmartDashboard.putNumber("Metering/Follower Position", inputs.followerPositionRotations);

                SmartDashboard.putNumber("Metering/Leader Stator Current", inputs.leaderStatorCurrentAmps);
                SmartDashboard.putNumber("Metering/Follower Stator Current", inputs.followerStatorCurrentAmps);

                SmartDashboard.putNumber("Metering/Leader Supply Current", inputs.leaderSupplyCurrentAmps);
                SmartDashboard.putNumber("Metering/Follower Supply Current", inputs.followerSupplyCurrentAmps);

                SmartDashboard.putNumber("Metering/Leader Motor Voltage", inputs.leaderAppliedVolts);
                SmartDashboard.putNumber("Metering/Follower Motor Voltage", inputs.followerAppliedVolts);

                SmartDashboard.putNumber("Metering/Leader Velocity", inputs.leaderVelocityRPM);
                SmartDashboard.putNumber("Metering/Follower Velocity", inputs.followerVelocityRPM);
        }
}
