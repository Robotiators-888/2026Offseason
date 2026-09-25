package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOHardware;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOSim;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the ground intake roller mechanism with AdvantageKit IO abstraction.
 */
public class SUB_Roller extends SubsystemBase {
        private static SUB_Roller INSTANCE = null;
        private final RollerIO io;
        private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

        public static SUB_Roller getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Roller();
                }
                return INSTANCE;
        }

        public SUB_Roller(RollerIO io) {
                this.io = io;
                INSTANCE = this;
        }

        public SUB_Roller() {
                this(createIO());
        }

        private static RollerIO createIO() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                return new RollerIOHardware();
                        case SIM:
                                return new RollerIOSim();
                        case REPLAY:
                        default:
                                return new RollerIO() {};
                }
        }

        public void setVolts(final double volts) {
                io.setVoltage(volts);
        }

        public void setRPM(final double rpm) {
                io.setRPM(rpm);
        }

        public void set(final double speed) {
                io.setVoltage(speed * 12.0);
        }

        public void stop() {
                io.stop();
        }

        public double rollerRPM() {
                return (inputs.leftVelocityRPM + Math.abs(inputs.rightVelocityRPM)) / 2.0;
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs("Roller", inputs);

                SmartDashboard.putNumber("Roller/Roller Average RPM", rollerRPM());
                SmartDashboard.putNumber("Roller/Left Applied Volts", inputs.leftAppliedVolts);
                SmartDashboard.putNumber("Roller/Right Applied Volts", inputs.rightAppliedVolts);
                SmartDashboard.putNumber("Roller/Left Supply Current", inputs.leftSupplyCurrentAmps);
                SmartDashboard.putNumber("Roller/Right Supply Current", inputs.rightSupplyCurrentAmps);
        }
}
