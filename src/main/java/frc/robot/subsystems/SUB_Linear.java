package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.linear.LinearIO;
import frc.robot.subsystems.linear.LinearIOHardware;
import frc.robot.subsystems.linear.LinearIOInputsAutoLogged;
import frc.robot.subsystems.linear.LinearIOSim;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling linear intake deployment with AdvantageKit IO abstraction.
 */
public class SUB_Linear extends SubsystemBase {
        public static boolean extended;
        private static SUB_Linear INSTANCE = null;

        private final LinearIO io;
        private final LinearIOInputsAutoLogged inputs = new LinearIOInputsAutoLogged();

        public static SUB_Linear getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Linear();
                }
                return INSTANCE;
        }

        public SUB_Linear(LinearIO io) {
                this.io = io;
                INSTANCE = this;
        }

        public SUB_Linear() {
                this(createIO());
        }

        private static LinearIO createIO() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                return new LinearIOHardware();
                        case SIM:
                                return new LinearIOSim();
                        case REPLAY:
                        default:
                                return new LinearIO() {};
                }
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs("Linear", inputs);

                SmartDashboard.putNumber("Linear/Linear Encoder Pos", inputs.positionRotations);
                SmartDashboard.putNumber("Linear/Linear Output Current", inputs.currentAmps);
                SmartDashboard.putNumber("Linear/Linear Bus Voltage", inputs.busVolts);
                SmartDashboard.putNumber("Linear/Linear Motor Temp", inputs.tempCelsius);
                SmartDashboard.putBoolean("Linear/IsForward", isForward());
                SmartDashboard.putBoolean("Linear/IsBackward", isBackward());
        }

        public boolean isExtended() {
                return extended;
        }

        public void forward() {
                setPosition(Constants.Linear.kLINEAR_FORWARD_SETPOINT);
        }

        public void setPosition(double position) {
                double output = Constants.Linear.kLINEAR_PID_CONTROLLER.calculate(
                    inputs.positionRotations, position);
                io.setDutyCycle(output);
        }

        public void backward() {
                setPosition(Constants.Linear.kLINEAR_BACKWARD_SETPOINT);
        }

        public boolean isForward() {
                return Math.abs(inputs.positionRotations - Constants.Linear.kLINEAR_FORWARD_SETPOINT)
                    < Constants.Linear.kTolerance;
        }

        public boolean isBackward() {
                return Math.abs(inputs.positionRotations - Constants.Linear.kLINEAR_BACKWARD_SETPOINT)
                    < Constants.Linear.kTolerance;
        }

        public void set(double speed) {
                io.setDutyCycle(speed);
        }

        public void stop() {
                io.stop();
        }
}
