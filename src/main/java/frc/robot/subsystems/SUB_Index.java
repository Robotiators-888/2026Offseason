package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.index.IndexIO;
import frc.robot.subsystems.index.IndexIOHardware;
import frc.robot.subsystems.index.IndexIOInputsAutoLogged;
import frc.robot.subsystems.index.IndexIOSim;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the spindexer and feeder mechanism with AdvantageKit IO abstraction.
 */
public class SUB_Index extends SubsystemBase {
        private static SUB_Index INSTANCE = null;

        private final IndexIO io;
        private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

        public static SUB_Index getInstance() {
                if (INSTANCE == null) {
                        INSTANCE = new SUB_Index();
                }
                return INSTANCE;
        }

        public SUB_Index(IndexIO io) {
                this.io = io;
                INSTANCE = this;
        }

        public SUB_Index() {
                this(createIO());
        }

        private static IndexIO createIO() {
                switch (Constants.CURRENT_MODE) {
                        case REAL:
                                return new IndexIOHardware();
                        case SIM:
                                return new IndexIOSim();
                        case REPLAY:
                        default:
                                return new IndexIO() {};
                }
        }

        public void set(double speed) {
                io.setDutyCycle(speed);
        }

        public double indexRPM() {
                return (inputs.rightVelocityRPM + Math.abs(inputs.leftVelocityRPM)) / 2.0;
        }

        public void setVolts(double volts) {
                io.setVoltage(volts);
        }

        public void stop() {
                io.stop();
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs("Index", inputs);

                SmartDashboard.putNumber("Index/Index Average RPM", indexRPM());
                SmartDashboard.putNumber("Index/Right Output Current", inputs.rightCurrentAmps);
                SmartDashboard.putNumber("Index/Left Output Current", inputs.leftCurrentAmps);
                SmartDashboard.putNumber("Index/Right Bus Voltage", inputs.rightBusVolts);
                SmartDashboard.putNumber("Index/Left Bus Voltage", inputs.leftBusVolts);
        }
}
