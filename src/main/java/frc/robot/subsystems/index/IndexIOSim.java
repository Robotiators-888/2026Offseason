package frc.robot.subsystems.index;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Physics simulation of spindexer feed mechanism using WPILib FlywheelSim.
 */
public class IndexIOSim implements IndexIO {
        private final FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.001, 1.0), DCMotor.getNEO(2));
        private double appliedVolts = 0.0;

        @Override
        public void updateInputs(IndexIOInputs inputs) {
                sim.setInputVoltage(appliedVolts);
                sim.update(0.02);

                double rpm = sim.getAngularVelocityRPM();
                inputs.rightVelocityRPM = rpm;
                inputs.leftVelocityRPM = rpm;
                inputs.rightAppliedVolts = appliedVolts;
                inputs.leftAppliedVolts = appliedVolts;
                inputs.rightCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
                inputs.leftCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
                inputs.rightBusVolts = 12.0;
                inputs.leftBusVolts = 12.0;
                inputs.rightTempCelsius = 25.0;
                inputs.leftTempCelsius = 25.0;
        }

        @Override
        public void setVoltage(double volts) {
                appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        }

        @Override
        public void setDutyCycle(double speed) {
                setVoltage(speed * 12.0);
        }

        @Override
        public void stop() {
                setVoltage(0.0);
        }
}
