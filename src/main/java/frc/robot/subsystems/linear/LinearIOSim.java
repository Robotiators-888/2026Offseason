package frc.robot.subsystems.linear;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Physics simulation of linear intake deploy mechanism using WPILib ElevatorSim.
 */
public class LinearIOSim implements LinearIO {
        private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getNEO(1), 23.0, 3.0, 0.015, 0.0, 0.3, false, 0.0);

        private double appliedVolts = 0.0;

        @Override
        public void updateInputs(LinearIOInputs inputs) {
                elevatorSim.setInputVoltage(appliedVolts);
                elevatorSim.update(0.02);

                // Convert linear position meters to rotations (approx 0.07m per rotation)
                double posMeters = elevatorSim.getPositionMeters();
                double posRotations = (posMeters / 0.3) * 4.2;

                inputs.positionRotations = posRotations;
                inputs.velocityRPM = (elevatorSim.getVelocityMetersPerSecond() / 0.3) * 4.2 * 60.0;
                inputs.appliedVolts = appliedVolts;
                inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
                inputs.busVolts = 12.0;
                inputs.tempCelsius = 25.0;
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
