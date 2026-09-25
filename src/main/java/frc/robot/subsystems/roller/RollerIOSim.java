package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Physics simulation of the intake roller mechanism using WPILib FlywheelSim.
 */
public class RollerIOSim implements RollerIO {
        private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), 0.001, Constants.Roller.kGearRatio),
            DCMotor.getKrakenX60(2));

        private final PIDController feedback = new PIDController(0.005, 0.0, 0.0);
        private double targetRPM = 0.0;
        private double appliedVolts = 0.0;
        private boolean isClosedLoop = false;

        @Override
        public void updateInputs(RollerIOInputs inputs) {
                if (isClosedLoop && targetRPM > 0) {
                        double ffVolts = (targetRPM / 6000.0) * 12.0;
                        double fbVolts = feedback.calculate(flywheelSim.getAngularVelocityRPM(), targetRPM);
                        appliedVolts = MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0);
                }

                flywheelSim.setInputVoltage(appliedVolts);
                flywheelSim.update(0.02);

                double currentRPM = flywheelSim.getAngularVelocityRPM();
                inputs.leftVelocityRPM = currentRPM;
                inputs.rightVelocityRPM = currentRPM;
                inputs.leftAppliedVolts = appliedVolts;
                inputs.rightAppliedVolts = appliedVolts;
                inputs.leftSupplyCurrentAmps = flywheelSim.getCurrentDrawAmps() / 2.0;
                inputs.rightSupplyCurrentAmps = flywheelSim.getCurrentDrawAmps() / 2.0;
                inputs.leftStatorCurrentAmps = inputs.leftSupplyCurrentAmps;
                inputs.rightStatorCurrentAmps = inputs.rightSupplyCurrentAmps;
                inputs.leftTempCelsius = 25.0;
                inputs.rightTempCelsius = 25.0;
        }

        @Override
        public void setRPM(double rpm) {
                this.targetRPM = rpm;
                this.isClosedLoop = true;
        }

        @Override
        public void setVoltage(double volts) {
                this.isClosedLoop = false;
                this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        }

        @Override
        public void stop() {
                this.isClosedLoop = false;
                this.targetRPM = 0.0;
                this.appliedVolts = 0.0;
        }
}
