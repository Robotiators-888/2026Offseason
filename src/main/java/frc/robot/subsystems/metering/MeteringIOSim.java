package frc.robot.subsystems.metering;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Physics-based simulation of the metering system using WPILib FlywheelSim.
 */
public class MeteringIOSim implements MeteringIO {
        private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX44Foc(2), 0.002, Constants.Metering.kGearRatio),
            DCMotor.getKrakenX44Foc(2));

        private final PIDController feedback = new PIDController(0.003, 0.0, 0.0);
        private double targetRPM = 0.0;
        private double appliedVolts = 0.0;
        private boolean isClosedLoop = false;
        private double simulatedRotations = 0.0;

        @Override
        public void updateInputs(MeteringIOInputs inputs) {
                if (isClosedLoop && Math.abs(targetRPM) > 0.0) {
                        double ffVolts = (targetRPM / 6000.0) * 12.0;
                        double fbVolts = feedback.calculate(flywheelSim.getAngularVelocityRPM(), targetRPM);
                        appliedVolts = MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0);
                }

                flywheelSim.setInputVoltage(appliedVolts);
                flywheelSim.update(0.02);

                double currentRPM = flywheelSim.getAngularVelocityRPM();
                simulatedRotations += (currentRPM / 60.0) * 0.02;

                inputs.leaderVelocityRPM = currentRPM;
                inputs.followerVelocityRPM = currentRPM;
                inputs.leaderPositionRotations = simulatedRotations;
                inputs.followerPositionRotations = simulatedRotations;
                inputs.leaderAppliedVolts = appliedVolts;
                inputs.followerAppliedVolts = appliedVolts;
                inputs.leaderSupplyCurrentAmps = flywheelSim.getCurrentDrawAmps() / 2.0;
                inputs.followerSupplyCurrentAmps = flywheelSim.getCurrentDrawAmps() / 2.0;
                inputs.leaderStatorCurrentAmps = inputs.leaderSupplyCurrentAmps;
                inputs.followerStatorCurrentAmps = inputs.followerSupplyCurrentAmps;
                inputs.leaderTempCelsius = 25.0;
                inputs.followerTempCelsius = 25.0;
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
