package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Physics-based simulation of the flywheel shooter using WPILib FlywheelSim.
 */
public class ShooterIOSim implements ShooterIO {
        private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), 0.003, Constants.Shooter.kGearRatio),
            DCMotor.getKrakenX60(2));

        private final PIDController feedback = new PIDController(0.002, 0.0, 0.0);
        private double targetRPM = 0.0;
        private double appliedVolts = 0.0;
        private boolean isClosedLoop = false;

        @Override
        public void updateInputs(ShooterIOInputs inputs) {
                if (isClosedLoop && targetRPM > 0) {
                        double ffVolts = (targetRPM / 6000.0) * 12.0;
                        double fbVolts = feedback.calculate(flywheelSim.getAngularVelocityRPM(), targetRPM);
                        appliedVolts = MathUtil.clamp(ffVolts + fbVolts, -12.0, 12.0);
                }

                flywheelSim.setInputVoltage(appliedVolts);
                flywheelSim.update(0.02);

                double currentRPM = flywheelSim.getAngularVelocityRPM();
                inputs.leaderVelocityRPM = currentRPM;
                inputs.followerVelocityRPM = currentRPM;
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
        public void setVelocity(double targetRPM) {
                this.targetRPM = targetRPM;
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
