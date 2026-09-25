package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * Physics-based simulation of the adjustable hood mechanism using WPILib SingleJointedArmSim.
 */
public class HoodIOSim implements HoodIO {
        private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            Constants.Hood.kGearRatio,
            0.005,
            0.2,
            0.0,
            Units.degreesToRadians(Constants.Hood.kMaxAngle),
            false,
            0.0
        );

        private final PIDController feedback = new PIDController(15.0, 0.0, 0.5);
        private double targetDegrees = 0.0;
        private double appliedVolts = 0.0;
        private boolean isClosedLoop = true;

        @Override
        public void updateInputs(HoodIOInputs inputs) {
                if (isClosedLoop) {
                        double currentRad = armSim.getAngleRads();
                        double targetRad = Units.degreesToRadians(targetDegrees);
                        appliedVolts = MathUtil.clamp(feedback.calculate(currentRad, targetRad), -12.0, 12.0);
                }

                armSim.setInputVoltage(appliedVolts);
                armSim.update(0.02);

                double currentDegrees = Units.radiansToDegrees(armSim.getAngleRads());
                inputs.positionDegrees = currentDegrees;
                inputs.positionRotations = currentDegrees / 360.0;
                inputs.velocityDegreesPerSec = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
                inputs.appliedVolts = appliedVolts;
                inputs.supplyCurrentAmps = armSim.getCurrentDrawAmps();
                inputs.statorCurrentAmps = armSim.getCurrentDrawAmps();
                inputs.tempCelsius = 25.0;
        }

        @Override
        public void setPositionDegrees(double degrees) {
                this.isClosedLoop = true;
                this.targetDegrees = MathUtil.clamp(degrees, Constants.Hood.kMinAngle, Constants.Hood.kMaxAngle);
        }

        @Override
        public void setVoltage(double volts) {
                this.isClosedLoop = false;
                this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        }

        @Override
        public void resetPosition(double positionRotations) {
                this.armSim.setState(positionRotations * 2.0 * Math.PI, 0.0);
        }

        @Override
        public void stop() {
                this.isClosedLoop = false;
                this.appliedVolts = 0.0;
        }
}
