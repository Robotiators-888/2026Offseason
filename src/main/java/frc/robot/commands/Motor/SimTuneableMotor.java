package frc.robot.commands.Motor;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * High-fidelity CTRE TalonFX simulated motor for testing {@link frc.robot.commands.CMD_VelocityTuner}.
 *
 * <p>Uses an actual CTRE {@link TalonFX} object paired with its official {@link TalonFXSimState}.
 * This means:
 * <ul>
 *   <li>The tuner talks to an actual TalonFX device using standard Phoenix 6 controls and CAN status signals.</li>
 *   <li>Firmware velocity filtering, CAN update frequencies (50Hz - 250Hz), and voltage requests behave identically to a physical motor.</li>
 *   <li>Physics (back-EMF, stator current limits, Coulomb friction, inertia) are integrated accurately on simulation ticks.</li>
 * </ul>
 */
public class SimTuneableMotor extends SubsystemBase implements TuneableMotor {
    // ---- Mechanism physics parameters ----
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);
    private static final double kGearing = 1.0;
    private static final double kMOI = 0.0005; // kg*m^2 (4-inch shooter wheel)
    private static final double kStaticFrictionVolts = 0.25; // true kS
    private static final double kViscousNmPerRadPerSec = 0.0002;
    private static final double kStatorCurrentLimit = 80.0; // amps

    // Real CTRE TalonFX device on arbitrary unused CAN ID (60)
    private final TalonFX talon = new TalonFX(60);
    private final TalonFXSimState talonSim = talon.getSimState();
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // Mechanical state
    private double mechanismRotations = 0.0;
    private double mechanismRPS = 0.0; // Rotations Per Second

    public SimTuneableMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(kStatorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60.0);
        talon.getConfigurator().apply(config);

        // Publish theoretical physics values for comparison
        double radPerSecPerRPM = 2.0 * Math.PI / 60.0;
        double backEmfTerm = kGearing / gearbox.KvRadPerSecPerVolt;
        double viscousTerm = kViscousNmPerRadPerSec * gearbox.rOhms / (gearbox.KtNMPerAmp * kGearing);
        double trueKv = (backEmfTerm + viscousTerm) * radPerSecPerRPM;
        SmartDashboard.putNumber("SimMotor/True kS", kStaticFrictionVolts);
        SmartDashboard.putNumber("SimMotor/True kV (V per RPM)", trueKv);
    }

    @Override
    public void set(double dutycycle) {
        talon.setControl(dutyCycleRequest.withOutput(dutycycle));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        talon.setControl(voltageRequest.withOutput(voltage.in(Volts)));
    }

    @Override
    public void setVoltage(double voltage) {
        talon.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public AngularVelocity getVelocity() {
        // Reads from actual Phoenix 6 firmware velocity status signal!
        return talon.getVelocity().getValue();
    }

    /**
     * WPILib subsystem periodic (runs every 20ms during robot loop).
     * Steps the physical mechanism and updates the CTRE TalonFXSimState.
     */
    @Override
    public void periodic() {
        updatePhysics(0.020);

        SmartDashboard.putNumber("SimMotor/Commanded Volts", talon.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("SimMotor/Velocity RPM", talon.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("SimMotor/Stator Current", talon.getStatorCurrent().getValueAsDouble());
    }

    /**
     * Sub-stepped physics integrator to model motor dynamics accurately without ODE instability.
     */
    private void updatePhysics(double dtTotal) {
        double batteryVoltage = RobotController.getBatteryVoltage();
        if (batteryVoltage < 1.0) batteryVoltage = 12.0;
        talonSim.setSupplyVoltage(batteryVoltage);

        // Read voltage applied by TalonFX controller
        double appliedVolts = talonSim.getMotorVoltage();

        final int substeps = 20; // 1 ms integration step
        final double dt = dtTotal / substeps;

        double omegaRadPerSec = mechanismRPS * 2.0 * Math.PI;

        for (int i = 0; i < substeps; i++) {
            double motorRadPerSec = omegaRadPerSec * kGearing;

            // Motor electrical circuit model: I = (V - backEMF) / R
            double current = (appliedVolts - (motorRadPerSec / gearbox.KvRadPerSecPerVolt)) / gearbox.rOhms;
            current = MathUtil.clamp(current, -kStatorCurrentLimit, kStatorCurrentLimit);

            double driveTorque = gearbox.KtNMPerAmp * current * kGearing;
            double staticFrictionTorque = (kStaticFrictionVolts / gearbox.rOhms) * gearbox.KtNMPerAmp * kGearing;
            double viscousTorque = kViscousNmPerRadPerSec * omegaRadPerSec;

            if (Math.abs(omegaRadPerSec) < 1e-3) {
                // Static friction breakaway
                if (Math.abs(driveTorque) <= staticFrictionTorque) {
                    omegaRadPerSec = 0.0;
                } else {
                    double netTorque = driveTorque - Math.copySign(staticFrictionTorque, driveTorque);
                    omegaRadPerSec += (netTorque / kMOI) * dt;
                }
            } else {
                double netTorque = driveTorque - Math.copySign(staticFrictionTorque, omegaRadPerSec) - viscousTorque;
                double newOmega = omegaRadPerSec + (netTorque / kMOI) * dt;

                // Friction cannot reverse direction
                if (Math.signum(newOmega) != Math.signum(omegaRadPerSec) && Math.abs(driveTorque) <= staticFrictionTorque) {
                    newOmega = 0.0;
                }
                omegaRadPerSec = newOmega;
            }

            mechanismRotations += (omegaRadPerSec / (2.0 * Math.PI)) * dt;
        }

        mechanismRPS = omegaRadPerSec / (2.0 * Math.PI);

        // Feed position, velocity, and current back into official CTRE TalonFX simulation registers
        talonSim.setRotorVelocity(mechanismRPS * kGearing);
        talonSim.setRawRotorPosition(mechanismRotations * kGearing);
    }

    public TalonFX getTalon() {
        return talon;
    }
}

