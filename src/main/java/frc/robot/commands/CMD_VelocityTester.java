package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Motor.TuneableMotor;

import static edu.wpi.first.units.Units.RPM;

/**
 * Temporary validation command that commands a {@link TuneableMotor} to random velocity setpoints
 * every two seconds to test closed-loop disturbance recovery, tracking accuracy, and damping.
 *
 * <p>Can automatically retrieve tuned feedforward ({@code kS}, {@code kV}) and feedback ({@code kP},
 * {@code kI}, {@code kD}) parameters directly from an executed {@link CMD_VelocityTuner} instance,
 * or accept explicit gains. Runs continuously until the command is cancelled or interrupted.
 */
public class CMD_VelocityTester extends Command {

    /** The motor or mechanism under test. */
    private final TuneableMotor motor;

    /** Optional reference to the preceding velocity tuner command. */
    private final CMD_VelocityTuner tuner;

    /** Timer used to track elapsed time between random setpoint changes. */
    private final Timer timer = new Timer();

    /** Pseudo-random number generator for velocity setpoint generation. */
    private final Random random = new Random();

    /**
     * Proportional feedback gain.
     * <p><b>Unit:</b> Volts per RPM error (V / RPM)
     */
    private double kP = 0.0;

    /**
     * Integral feedback gain.
     * <p><b>Unit:</b> Volts per (RPM * s)
     */
    private double kI = 0.0;

    /**
     * Derivative feedback gain.
     * <p><b>Unit:</b> Volts per (RPM / s)
     */
    private double kD = 0.0;

    /**
     * Static friction feedforward voltage.
     * <p><b>Unit:</b> Volts (V)
     */
    private double kS = 0.0;

    /**
     * Velocity feedforward gain.
     * <p><b>Unit:</b> Volts per RPM (V / RPM)
     */
    private double kV = 0.0;

    /**
     * Active commanded target velocity.
     * <p><b>Unit:</b> Rotations Per Minute (RPM)
     */
    private double targetRPM = 0.0;

    /**
     * Lower bound for random velocity setpoints.
     * <p><b>Unit:</b> Rotations Per Minute (RPM)
     */
    private final double minRPM;

    /**
     * Upper bound for random velocity setpoints.
     * <p><b>Unit:</b> Rotations Per Minute (RPM)
     */
    private final double maxRPM;

    /** WPILib PID feedback controller. */
    private PIDController pidController;

    /**
     * Constructs a velocity tester that retrieves tuned gains directly from a {@link CMD_VelocityTuner}.
     *
     * @param motor The {@link TuneableMotor} mechanism to drive.
     * @param tuner The {@link CMD_VelocityTuner} instance from which to read {@code kS}, {@code kV}, {@code kP}, {@code kI}, and {@code kD}.
     * @param minRPM Minimum random target velocity in RPM (e.g. 1500.0 RPM).
     * @param maxRPM Maximum random target velocity in RPM (e.g. 5000.0 RPM).
     */
    public CMD_VelocityTester(TuneableMotor motor, CMD_VelocityTuner tuner, double minRPM, double maxRPM) {
        this.motor = motor;
        this.tuner = tuner;
        this.minRPM = minRPM;
        this.maxRPM = maxRPM;
    }

    /**
     * Constructs a velocity tester that retrieves tuned gains directly from a {@link CMD_VelocityTuner}
     * with standard default velocity range between 1500.0 RPM and 5000.0 RPM.
     *
     * @param motor The {@link TuneableMotor} mechanism to drive.
     * @param tuner The {@link CMD_VelocityTuner} instance from which to read {@code kS}, {@code kV}, {@code kP}, {@code kI}, and {@code kD}.
     */
    public CMD_VelocityTester(TuneableMotor motor, CMD_VelocityTuner tuner) {
        this(motor, tuner, 1500.0, 5000.0);
    }

    /**
     * Constructs a velocity tester with explicit feedforward and full PID feedback gains.
     *
     * @param motor The {@link TuneableMotor} mechanism to drive.
     * @param kS Static friction feedforward in Volts (V).
     * @param kV Velocity feedforward gain in Volts per RPM (V / RPM).
     * @param kP Proportional feedback gain in Volts per RPM error (V / RPM).
     * @param kI Integral feedback gain in Volts per (RPM * s).
     * @param kD Derivative feedback gain in Volts per (RPM / s).
     * @param minRPM Minimum random target velocity in RPM.
     * @param maxRPM Maximum random target velocity in RPM.
     */
    public CMD_VelocityTester(TuneableMotor motor, double kS, double kV, double kP, double kI, double kD, double minRPM, double maxRPM) {
        this.motor = motor;
        this.tuner = null;
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.minRPM = minRPM;
        this.maxRPM = maxRPM;
    }

    /**
     * Constructs a velocity tester with explicit feedforward and feedback gains.
     *
     * @param motor The {@link TuneableMotor} mechanism to drive.
     * @param kS Static friction feedforward in Volts (V).
     * @param kV Velocity feedforward gain in Volts per RPM (V / RPM).
     * @param kP Proportional feedback gain in Volts per RPM error (V / RPM).
     */
    public CMD_VelocityTester(TuneableMotor motor, double kS, double kV, double kP) {
        this(motor, kS, kV, kP, 0.0, 0.0, 1500.0, 5000.0);
    }

    /**
     * Initializes the tester, extracting tuned constants from the tuner (if supplied), initializing
     * the PID controller, resetting the step timer, and choosing the first random target velocity.
     */
    @Override
    public void initialize() {
        if (tuner != null) {
            this.kS = tuner.kS;
            this.kV = tuner.kV;
            this.kP = tuner.kP;
            this.kI = tuner.kI;
            this.kD = tuner.kD;
        }

        pidController = new PIDController(kP, kI, kD);

        pickNewTargetRPM();
        timer.reset();
        timer.start();

        System.out.println("=== Velocity Tester Initialized ===");
        System.out.println("kS: " + kS + " Volts");
        System.out.println("kV: " + kV + " Volts/RPM");
        System.out.println("kP: " + kP + " Volts/RPM");
        System.out.println("kI: " + kI + " Volts/(RPM*s)");
        System.out.println("kD: " + kD + " Volts/(RPM/s)");
        System.out.println("Starting Target: " + targetRPM + " RPM");
    }

    /**
     * Executes the control loop every 20ms: updates target setpoint every 2.0 seconds,
     * computes feedforward and PID voltage, sends voltage to motor, and posts telemetry.
     */
    @Override
    public void execute() {
        // Change target velocity every 2.0 seconds
        if (timer.advanceIfElapsed(2.0)) {
            pickNewTargetRPM();
        }

        double currentRPM = motor.getVelocity().in(RPM);
        double errorRPM = targetRPM - currentRPM;

        // Feedforward: V_ff = kS * sgn(target) + kV * target
        double ffVolts = (Math.abs(targetRPM) > 1.0 ? Math.copySign(kS, targetRPM) : 0.0) + (kV * targetRPM);

        // Feedback: V_pid = PID(current, target)
        double pidVolts = pidController.calculate(currentRPM, targetRPM);

        // Commanded voltage clamped to standard battery limits [-12V, 12V]
        double totalVolts = MathUtil.clamp(ffVolts + pidVolts, -12.0, 12.0);
        motor.setVoltage(totalVolts);

        // Real-time telemetry for AdvantageScope / SmartDashboard
        SmartDashboard.putNumber("VelocityTester/TargetRPM", targetRPM);
        SmartDashboard.putNumber("VelocityTester/ActualRPM", currentRPM);
        SmartDashboard.putNumber("VelocityTester/ErrorRPM", errorRPM);
        SmartDashboard.putNumber("VelocityTester/AppliedVolts", totalVolts);
        SmartDashboard.putNumber("VelocityTester/FFVolts", ffVolts);
        SmartDashboard.putNumber("VelocityTester/PIDVolts", pidVolts);
    }

    /**
     * Selects a new random target RPM within the configured range, rounded to the nearest 50 RPM.
     */
    private void pickNewTargetRPM() {
        targetRPM = minRPM + (random.nextDouble() * (maxRPM - minRPM));
        targetRPM = Math.round(targetRPM / 50.0) * 50.0;
    }

    /**
     * Indicates whether the command has finished.
     *
     * @return {@code false} so the command continues executing indefinitely until stopped.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Shuts off the motor and stops the timer when the command is cancelled or interrupted.
     *
     * @param interrupted {@code true} if the command was cancelled.
     */
    @Override
    public void end(boolean interrupted) {
        motor.setVoltage(0);
        timer.stop();
        System.out.println("=== Velocity Tester Stopped ===");
    }
}
