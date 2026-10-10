package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Motor.TuneableMotor;
import static edu.wpi.first.units.Units.RPM;

/**
 * Automated empirical characterization and tuning command for velocity-controlled mechanisms.
 *
 * <p>Characterizes feedforward parameters ({@link #kS}, {@link #kV}, {@link #kA}) and calculates
 * an optimal proportional feedback gain ({@link #kP}) for any motor or coupled mechanism
 * (including flywheels, shooters, intakes, rollers, and leader-follower motor pairs).
 *
 * <p><b>Tuning Stages:</b>
 * <ol>
 *   <li><b>FIND_KS:</b> Slowly ramps voltage by 0.01V/loop until motion begins to identify static breakaway friction (in Volts).</li>
 *   <li><b>FIND_KV:</b> Holds 10 step voltages up to 12V for 2 seconds each, recording steady-state velocity to calculate linear velocity gain via linear regression (in Volts / RPM).</li>
 *   <li><b>SETUP_KP_TEST:</b> Cuts voltage to zero and waits until the motor comes to rest.</li>
 *   <li><b>RUN_KP_STEP:</b> Applies a 6V step to measure angular acceleration between 20% and 70% of steady-state speed, deriving acceleration gain {@link #kA} and calculating {@link #kP} from the desired recovery time.</li>
 *   <li><b>DONE:</b> Stops the motor and publishes final values to {@link SmartDashboard} and stdout.</li>
 * </ol>
 */
public class CMD_VelocityTuner extends Command {

    /** The motor or mechanism abstraction under test. */
    private final TuneableMotor motor;

    /** Universal timer used across state transitions and step measurements. */
    private final Timer timer = new Timer();
    
    /** Internal state machine stages. */
    private enum TuneState {
        FIND_KS,
        SETUP_KV_TEST,
        FIND_KV,
        SETUP_KP_TEST,
        RUN_KP_STEP,
        DONE
    }
    
    /** Current state machine phase. */
    private TuneState currentState = TuneState.FIND_KS;

    /**
     * Calculated static friction feedforward voltage.
     * <p><b>Unit:</b> Volts (V)
     */
    public double kS = 0;

    /**
     * Calculated velocity feedforward gain (voltage required per unit steady-state speed).
     * <p><b>Unit:</b> Volts per RPM (V / RPM)
     */
    public double kV = 0;

    /**
     * Calculated mechanism acceleration gain (voltage required per unit angular acceleration).
     * <p><b>Unit:</b> Volts per RPM per second (V / (RPM/s))
     */
    public double kA = 0;

    /**
     * Calculated proportional feedback gain for closed-loop error correction.
     * <p><b>Unit:</b> Volts per RPM error (V / RPM)
     */
    public double kP = 0.0;

    /**
     * Desired disturbance recovery time constant.
     * <p><b>Unit:</b> Seconds (s)
     */
    public double recoveryTime = 0.080; 

    /** Internal voltage applied during the kS ramp test (Volts). */
    private double currentVoltage = 0.0;

    /** Linear regression accumulators for kV calculation. */
    private double sumX, sumY, sumXY, sumXX;
    private int n;

    /** Theoretical steady-state speed at 6V step during kA/kP measurement (RPM). */
    private double kpTestTargetSpeed = 0.0;

    /** Timestamp when mechanism reached 20% steady speed during step response (seconds). */
    private double t10 = -1.0;

    /** Timestamp when mechanism reached 70% steady speed during step response (seconds). */
    private double t90 = -1.0;

    /** Velocity recorded at 20% steady speed (RPM). */
    private double v10 = 0.0;

    /** Velocity recorded at 70% steady speed (RPM). */
    private double v90 = 0.0;

    /** Step-counter for kV voltage increments. */
    private double skibidi = 2.0;

    /**
     * Constructs a velocity tuner with a specified target recovery duration.
     *
     * @param motor The {@link TuneableMotor} mechanism to characterize and tune.
     * @param recoveryTime Target disturbance recovery duration in milliseconds (e.g. {@code 80.0} ms)
     *                     or seconds (e.g. {@code 0.080} s).
     */
    public CMD_VelocityTuner(TuneableMotor motor, double recoveryTime) {
        this.motor = motor;
        this.recoveryTime = (recoveryTime > 1.0) ? (recoveryTime / 1000.0) : recoveryTime;
    }

    /**
     * Constructs a velocity tuner with the standard default recovery time of 80 milliseconds (0.080 s).
     *
     * @param motor The {@link TuneableMotor} mechanism to characterize and tune.
     */
    public CMD_VelocityTuner(TuneableMotor motor) {
        this(motor, 0.080);
    }

    /**
     * Initializes the command, resetting state machine flags, linear regression accumulators, and timers.
     */
    @Override
    public void initialize() {
        currentState = TuneState.FIND_KS;
        currentVoltage = 0.0;
        kP = 0.0;
        kA = 0.0;
        sumX = 0;
        sumY = 0;
        sumXY = 0;
        sumXX = 0;
        n = 0; 
        skibidi = 2.0;
        timer.reset();
        timer.start();
    }

    /**
     * Periodic execution loop called every 20ms by the WPILib command scheduler.
     * Advances the tuning state machine and updates live dashboard telemetry.
     */
    @Override
    public void execute() {
        switch (currentState) {
            
            case FIND_KS:
                // Slowly ramp voltage by 0.01V (10mV) every 20ms
                currentVoltage += 0.01; 
                motor.setVoltage(currentVoltage);
                
                // Detect initial motion breakaway (threshold > 0.1 RPM)
                if (motor.getVelocity().in(RPM) > 0.1) {
                    kS = currentVoltage - 0.02; // Account for step size and sensor filter latency
                    motor.setVoltage(0);
                    timer.reset();
                    currentState = TuneState.FIND_KV;
                }
                
                // Safety limit: do not exceed 6V looking for kS
                if (currentVoltage >= 6.0) {
                    motor.setVoltage(0);
                    currentState = TuneState.DONE;
                }
                break;

            case SETUP_KV_TEST: 
                motor.setVoltage(0);
                skibidi = 2.0;
                currentState = TuneState.FIND_KV;
                break;

            case FIND_KV:
                // Step voltage through 10 equal increments up to 12V
                if (skibidi % 2 == 0) {
                    timer.reset();
                    timer.start();
                    motor.setVoltage(kS + (skibidi / 2.0) * ((12.0 - kS) / 10.0));
                    skibidi++;
                }

                // Wait 2.0 seconds for mechanism velocity to reach steady-state
                if (timer.get() > 2.0) {
                    double velocity = motor.getVelocity().in(RPM);
                    double targetVolts = ((skibidi - 1) / 2.0) * ((12.0 - kS) / 10.0);
                    if (velocity > 0.5) {
                        sumX += velocity;
                        sumY += targetVolts;
                        sumXY += velocity * targetVolts;
                        sumXX += velocity * velocity;
                        n++;
                    }
                    skibidi++;
                }

                // Finish regression when all 10 voltage steps have been sampled
                if (skibidi > 21.0) {
                    kV = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
                    motor.setVoltage(0);
                    currentState = TuneState.SETUP_KP_TEST;
                }
                break;

            case SETUP_KP_TEST: 
                motor.setVoltage(0);
                kpTestTargetSpeed = (6.0 - kS) / kV;
                timer.reset();
                t10 = -1.0;
                t90 = -1.0;
                v10 = 0.0;
                v90 = 0.0;
                // Ensure motor comes completely to rest before firing step response
                if (motor.getVelocity().in(RPM) > 5.0) {
                    currentState = TuneState.SETUP_KP_TEST; 
                } else {
                    currentState = TuneState.RUN_KP_STEP;
                }
                break;

            case RUN_KP_STEP:
                motor.setVoltage(6.0); // 6V step to measure mechanism acceleration
                double currentSpeed = motor.getVelocity().in(RPM);
                
                // Sample linear acceleration region between 20% and 70% of steady-state speed
                if (t10 < 0 && currentSpeed >= 0.20 * kpTestTargetSpeed) {
                    t10 = timer.get();
                    v10 = currentSpeed;
                }
                if (t90 < 0 && currentSpeed >= 0.70 * kpTestTargetSpeed) {
                    t90 = timer.get();
                    v90 = currentSpeed;
                }
                
                // Calculate kA and kP once acceleration window is captured (or on 1.5s timeout)
                if ((t90 > 0 && t10 > 0) || timer.get() >= 1.5) {
                    if (t90 > 0 && t10 > 0 && t90 > t10) {
                        double dt = t90 - t10; // seconds
                        double dw = v90 - v10; // RPM
                        double alpha = dw / dt; // RPM / s
                        double avgSpeed = (v10 + v90) / 2.0; // RPM
                        double netVolts = 6.0 - kS - (kV * avgSpeed); // Volts accelerating the mechanism
                        kA = netVolts / alpha; // V / (RPM/s)
                        kP = kA / recoveryTime; // V / RPM
                    }
                    motor.setVoltage(0);
                    currentState = TuneState.DONE;
                }
                break;

            case DONE:
                motor.setVoltage(0);
                break;
        }

        // Live dashboard telemetry
        SmartDashboard.putNumber("VelocityTuner/State", currentState.ordinal());
        SmartDashboard.putNumber("VelocityTuner/S", kS);
        SmartDashboard.putNumber("VelocityTuner/V", kV);
        SmartDashboard.putNumber("VelocityTuner/A", kA);
        SmartDashboard.putNumber("VelocityTuner/P", kP);

        SmartDashboard.putNumber("VelocityTuner/Current Voltage", currentVoltage);
        SmartDashboard.putNumber("VelocityTuner/kpTestTargetSpeed", kpTestTargetSpeed);
        SmartDashboard.putNumber("VelocityTuner/skibidi", skibidi);
    }

    /**
     * Determines whether the tuning sequence has completed.
     *
     * @return {@code true} when the tuner reaches {@link TuneState#DONE}.
     */
    @Override
    public boolean isFinished() {
        return currentState == TuneState.DONE;
    }

    /**
     * Cleanup hook called when the command finishes or is interrupted.
     * Safely shuts off motor voltage and prints final tuned constants to standard output.
     *
     * @param interrupted {@code true} if the command was cancelled prior to completing.
     */
    @Override
    public void end(boolean interrupted) {
        motor.setVoltage(0);
        timer.stop();
        
        System.out.println("--- TUNING RESULTS ---");
        System.out.println("Calculated kS: " + kS + " Volts");
        System.out.println("Calculated kV: " + kV + " Volts/RPM");
        System.out.println("Calculated kA: " + kA + " Volts/(RPM/s)");
        System.out.println("Optimized kP:  " + kP + " Volts/RPM (for " + String.format("%.0f ms", recoveryTime * 1000.0) + " recovery)");
    }
}
