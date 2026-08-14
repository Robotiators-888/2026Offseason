package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Linear extends SubsystemBase {
    /** Ceiling on the position loop's duty-cycle output. */
    private static final double kMaxOutput = 0.6;

    private final SparkMax linear;
    private static SUB_Linear INSTANCE = null;

    /**
     * Owned here rather than shared out of Constants. As {@code public static final} fields they
     * were handed to several concurrently-schedulable commands at once, which meant the integral
     * accumulator and setpoint were global mutable state.
     */
    private final PIDController fastController = new PIDController(
        Constants.Linear.kLINEAR_FAST_kP, 0, 0);
    private final PIDController slowController = new PIDController(
        Constants.Linear.kLINEAR_SLOW_kP, 0, 0);

    /**
     * @return Single instance of the SUB_Arm subsystem
     */
    public static SUB_Linear getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Linear();
        } 
        return INSTANCE;
    }

    private SUB_Linear () {
        // Defines motors with IDs and motor type
        linear = new SparkMax(Constants.Linear.kLINEAR_MOTOR_CANID, MotorType.kBrushless);
        configureMotors();
    }

    private void configureMotors(){
        // Creates config for motor and encoder (23:1 cycloidal gearbox)
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35, 5); // Sets stall limit in amps
        config.inverted(true);
        linear.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Linear/Linear Encoder Pos", linear.getEncoder().getPosition());
        SmartDashboard.putNumber("Linear/Linear Output Current", linear.getOutputCurrent());
        SmartDashboard.putNumber("Linear/Linear Bus Voltage", linear.getBusVoltage());
        SmartDashboard.putNumber("Linear/Linear Motor Temp", linear.getMotorTemperature());

        Alert.alertNeoFaults(linear);
        Alert.alertNeoWarnings(linear);
    }

    /** Drives the arm out at the fast gain. */
    public void forward() {
        driveTo(fastController, Constants.Linear.kLINEAR_FORWARD_SETPOINT);
    }

    /** Retracts the arm at the slow gain, which is what compressing against the stop wants. */
    public void backward() {
        driveTo(slowController, Constants.Linear.kLINEAR_BACKWARD_SETPOINT);
    }

    /**
     * Runs one iteration of a position loop.
     *
     * <p>The output is clamped: these are proportional gains of 4 and 1 against an error measured
     * in encoder rotations, so without a clamp any error above a quarter turn saturates the motor.
     */
    private void driveTo(final PIDController controller, final double setpoint) {
        final double output = controller.calculate(linear.getEncoder().getPosition(), setpoint);
        linear.set(MathUtil.clamp(output, -kMaxOutput, kMaxOutput));
    }

    public void set(double speed) {
        linear.set(speed);
    }
}
