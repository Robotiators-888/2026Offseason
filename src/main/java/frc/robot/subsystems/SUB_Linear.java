package frc.robot.subsystems;

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
    /** Subsystem state and configuration constants */
    public static boolean extended = false;
    private final SparkMax linear;
    private static SUB_Linear INSTANCE = null;

    /**
     * @return Single instance of the SUB_Linear subsystem
     */
    public static SUB_Linear getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Linear();
        } 
        return INSTANCE;
    }

    private SUB_Linear() {
        linear = new SparkMax(Constants.Linear.kLINEAR_MOTOR_CANID, MotorType.kBrushless);
        configureMotors();
    }

    @SuppressWarnings("removal")
    private void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35, 15); // Stall limit 35A, free limit 15A
        config.inverted(true);
        linear.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        extended = isForward();
        SmartDashboard.putNumber("Linear/Linear Encoder Pos", linear.getEncoder().getPosition());
        SmartDashboard.putNumber("Linear/Linear Output Current", linear.getOutputCurrent());
        SmartDashboard.putNumber("Linear/Linear Bus Voltage", linear.getBusVoltage());
        SmartDashboard.putNumber("Linear/Linear Motor Temp", linear.getMotorTemperature());
        SmartDashboard.putBoolean("Linear/Extended", extended);

        Alert.alertNeoFaults(linear);
        Alert.alertNeoWarnings(linear);
    }

    public boolean isExtended() {
        return extended;
    }

    public void forward(final PIDController controller) {
        double output = controller.calculate(linear.getEncoder().getPosition(), Constants.Linear.kLINEAR_FORWARD_SETPOINT);
        linear.set(MathUtil.clamp(output, -1.0, 1.0)); 
    }

    public void backward(final PIDController controller) {
        double output = controller.calculate(linear.getEncoder().getPosition(), Constants.Linear.kLINEAR_BACKWARD_SETPOINT);
        linear.set(MathUtil.clamp(output, -1.0, 1.0)); 
    }

    public boolean isForward() {
        return Math.abs(linear.getEncoder().getPosition() - Constants.Linear.kLINEAR_FORWARD_SETPOINT) < 3.0;
    }

    public boolean isBackward() {
        return Math.abs(linear.getEncoder().getPosition() - Constants.Linear.kLINEAR_BACKWARD_SETPOINT) < 3.0;
    }

    public void set(double speed) {
        linear.set(MathUtil.clamp(speed, -1.0, 1.0));
    }
}
