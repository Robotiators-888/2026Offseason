package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Linear extends SubsystemBase {
    /** Subsystem state and configuration constants */
    public static boolean extended;
    private SparkMax linear;
    private static SUB_Linear INSTANCE = null;

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
        config.encoder.positionConversionFactor(360.0 / 23); // Converts rotations to degrees
        config.encoder.velocityConversionFactor((360.0 / 23) / 60.0); // Converts RPM to deg/sec
        config.smartCurrentLimit(35); // Sets stall limit in amps
        config.inverted(true);
        linear.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Arm/Arm Encoder Pos", linear.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm/Arm Output Current", linear.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Arm Bus Voltage", linear.getBusVoltage());
        SmartDashboard.putNumber("Arm/Arm Motor Temp", linear.getMotorTemperature());

        Alert.alertNeoFaults(linear);
        Alert.alertNeoWarnings(linear);
    }

    public boolean isExtended() {
        return extended;
    }

    /** Drives the arm to the bottom setpoint using PID */
    public void forward(PIDController controller) {
        linear.set(controller.calculate(linear.getEncoder().getPosition(), Constants.Linear.kLINEAR_BOTTOM_SETPOINT)); 
    }

    /** Drives the arm to the top setpoint using PID */
    public void backward(PIDController controller) {
        linear.set(controller.calculate(linear.getEncoder().getPosition(), Constants.Linear.kLINEAR_TOP_SETPOINT)); 
    }

    public boolean isForward() {
        return Math.abs(linear.getEncoder().getPosition() - Constants.Linear.kLINEAR_BOTTOM_SETPOINT) < 3.0;
    }

    public boolean isBackward() {
        return Math.abs(linear.getEncoder().getPosition() - Constants.Linear.kLINEAR_TOP_SETPOINT) < 3.0;
    }

    public void set(double speed) {
        linear.set(speed);
    }
}
