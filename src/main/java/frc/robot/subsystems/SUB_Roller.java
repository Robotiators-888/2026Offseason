package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Roller extends SubsystemBase {
    /** Subsystem hardware components */
    private TalonFX roller;
    private static SUB_Roller INSTANCE = null;

    /**
     * @return Single instance of the SUB_Roller subsystem
     */
    public static SUB_Roller getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Roller();
        } 
        return INSTANCE;
    }

    private SUB_Roller () {
        // Defines motor with ID from Constants
        roller = new TalonFX(Constants.Roller.kINTAKE_MOTOR_CANID);
        configureMotors();
    }

    private void configureMotors(){
        // Configure TalonFX motor controller with current limits and inversion
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1.2;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        roller.getConfigurator().apply(talonConfig);
    }


    /** @param speed Target voltage for the roller motor */
    public void setVolts(double speed){
        roller.setVoltage(speed);
    }

    /** @param speed Target percent output for the roller motor [-1.0, 1.0] */
    public void set(double speed){
        roller.set(speed);
    }

    /** @return Current velocity of the roller in RPM */
    public double rollerRPM(){
        return roller.getVelocity().getValue().baseUnitMagnitude();
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("Roller/RollerRPM", rollerRPM());
        SmartDashboard.putNumber("Roller/Roller Encoder Pos", roller.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Stator Current", roller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Supply Current", roller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Torque Current", roller.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Supply Voltage", roller.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Motor Voltage", roller.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Device Temp", roller.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Processor Temp", roller.getProcessorTemp().getValueAsDouble());

        Alert.alertKraken(roller);
    }

    
}
