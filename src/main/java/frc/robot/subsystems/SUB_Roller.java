package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Roller extends SubsystemBase {
    /** Subsystem hardware components */
    private TalonFX LeftRollerMotor;
    private TalonFX RightRollerMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
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
        LeftRollerMotor = new TalonFX(Constants.Roller.kINTAKE_LEFTMOTOR_CANID);
        RightRollerMotor = new TalonFX(Constants.Roller.kINTAKE_RIGHTMOTOR_CANID);
        configureMotors();
    }

    private void configureMotors(){
        // Configure TalonFX motor controller with current limits and inversion
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = 80;
        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = 2.2;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        LeftRollerMotor.getConfigurator().apply(talonConfig);
        RightRollerMotor.getConfigurator().apply(talonConfig);

        RightRollerMotor.setControl(new Follower(LeftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }


    /** @param speed Target voltage for the roller motor */
    public void setVolts(double speed){
        LeftRollerMotor.setControl(voltageRequest.withOutput(speed));
    }

    /** @param speed Target percent output for the roller motor [-1.0, 1.0] */
    public void set(double speed){
        LeftRollerMotor.setControl(dutyCycleRequest.withOutput(speed));
    }

    /** @return Current velocity of the roller in RPM */
    public double rollerRPM(){
        return (LeftRollerMotor.getVelocity().getValue().baseUnitMagnitude()+ RightRollerMotor.getVelocity().getValue().baseUnitMagnitude())/2;
    }

    @Override
    public void periodic() {
        // Telemetry logging for dashboard
        SmartDashboard.putNumber("RollerRPM", rollerRPM());
        SmartDashboard.putNumber("Left Roller Encoder Pos", LeftRollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Encoder Pos", RightRollerMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Left Roller Stator Current", LeftRollerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Stator Current", RightRollerMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Roller Supply Current", LeftRollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Supply Current", RightRollerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Left Roller Torque Current", LeftRollerMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Torque Current", RightRollerMotor.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Roller Supply Voltage", LeftRollerMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Supply Voltage", RightRollerMotor.getSupplyVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Roller Motor Voltage", LeftRollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Motor Voltage", RightRollerMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Roller Device Temp", LeftRollerMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Device Temp", RightRollerMotor.getDeviceTemp().getValueAsDouble());

        SmartDashboard.putNumber("Left Roller Processor Temp", LeftRollerMotor.getProcessorTemp().getValueAsDouble());
        SmartDashboard.putNumber("Right Roller Processor Temp", RightRollerMotor.getProcessorTemp().getValueAsDouble());


        Alert.alertKraken(LeftRollerMotor);
        Alert.alertKraken(RightRollerMotor);
    }

    
}
