package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.Alert;

public class SUB_Roller extends SubsystemBase {
    // Initiliazes values and objects used in subsystem
    private TalonFX roller;
    private static SUB_Roller INSTANCE = null;
    public static SUB_Roller getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Roller();
        } 
        return INSTANCE;
    }

    private SUB_Roller () {
        //Defines motors with IDs and what controller
        roller = new TalonFX(Constants.Roller.kINTAKE_MOTOR_CANID);
        configureMotors();
    }

    private void configureMotors(){
        //Creates config for motors
        TalonFXConfiguration talonConfig = new TalonFXConfiguration(); //Creates new TalonFX Config
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true; //enables supply current limit which is how much goes to motor controller
        talonConfig.CurrentLimits.SupplyCurrentLimit = 63; //Sets high supply current limit in amps
        talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40; //Sets low supply current limit in amps
        talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1.2; //Sets how long current has to be above limit before it is considered a fault in seconds
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Makes it so positive values make the motor spin CC
        roller.getConfigurator().apply(talonConfig); //Applies Config to the intake roller
    }


    //Sets voltage of intake roller
    public void setVolts(double speed){
        roller.setVoltage(speed);
    }

    //Sets speed of intake roller
    public void set(double speed){
        roller.set(speed);
    }
    //Returns RPM of intake roller
    public double rollerRPM(){
        return roller.getVelocity().getValue().baseUnitMagnitude();
    }

    // Logs everything every periodic
    public void periodic() {
                if (RobotContainer.shouldAlert) {
        SmartDashboard.putNumber("Roller/RollerRPM", rollerRPM()); //puts roller motor RPM into Smart Dashboard

        
        SmartDashboard.putNumber("Roller/Roller Encoder Pos", roller.getPosition().getValueAsDouble());


        SmartDashboard.putNumber("Roller/Roller Stator Current", roller.getStatorCurrent().getValueAsDouble()); //Return stator current of intake roller
        SmartDashboard.putNumber("Roller/Roller Supply Current", roller.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Torque Current", roller.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Roller/Roller Supply Voltage", roller.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Motor Voltage", roller.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Roller/Roller Device Temp", roller.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Roller/Roller Processor Temp", roller.getProcessorTemp().getValueAsDouble());
          Alert.alertKraken(roller);
                }
    }

    
}
