package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandSwerveDrivetrain;

public class RobotTelemetry {
    private final CommandSwerveDrivetrain drivetrain;
    private final PowerDistribution powerDistribution;

    public RobotTelemetry(CommandSwerveDrivetrain drivetrain, PowerDistribution powerDistribution) {
        this.drivetrain = drivetrain;
        this.powerDistribution = powerDistribution;
    }

    public void update() {
        logDrivetrain();
        logPDH();
        checkAlerts();
    }

    private void logDrivetrain() {
        drivetrain.swerveModuleStatesPublisher.set(drivetrain.getState().ModuleStates);
        drivetrain.desiredSwerveModuleStatesPublisher.set(drivetrain.getState().ModuleTargets);
        
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            TalonFX driveMotor = drivetrain.getModule(i).getDriveMotor();
            TalonFX steerMotor = drivetrain.getModule(i).getSteerMotor();
            int driveMotorId = driveMotor.getDeviceID();
            int steerMotorId = steerMotor.getDeviceID();
            
            SmartDashboard.putNumber("Drivetrain/Motors/Current/Drive Motor ID " + driveMotorId + " Stator Current", driveMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Current/Steer Motor ID " + steerMotorId + " Stator Current", steerMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Current/Drive Motor ID " + driveMotorId + " Supply Current", driveMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Current/Steer Motor ID " + steerMotorId + " Supply Current", steerMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Drive Motor ID " + driveMotorId + " Motor Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Steer Motor ID " + steerMotorId + " Motor Voltage", steerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Drive Motor ID " + driveMotorId + " Supply Voltage", driveMotor.getSupplyVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Voltage/Steer Motor ID " + steerMotorId + " Supply Voltage", steerMotor.getSupplyVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/RPM/Drive Motor ID " + driveMotorId + " RPM", driveMotor.getVelocity().getValue().baseUnitMagnitude());
            SmartDashboard.putNumber("Drivetrain/Motors/RPM/Steer Motor ID " + steerMotorId + " RPM", steerMotor.getVelocity().getValue().baseUnitMagnitude());
            SmartDashboard.putNumber("Drivetrain/Motors/Pos/Drive Motor ID " + driveMotorId + " Encoder Pos", driveMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Pos/Steer Motor ID " + steerMotorId + " Encoder Pos", steerMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Current/Drive Motor ID " + driveMotorId + " Torque Current", driveMotor.getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Current/Steer Motor ID " + steerMotorId + " Torque Current", steerMotor.getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Temp/Drive Motor ID " + driveMotorId + " Device Temp", driveMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Temp/Steer Motor ID " + steerMotorId + " Device Temp", steerMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Temp/Drive Motor ID " + driveMotorId + " Processor Temp", driveMotor.getProcessorTemp().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/Temp/Steer Motor ID " + steerMotorId + " Processor Temp", steerMotor.getProcessorTemp().getValueAsDouble());
            SmartDashboard.putNumber("Drivetrain/Motors/AbsEncoder/Encoder ID " + drivetrain.getModule(i).getEncoder().getDeviceID() + " Position", drivetrain.getModule(i).getEncoder().getPosition().getValueAsDouble());
        }
    }

    public void logPDH () {
                SmartDashboard.putNumber("PDH/Battery Voltage", powerDistribution.getVoltage());
                SmartDashboard.putNumberArray("PDH/All Currents Amps", powerDistribution.getAllCurrents());
                SmartDashboard.putNumber("PDH/Tempurature Celsius", powerDistribution.getTemperature());
                SmartDashboard.putNumber("PDH/Total Current Amps", powerDistribution.getTotalCurrent());
                SmartDashboard.putNumber("PDH/Total Energy Joules", powerDistribution.getTotalEnergy());
                // Not supported on PDH
                // SmartDashboard.putNumber("PDH/Total Watts", powerDistribution.getTotalPower());
                if (powerDistribution.getFaults().Brownout)
                        Alert.registerError("PDH Brownout!!");
                if (powerDistribution.getFaults().CanWarning)
                        Alert.registerWarning("PDH CanWarning");
                if (powerDistribution.getFaults().HardwareFault)
                        Alert.registerError("PDH HardwareFault");
                if (powerDistribution.getFaults().Channel0BreakerFault)
                        Alert.registerError("PDH Channel0BreakerFault");
                if (powerDistribution.getFaults().Channel10BreakerFault)
                        Alert.registerError("PDH Channel10BreakerFault");
                if (powerDistribution.getFaults().Channel11BreakerFault)
                        Alert.registerError("PDH Channel11BreakerFault");
                // We dont use these ports so they cause false errors
                // if (powerDistribution.getFaults().Channel12BreakerFault)
                //         Alert.registerError("PDH Channel12BreakerFault");
                // if (powerDistribution.getFaults().Channel13BreakerFault)
                //         Alert.registerError("PDH Channel13BreakerFault");
                // if (powerDistribution.getFaults().Channel14BreakerFault)
                //         Alert.registerError("PDH Channel14BreakerFault");
                if (powerDistribution.getFaults().Channel15BreakerFault)
                        Alert.registerError("PDH Channel15BreakerFault");
                if (powerDistribution.getFaults().Channel16BreakerFault)
                        Alert.registerError("PDH Channel16BreakerFault");
                if (powerDistribution.getFaults().Channel17BreakerFault)
                        Alert.registerError("PDH Channel17BreakerFault");
                if (powerDistribution.getFaults().Channel18BreakerFault)
                        Alert.registerError("PDH Channel18BreakerFault");
                if (powerDistribution.getFaults().Channel19BreakerFault)
                        Alert.registerError("PDH Channel19BreakerFault");
                if (powerDistribution.getFaults().Channel1BreakerFault)
                        Alert.registerError("PDH Channel1BreakerFault");
                if (powerDistribution.getFaults().Channel20BreakerFault)
                        Alert.registerError("PDH Channel20BreakerFault");
                if (powerDistribution.getFaults().Channel21BreakerFault)
                        Alert.registerError("PDH Channel21BreakerFault");
                if (powerDistribution.getFaults().Channel22BreakerFault)
                        Alert.registerError("PDH Channel22BreakerFault");
                // Also unused
                // if (powerDistribution.getFaults().Channel23BreakerFault)
                //         Alert.registerError("PDH Channel23BreakerFault");
                if (powerDistribution.getFaults().Channel2BreakerFault)
                        Alert.registerError("PDH Channel2BreakerFault");
                if (powerDistribution.getFaults().Channel3BreakerFault)
                        Alert.registerError("PDH Channel3BreakerFault");
                if (powerDistribution.getFaults().Channel4BreakerFault)
                        Alert.registerError("PDH Channel4BreakerFault");
                if (powerDistribution.getFaults().Channel5BreakerFault)
                        Alert.registerError("PDH Channel5BreakerFault");
                if (powerDistribution.getFaults().Channel6BreakerFault)
                        Alert.registerError("PDH Channel6BreakerFault");
                if (powerDistribution.getFaults().Channel7BreakerFault)
                        Alert.registerError("PDH Channel7BreakerFault");
                if (powerDistribution.getFaults().Channel18BreakerFault)
                        Alert.registerError("PDH Channel18BreakerFault");
                if (powerDistribution.getFaults().Channel9BreakerFault)
                        Alert.registerError("PDH Channel19BreakerFault");
        }

    private void checkAlerts() {
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            Alert.alertKraken(drivetrain.getModule(i).getDriveMotor());
            Alert.alertKraken(drivetrain.getModule(i).getSteerMotor());
        }
    }
}