package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CommandSwerveDrivetrain;

public class RobotTelemetry {
    /**
     * PDH channels that actually have something wired to them. Channels 12-14 and 23 are unused and
     * would raise false breaker faults.
     */
    private static final int[] kMonitoredPdhChannels = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 15, 16, 17, 18, 19, 20, 21, 22
    };

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
                // One read of the fault struct, not one per channel.
                final PowerDistributionFaults faults = powerDistribution.getFaults();

                if (faults.Brownout)
                        Alert.registerError("PDH Brownout!!");
                if (faults.CanWarning)
                        Alert.registerWarning("PDH CanWarning");
                if (faults.HardwareFault)
                        Alert.registerError("PDH HardwareFault");

                // Previously hand-written per channel, which had drifted: channel 18 was checked
                // twice, channel 9's fault was reported as "Channel19BreakerFault", and channel 8
                // was never checked at all.
                for (final int channel : kMonitoredPdhChannels) {
                        if (faults.getBreakerFault(channel))
                                Alert.registerError("PDH Channel" + channel + "BreakerFault");
                }
        }

    private void checkAlerts() {
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            Alert.alertKraken(drivetrain.getModule(i).getDriveMotor());
            Alert.alertKraken(drivetrain.getModule(i).getSteerMotor());
        }
    }
}