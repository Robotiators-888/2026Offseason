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

    /** Loops between diagnostic publishes of any one swerve module (staggered): ~4 Hz each. */
    private static final int kModuleTelemetryPeriodLoops = 12;

    private final CommandSwerveDrivetrain drivetrain;
    private final PowerDistribution powerDistribution;

    /**
     * Dashboard keys per module, built once. The per-loop string concatenation this replaces
     * allocated 76 fresh key strings every 20 ms (~3,800 per second) for names that never change.
     */
    private final String[][] moduleKeys;

    private int telemetryTick = 0;

    public RobotTelemetry(CommandSwerveDrivetrain drivetrain, PowerDistribution powerDistribution) {
        this.drivetrain = drivetrain;
        this.powerDistribution = powerDistribution;

        moduleKeys = new String[drivetrain.getModules().length][];
        for (int i = 0; i < moduleKeys.length; i++) {
            final int driveId = drivetrain.getModule(i).getDriveMotor().getDeviceID();
            final int steerId = drivetrain.getModule(i).getSteerMotor().getDeviceID();
            final int encoderId = drivetrain.getModule(i).getEncoder().getDeviceID();
            moduleKeys[i] = new String[] {
                "Drivetrain/Motors/Current/Drive Motor ID " + driveId + " Stator Current",
                "Drivetrain/Motors/Current/Steer Motor ID " + steerId + " Stator Current",
                "Drivetrain/Motors/Current/Drive Motor ID " + driveId + " Supply Current",
                "Drivetrain/Motors/Current/Steer Motor ID " + steerId + " Supply Current",
                "Drivetrain/Motors/Voltage/Drive Motor ID " + driveId + " Motor Voltage",
                "Drivetrain/Motors/Voltage/Steer Motor ID " + steerId + " Motor Voltage",
                "Drivetrain/Motors/Voltage/Drive Motor ID " + driveId + " Supply Voltage",
                "Drivetrain/Motors/Voltage/Steer Motor ID " + steerId + " Supply Voltage",
                "Drivetrain/Motors/RPM/Drive Motor ID " + driveId + " RPM",
                "Drivetrain/Motors/RPM/Steer Motor ID " + steerId + " RPM",
                "Drivetrain/Motors/Pos/Drive Motor ID " + driveId + " Encoder Pos",
                "Drivetrain/Motors/Pos/Steer Motor ID " + steerId + " Encoder Pos",
                "Drivetrain/Motors/Current/Drive Motor ID " + driveId + " Torque Current",
                "Drivetrain/Motors/Current/Steer Motor ID " + steerId + " Torque Current",
                "Drivetrain/Motors/Temp/Drive Motor ID " + driveId + " Device Temp",
                "Drivetrain/Motors/Temp/Steer Motor ID " + steerId + " Device Temp",
                "Drivetrain/Motors/Temp/Drive Motor ID " + driveId + " Processor Temp",
                "Drivetrain/Motors/Temp/Steer Motor ID " + steerId + " Processor Temp",
                "Drivetrain/Motors/AbsEncoder/Encoder ID " + encoderId + " Position",
            };
        }
    }

    public void update() {
        telemetryTick++;
        logDrivetrain();
        logPDH();
        checkAlerts();
    }

    private void logDrivetrain() {
        // Module states at full rate — they are what anyone watches live.
        drivetrain.swerveModuleStatesPublisher.set(drivetrain.getState().ModuleStates);
        drivetrain.desiredSwerveModuleStatesPublisher.set(drivetrain.getState().ModuleTargets);

        for (int i = 0; i < drivetrain.getModules().length; i++) {
            // Per-module diagnostics staggered to ~4 Hz each; all four at full rate was the
            // single biggest block of the robot's ~10,000 NT updates per second.
            if ((telemetryTick + i) % kModuleTelemetryPeriodLoops != 0) {
                continue;
            }
            TalonFX driveMotor = drivetrain.getModule(i).getDriveMotor();
            TalonFX steerMotor = drivetrain.getModule(i).getSteerMotor();
            String[] keys = moduleKeys[i];

            SmartDashboard.putNumber(keys[0], driveMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber(keys[1], steerMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber(keys[2], driveMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber(keys[3], steerMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber(keys[4], driveMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber(keys[5], steerMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber(keys[6], driveMotor.getSupplyVoltage().getValueAsDouble());
            SmartDashboard.putNumber(keys[7], steerMotor.getSupplyVoltage().getValueAsDouble());
            SmartDashboard.putNumber(keys[8], driveMotor.getVelocity().getValue().baseUnitMagnitude());
            SmartDashboard.putNumber(keys[9], steerMotor.getVelocity().getValue().baseUnitMagnitude());
            SmartDashboard.putNumber(keys[10], driveMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber(keys[11], steerMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber(keys[12], driveMotor.getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber(keys[13], steerMotor.getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber(keys[14], driveMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber(keys[15], steerMotor.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber(keys[16], driveMotor.getProcessorTemp().getValueAsDouble());
            SmartDashboard.putNumber(keys[17], steerMotor.getProcessorTemp().getValueAsDouble());
            SmartDashboard.putNumber(keys[18],
                    drivetrain.getModule(i).getEncoder().getPosition().getValueAsDouble());
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
        // alertKraken self-staggers by device ID, so calling it every loop is cheap.
        for (int i = 0; i < drivetrain.getModules().length; i++) {
            Alert.alertKraken(drivetrain.getModule(i).getDriveMotor());
            Alert.alertKraken(drivetrain.getModule(i).getSteerMotor());
        }
    }
}
