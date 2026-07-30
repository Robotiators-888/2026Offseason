package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUB_Hood extends SubsystemBase {
    private static SUB_Hood INSTANCE = null;
    private final TalonFX hood;

    public static SUB_Hood getInstance () {
        if (INSTANCE == null) {
            INSTANCE = new SUB_Hood();
        }
        return INSTANCE;
    }

    private SUB_Hood () {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(25)
                .withSupplyCurrentLowerLimit(15)
                .withSupplyCurrentLowerTime(.5));
        hood = new TalonFX(Constants.Hood.kHOOD_CAN_ID);
        hood.getConfigurator().apply(config);
    }

    public void setToPosition (double angle) {
        hood.set(Constants.Hood.kHOOD_PID_CONTROLLER.calculate(hood.getPosition().getValueAsDouble(), angle));
    }

    public double getPosition () {
        return hood.getPosition().getValueAsDouble();
    }

    public static double findoptimalangle(double distance) {
        double lowestrpm = Double.MAX_VALUE;
        double optimalangle = 0;
        double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        double minexitrange = Units.radiansToDegrees(Math.atan2(2*height,distance));
        double maxexitrange = 85;
        for (int i = (int)Math.ceil(minexitrange); i < maxexitrange; i ++) {
            double exitvelocity = (1/Math.cos(Units.degreesToRadians(i)))*Math.sqrt((9.8*distance*distance)/(2*(distance*Math.tan(i)-height)));
            double exitRPM = ((720 / Constants.Shooter.ShooterDiameter)*exitvelocity)/(Constants.Shooter.CompressionValue * Math.PI);
            if (exitRPM < lowestrpm) {
                lowestrpm = exitRPM;
                optimalangle = i;
            }

        }
        return optimalangle;
    }

    // Reset the hood to the origional position while including checks for motor stalling
    public void resetSafe () {
        setToPosition(0); // TODO: Actually make this safe maybe by using a high I PID and detected when the current gets high
    }

    @Override
    public void periodic () {
        SmartDashboard.putNumber("Hood/Position", getPosition());
    }
}
