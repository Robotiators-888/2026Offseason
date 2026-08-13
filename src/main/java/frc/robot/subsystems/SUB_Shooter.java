package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;

    /** Subsystem hardware and control state */
    private final TalonFX MotorOne;
    private final TalonFX MotorTwo;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage m_request = new VelocityVoltage(0);
    private double desiredSpeed = 0;
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    /**
     * Current limits applied while actively shooting vs. idling. Only these differ between the two
     * states, so runtime switching applies a CurrentLimitsConfigs rather than the whole
     * TalonFXConfiguration.
     */
    private final CurrentLimitsConfigs highPowerLimits = new CurrentLimitsConfigs();
    private final CurrentLimitsConfigs lowPowerLimits = new CurrentLimitsConfigs();

    /** Tracks which limit profile is currently on the motors so we only touch CAN on a change. */
    private boolean highPower = false;

    /** Index into {@link Constants.Shooter#kREADY_RPM_BANDS}, held with hysteresis. */
    private int currentBand = 0;

    /** Interpolation map for distance-based RPM calibration */
    private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    /** @return Single instance of the SUB_Shooter subsystem */
    public static SUB_Shooter getInstance (){
        if (INSTANCE == null) {
            INSTANCE = new SUB_Shooter();
          }
      
          return INSTANCE;
    }

    private SUB_Shooter() {
        // Initialize dual flywheel motors
        MotorOne = new TalonFX(Constants.Shooter.kSHOOTER_MotorOne_MOTOR_CANID); 
        MotorTwo = new TalonFX(Constants.Shooter.kSHOOTER_MotorTwo_MOTOR_CANID);

        // Populate distance-to-RPM look-up table (meters -> RPM)
        distanceToRPM.put(2.49493587092, 1250.0);
        distanceToRPM.put(3.03308176613, 1375.0+15);
        distanceToRPM.put(1.6346195276, 1075.0-25);
        distanceToRPM.put(4.10526503, 1575.0+25);
        distanceToRPM.put(5.34766117, 1750.0+40);
        distanceToRPM.put(10.5, 2400.0); // TODO: REDO
        
        configFlywheel();
    }

    private void configFlywheel() {
        // The two current-limit profiles. Stator, neutral mode, inversion and gains are identical
        // between them, so only the supply limits are switched at runtime.
        highPowerLimits.StatorCurrentLimitEnable = true;
        highPowerLimits.StatorCurrentLimit = 100;
        highPowerLimits.SupplyCurrentLimitEnable = true;
        highPowerLimits.SupplyCurrentLimit = 60;
        highPowerLimits.SupplyCurrentLowerLimit = 40;
        highPowerLimits.SupplyCurrentLowerTime = 1.0;

        lowPowerLimits.StatorCurrentLimitEnable = true;
        lowPowerLimits.StatorCurrentLimit = 100;
        lowPowerLimits.SupplyCurrentLimitEnable = true;
        lowPowerLimits.SupplyCurrentLimit = 10;
        lowPowerLimits.SupplyCurrentLowerLimit = 5;
        lowPowerLimits.SupplyCurrentLowerTime = 1.0;

        // Boot into the idle profile so it agrees with the initial value of highPower.
        shooterConfig.CurrentLimits = lowPowerLimits;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure PID control loop coefficients
        shooterConfig.Slot0.kS = Constants.Shooter.kSHOOTER_FLYWHEEL_kS;
        shooterConfig.Slot0.kV = Constants.Shooter.kSHOOTER_FLYWHEEL_kV;
        shooterConfig.Slot0.kA = Constants.Shooter.kSHOOTER_FLYWHEEL_kA;
        shooterConfig.Slot0.kP = Constants.Shooter.kSHOOTER_FLYWHEEL_kP;
        shooterConfig.Slot0.kI = Constants.Shooter.kSHOOTER_FLYWHEEL_kI;
        shooterConfig.Slot0.kD = Constants.Shooter.kSHOOTER_FLYWHEEL_kD;

        MotorOne.getConfigurator().apply(shooterConfig);
        MotorTwo.getConfigurator().apply(shooterConfig);

        // Synchronize bottom flywheel to top flywheel
        MotorTwo.setControl(new Follower(MotorOne.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * Selects the shooting (high supply current) or idle (low supply current) limit profile.
     *
     * <p>Every command that spins the flywheel to score must call this with {@code true} in
     * {@code initialize()} and {@code false} in {@code end()}. The call is edge-triggered, so
     * repeating it costs nothing; only an actual change touches the CAN bus.
     *
     * @param enabled true while actively shooting
     */
    public void setHighPower(final boolean enabled) {
        if (enabled == highPower) {
            return;
        }
        highPower = enabled;
        final CurrentLimitsConfigs limits = enabled ? highPowerLimits : lowPowerLimits;
        MotorOne.getConfigurator().apply(limits);
        MotorTwo.getConfigurator().apply(limits);
    }

    /**
     * Flywheel speed needed to land a projectile launched at {@code angle} at horizontal range
     * {@code distance}.
     *
     * <p>{@code angle} is in <b>radians</b> throughout — it previously went into
     * {@code Units.degreesToRadians()} for the secant term while being used raw in the tangent term,
     * so the same variable was read as two different units inside one expression.
     *
     * <p>The surface-speed to RPM conversion is {@code v * 60 / (pi * D)} with D in meters. Since
     * {@link Constants.Shooter#ShooterDiameter} is in inches, the numerator carries the inches per
     * meter factor; the old literal 720 was short by a factor of ~3.3.
     *
     * @param distance Horizontal range to the target, in meters
     * @param angle Launch angle in radians (see {@link SUB_Hood#findoptimalangle})
     * @return Required flywheel RPM, or 0 if the shot is not physically reachable
     */
    /**
     * Lowest launch speed, in m/s, that can reach a target {@code distance} away and
     * {@link Constants.Hood#ScoreHeight} above the shooter: {@code sqrt(g*(h + hypot(h, d)))}.
     *
     * <p>This is the speed the {@link SUB_Hood#findoptimalangle} launch angle corresponds to, which
     * makes it the anchor point the RPM look-up table was tuned at.
     */
    public static double minLaunchSpeed(final double distance) {
        final double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        return Math.sqrt(Constants.Shooter.kGRAVITATIONAL_CONSTANT
            * (height + Math.hypot(height, distance)));
    }

    /**
     * Launch speed, in m/s, needed to reach {@code distance} at a given launch {@code angle} in
     * radians. Returns {@link Double#NaN} for a shot no speed can make.
     */
    public static double requiredLaunchSpeed(final double distance, final double angle) {
        final double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
        final double denominator = 2 * (distance * Math.tan(angle) - height);
        if (denominator <= 0) {
            return Double.NaN;
        }
        return (1 / Math.cos(angle))
            * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance) / denominator);
    }

    /**
     * Flywheel RPM needed to reach {@code distance} at a given hood {@code angle} in radians.
     *
     * <p>Anchored on the tuned look-up table rather than on absolute physics. The table is sampled
     * along the minimum-energy angle, so this scales its value by the <em>ratio</em> of the speed
     * this angle needs to the speed that angle needs:
     *
     * <pre>rpm(d, theta) = lut(d) * requiredLaunchSpeed(d, theta) / minLaunchSpeed(d)</pre>
     *
     * <p>Taking a ratio at nearly the same operating point cancels most of the wheel-to-ball
     * transfer error. That matters here: fitting the table against ideal projectile motion gives
     * RPM-per-m/s rising from ~178 at 1.6 m to ~221 at 10.5 m, while {@link #findoptimalRPM}
     * assumes a flat 313. The slip is real and speed-dependent, so absolute predictions from the
     * analytic model are not usable — but ratios near a calibrated point are.
     *
     * @return Required RPM, or {@link Double#NaN} if the shot is unreachable at that angle
     */
    public double requiredRPM(final double distance, final double angle) {
        final double needed = requiredLaunchSpeed(distance, angle);
        if (Double.isNaN(needed)) {
            return Double.NaN;
        }
        return distanceToRPM.get(distance) * (needed / minLaunchSpeed(distance));
    }

    /**
     * Lowest standing RPM band that can still reach {@code distance}.
     *
     * <p>A band is viable when it is at least the table's tuned RPM for that range — below that no
     * hood angle exists, since the tuned value already sits at the minimum-energy angle. Bands are
     * held with hysteresis so a robot hovering on a boundary does not oscillate between two
     * flywheel speeds.
     */
    public double readyRPM(final double distance) {
        final double[] bands = Constants.Shooter.kREADY_RPM_BANDS;
        final double needed = distanceToRPM.get(distance);

        // Stay in the current band while it still reaches, minus a margin, so the boundary has to
        // be cleared properly before stepping down.
        if (bands[currentBand] >= needed
            && (currentBand == 0
                || bands[currentBand - 1] < needed * Constants.Shooter.kBAND_HYSTERESIS)) {
            return bands[currentBand];
        }

        for (int i = 0; i < bands.length; i++) {
            if (bands[i] >= needed) {
                currentBand = i;
                return bands[i];
            }
        }
        // Out of range of every band: run the top one and let the caller see the shot is short.
        currentBand = bands.length - 1;
        return bands[currentBand];
    }

    /** @return The RPM most recently commanded, which the hood solves its angle against */
    public double getTargetRPM() {
        return desiredSpeed;
    }

    /**
     * Purely analytic RPM solve.
     *
     * @deprecated Its absolute predictions do not match the robot — fitting {@link #distanceToRPM}
     *     against ideal projectile motion shows the wheel-to-ball transfer varies with speed, while
     *     this assumes it is constant. Kept for reference and unit tests. Use {@link #requiredRPM}
     *     or {@link #shootMeters}, both of which are anchored on the tuned table.
     */
    @Deprecated
    public static double findoptimalRPM(final double distance, final double angle) {
        final double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);

        // The projectile must still be rising to the target at this range; otherwise there is no
        // real solution and the square root would produce NaN.
        final double denominator = 2 * (distance * Math.tan(angle) - height);
        if (denominator <= 0) {
            return 0;
        }

        final double exitVelocity = (1 / Math.cos(angle))
            * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance) / denominator);

        final double inchesPerMinutePerMeterPerSecond = 60.0 * Units.metersToInches(1.0);
        return (inchesPerMinutePerMeterPerSecond * exitVelocity)
            / (Constants.Shooter.kSHOOTER_COMPRESSION_RATIO * Math.PI * Constants.Shooter.ShooterDiameter);
    }

    @Deprecated
    public void set(final double speed) {
        MotorOne.set(speed);
    }

    /** @param rpm Target velocity for both flywheels */
    public void setRPM(final double rpm) {
        this.desiredSpeed = rpm;
        MotorOne.setControl(m_request.withVelocity(rpm / 60.0));
    }

    /** @return Average current RPM of both flywheels */
    public double flywheelRPM() {
        return (MotorOne.getVelocity().getValue().in(RPM) + MotorTwo.getVelocity().getValue().in(RPM)) / 2;
    }
  
    /** @return true if flywheels are within the tolerance of the target RPM */
    public boolean atDesiredRPM() {
        return Math.abs(flywheelRPM() - desiredSpeed) < 75;
    }

    /**
     * Sets target RPM based on distance to hub.
     * @param meters Distance to target in meters
     */
    public void shootMeters(final double meters) {
        double targetRPM = distanceToRPM.get(meters);
        setRPM(targetRPM);
    }

    /**
     * Holds the standing band for this range instead of chasing an exact per-distance RPM.
     *
     * <p>The hood covers the difference — see {@link SUB_Hood#angleForRPM}. Within a band a range
     * change costs a hood move rather than a flywheel spin-up, so consecutive shots at different
     * distances do not each pay a settling delay.
     *
     * @param meters Distance to target in meters
     */
    public void holdReadyBand(final double meters) {
        setRPM(readyRPM(meters));
    }

    /** @return The tuned table RPM for this range, which is the band solve's floor */
    public double tunedRPM(final double meters) {
        return distanceToRPM.get(meters);
    }

    /** 
     * Calibration utility for autonomous logic.
     * @param meters Distance to target in meters
     * @return Required RPM from the look-up table
     */
    public double getDistanceRPM (final double meters) {
        return distanceToRPM.get(meters);
    }

    /** Stops both flywheels */
    public void stop() {
        this.desiredSpeed = 0;
        MotorOne.setControl(voltageRequest.withOutput(0));
    }

    /** @param volts Direct voltage output for manual testing */
    public void setVolts(final double volts) {
        MotorOne.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void periodic() {
      // Telemetry logging for dashboard and diagnostics
      SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
      SmartDashboard.putNumber("Shooter/Motor One Stator Current", MotorOne.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Stator Current", MotorTwo.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Supply Current", MotorOne.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Supply Current", MotorTwo.getSupplyCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Supply Voltage", MotorOne.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Supply Voltage", MotorTwo.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Voltage", MotorOne.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Voltage", MotorTwo.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor One Encoder Pos", MotorOne.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Encoder Pos", MotorTwo.getPosition().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/Motor One Torque Current", MotorOne.getTorqueCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Torque Current", MotorTwo.getTorqueCurrent().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/Motor One Device Temp", MotorOne.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Device Temp", MotorTwo.getDeviceTemp().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/Motor One Processor Temp", MotorOne.getProcessorTemp().getValueAsDouble());
      SmartDashboard.putNumber("Shooter/Motor Two Processor Temp", MotorTwo.getProcessorTemp().getValueAsDouble());

      SmartDashboard.putNumber("Shooter/FlywheelRPM (One)", MotorOne.getVelocity().getValue().in(RPM));
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Two)", MotorTwo.getVelocity().getValue().in(RPM));

      SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());
      
      SmartDashboard.putBoolean("Shooter/High Power Limits", highPower);

      Alert.alertKraken(MotorOne);
      Alert.alertKraken(MotorTwo);
    }


    /** 
     * Calculates time-of-flight based on projectile physics.
     * @param distanceMeters Distance to target
     * @return Estimated seconds until impact
     */
    public double getExpectedTOF(final double distanceMeters) {
        return distanceMeters*0.215298795+0.753755412;
    }
}
