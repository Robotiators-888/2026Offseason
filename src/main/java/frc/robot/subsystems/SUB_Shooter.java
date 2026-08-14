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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Alert;

public class SUB_Shooter extends SubsystemBase {
    private static SUB_Shooter INSTANCE = null;

    /** Subsystem hardware and control state */
    private final TalonFX shooterLeader;
    private final TalonFX shooterFollower;
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

    /**
     * The profile the last {@link #setHighPower} call asked for. Reconciled against
     * {@link #highPower} in {@link #periodic()} with non-blocking applies — a blocking apply in
     * a command's initialize()/end() used to stall the whole loop by up to ~100 ms per motor.
     */
    private boolean requestedHighPower = false;

    /** Index into {@link Constants.Shooter#kREADY_RPM_BANDS}, held with hysteresis. */
    private int currentBand = 0;

    /**
     * Sustained sign disagreement between leader and follower means the Follower request's
     * MotorAlignmentValue is wrong for this gearbox — {@link #flywheelRPM()} would average two
     * opposing speeds toward zero and the robot would silently never fire.
     */
    private final Debouncer followerOpposedDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);

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
        shooterLeader = new TalonFX(Constants.Shooter.kSHOOTER_LEADER_MOTOR_CANID); 
        shooterFollower = new TalonFX(Constants.Shooter.kSHOOTER_FOLLOWER_MOTOR_CANID);

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

        shooterLeader.getConfigurator().apply(shooterConfig);
        shooterFollower.getConfigurator().apply(shooterConfig);

        // Synchronize bottom flywheel to top flywheel
        shooterFollower.setControl(new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * Selects the shooting (high supply current) or idle (low supply current) limit profile.
     *
     * <p>Every command that spins the flywheel to score must call this with {@code true} in
     * {@code initialize()} and {@code false} in {@code end()}. Only the request is recorded here;
     * {@link #periodic()} pushes it to the motors with zero-timeout (non-blocking) applies, so
     * calling this can never stall the command scheduler loop. The one-loop delay is immaterial
     * against a ~1 s spin-up.
     *
     * @param enabled true while actively shooting
     */
    public void setHighPower(final boolean enabled) {
        requestedHighPower = enabled;
    }

    /** Pushes a pending limit-profile change to both motors without blocking; retried until acked. */
    private void reconcileCurrentLimits() {
        if (requestedHighPower == highPower) {
            return;
        }
        final CurrentLimitsConfigs limits = requestedHighPower ? highPowerLimits : lowPowerLimits;
        final boolean leaderOk = shooterLeader.getConfigurator().apply(limits, 0.0).isOK();
        final boolean followerOk = shooterFollower.getConfigurator().apply(limits, 0.0).isOK();
        if (leaderOk && followerOk) {
            highPower = requestedHighPower;
        }
    }

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
     * Lowest standing RPM band that can still reach {@code distance}.
     *
     * <p>A band is viable when it is at least the table's tuned RPM for that range — below that no
     * hood angle exists, since the tuned value already sits at the minimum-energy angle. Bands are
     * held with hysteresis so a robot hovering on a boundary does not oscillate between two
     * flywheel speeds.
     */
    public double readyRPM(final double distance) {
        final double[] bands = Constants.Shooter.kREADY_RPM_BANDS;
        currentBand = selectBand(bands, distanceToRPM.get(distance), currentBand,
            Constants.Shooter.kBAND_HYSTERESIS);
        return bands[currentBand];
    }

    /**
     * Band selection as a pure function, so the hysteresis behaviour is unit-testable.
     *
     * <p>Stays in {@code currentBand} while it still reaches and, on the way down, until the
     * needed RPM drops below {@code hysteresis} of the band below. Otherwise a robot hovering on
     * a band boundary steps the flywheel up and down on every odometry jitter — which is exactly
     * what the previous comparison allowed: it divided where it should have multiplied, making
     * the early return provably identical to the plain scan (no hysteresis at all).
     *
     * @param bands ascending standing speeds
     * @param needed the table RPM for the current range
     * @param currentBand the band currently held
     * @param hysteresis fraction of the lower band's speed the requirement must drop below
     *     before stepping down
     * @return index of the band to run; the top band with nothing reaching means the shot is out
     *     of range (see {@link #canReach})
     */
    public static int selectBand(final double[] bands, final double needed, final int currentBand,
            final double hysteresis) {
        if (bands[currentBand] >= needed
            && (currentBand == 0 || needed > bands[currentBand - 1] * hysteresis)) {
            return currentBand;
        }
        for (int i = 0; i < bands.length; i++) {
            if (bands[i] >= needed) {
                return i;
            }
        }
        // Out of range of every band: run the top one and let the caller see the shot is short.
        return bands.length - 1;
    }

    /**
     * @return true when some standing band reaches {@code distanceMeters}. Beyond the top band
     *     the aim commands must not feed — the flywheel pre-spins at the top band regardless,
     *     but the shot is guaranteed short.
     */
    public boolean canReach(final double distanceMeters) {
        final double[] bands = Constants.Shooter.kREADY_RPM_BANDS;
        return distanceToRPM.get(distanceMeters) <= bands[bands.length - 1];
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
     *     this assumes it is constant. Kept for reference and unit tests. Use {@link #shootMeters}
     *     or the banded {@link #holdReadyBand}, both of which are anchored on the tuned table.
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

    /** @param rpm Target velocity for both flywheels */
    public void setRPM(final double rpm) {
        this.desiredSpeed = rpm;
        shooterLeader.setControl(m_request.withVelocity(rpm / 60.0));
    }

    /** @return Average current RPM of both flywheels */
    public double flywheelRPM() {
        return (shooterLeader.getVelocity().getValue().in(RPM) + shooterFollower.getVelocity().getValue().in(RPM)) / 2;
    }
  
    /**
     * @return true if a nonzero target is set and the flywheels are within tolerance of it. The
     *     nonzero guard matters: a stopped shooter at a zero setpoint used to count as "at
     *     speed", so auto shots skipped their spin-up and fed balls into a dead flywheel.
     */
    public boolean atDesiredRPM() {
        return desiredSpeed > 0
            && Math.abs(flywheelRPM() - desiredSpeed) < Constants.Shooter.kRPM_TOLERANCE;
    }

    /** @return Signed difference between measured and target RPM, for the feed-sustain check */
    public double rpmError() {
        return flywheelRPM() - desiredSpeed;
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

    /** Stops both flywheels */
    public void stop() {
        this.desiredSpeed = 0;
        shooterLeader.setControl(voltageRequest.withOutput(0));
    }

    /** @param volts Direct voltage output for manual testing */
    public void setVolts(final double volts) {
        shooterLeader.setControl(voltageRequest.withOutput(volts));
    }

    /** Loop counter used to decimate the diagnostic telemetry below to ~4 Hz. */
    private int telemetryTick = 0;

    @Override
    public void periodic() {
      reconcileCurrentLimits();

      // Fire-gate signals stay at full rate — they are what anyone debugs a no-shoot with.
      final double leaderRPM = shooterLeader.getVelocity().getValue().in(RPM);
      final double followerRPM = shooterFollower.getVelocity().getValue().in(RPM);
      SmartDashboard.putNumber("Shooter/Desired RPM", desiredSpeed);
      SmartDashboard.putNumber("Shooter/FlywheelRPM (One)", leaderRPM);
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Two)", followerRPM);
      SmartDashboard.putNumber("Shooter/FlywheelRPM (Average)", flywheelRPM());
      SmartDashboard.putBoolean("Shooter/High Power Limits", highPower);

      if (followerOpposedDebouncer.calculate(
              Math.abs(leaderRPM) > 300 && Math.abs(followerRPM) > 300
                  && Math.signum(followerRPM) != Math.signum(leaderRPM))) {
        Alert.registerError("Shooter follower (44) opposing leader — MotorAlignmentValue likely wrong");
      }

      // Slow-moving diagnostics at ~4 Hz.
      if (telemetryTick++ % 12 == 0) {
        SmartDashboard.putNumber("Shooter/Motor One Stator Current", shooterLeader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Stator Current", shooterFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor One Supply Current", shooterLeader.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Supply Current", shooterFollower.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor One Supply Voltage", shooterLeader.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Supply Voltage", shooterFollower.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor One Voltage", shooterLeader.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Voltage", shooterFollower.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor One Encoder Pos", shooterLeader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Encoder Pos", shooterFollower.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motor One Torque Current", shooterLeader.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Torque Current", shooterFollower.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motor One Device Temp", shooterLeader.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Device Temp", shooterFollower.getDeviceTemp().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Motor One Processor Temp", shooterLeader.getProcessorTemp().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Two Processor Temp", shooterFollower.getProcessorTemp().getValueAsDouble());
      }

      Alert.alertKraken(shooterLeader);
      Alert.alertKraken(shooterFollower);
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
