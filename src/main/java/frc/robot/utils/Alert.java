package frc.robot.utils;

import java.util.LinkedHashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Central alert surface. Messages are deduplicated and counted, published to SmartDashboard as
 * three string arrays plus a check-engine colour, and toasted to Elastic on first occurrence only.
 *
 * <p>Each severity is a single LinkedHashMap from message to occurrence count. This replaces a
 * parallel Vector + HashMap pair that had to be kept in sync by hand: a miss in the lookup returned
 * -1, which was fed straight into an unchecked array write and threw
 * {@code ArrayIndexOutOfBoundsException} out of {@code robotPeriodic()}. Insertion order is
 * preserved, so the dashboard ordering is unchanged.
 */
public class Alert {
  private static final Elastic.Notification notification = new Elastic.Notification(); // Creates one notification object that can be method chained on to increase garbage collection performance
  private static final Map<String, Integer> errors = new LinkedHashMap<>();
  private static final Map<String, Integer> warnings = new LinkedHashMap<>();
  private static final Map<String, Integer> infos = new LinkedHashMap<>();
  private static Color alertColor = new Color(0, 255, 0); // Green
  private static final double connectedTimeout = .5; // Seconds

  /** Loop counter, advanced once per loop by {@link #periodic()}, used to stagger fault scans. */
  private static int tick = 0;

  /**
   * Loops between fault scans of any one device: 12 loops is ~4 Hz, matching the default refresh
   * rate of Phoenix fault signals — scanning faster only re-reads unchanged data.
   */
  private static final int kFaultScanPeriodLoops = 12;

  /** Loops between routine dashboard re-renders (which refresh the occurrence counts). */
  private static final int kRenderPeriodLoops = 50;

  /** Set when a NEW message arrives; makes the next periodic() render immediately. */
  private static boolean dirty = false;

  public static void setup () {
    updateSmartDashboard();
  }

  /**
   * Advances the fault-scan stagger and renders the dashboard when something changed (or at a
   * slow steady rate, to refresh occurrence counts). Call exactly once per loop from
   * {@code Robot.robotPeriodic()}. Rendering here instead of inside every register call matters:
   * one disconnected camera used to rebuild the entire alert surface three times per loop.
   */
  public static void periodic () {
    tick++;
    if (dirty || tick % kRenderPeriodLoops == 0) {
      dirty = false;
      updateSmartDashboard();
    }
  }

  public static void registerError (String alert) {
    if (record(errors, alert)) {
      notifyError(alert);
    }
  }

  public static void registerWarning (String alert) {
    if (record(warnings, alert)) {
      notifyWarning(alert);
    }
  }

  public static void registerInfo (String alert) {
    if (record(infos, alert)) {
      notifyInfo(alert);
    }
  }

  /**
   * Counts an occurrence of {@code alert}.
   *
   * @return true if this is the first time the message has been seen, i.e. it warrants a toast.
   *     Repeats only bump the counter, otherwise a flapping fault would spam the dashboard.
   */
  private static boolean record (final Map<String, Integer> into, final String alert) {
    // getOrDefault(0) + 1, so the first occurrence renders as "<message> 1", not "<message> 0".
    final Integer previous = into.put(alert, into.getOrDefault(alert, 0) + 1);
    if (previous == null) {
      dirty = true;
      return true;
    }
    return false;
  }

  /** Renders a severity's map as the "<message> <count>" array the dashboard expects. */
  private static String[] render (final Map<String, Integer> from) {
    return from.entrySet().stream()
        .map(entry -> entry.getKey() + " " + entry.getValue())
        .toArray(String[]::new);
  }

  // Don't use direct notifications on things that spam them use them for things like telop init, things that happen multiple times, infrequently
  public static void notifyError (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.ERROR)
      .withTitle("Error!")
      .withDescription(alert)
    );
  }

  public static void notifyWarning (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.WARNING)
      .withTitle("Warning:")
      .withDescription(alert)
    );
  }

  public static void notifyInfo (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.INFO)
      .withTitle("Info")
      .withDescription(alert)
    );
  }

  // Sets the single color elastic object to the highest severity level (severity level meaning either info, error or warning) that the robot has at least one alert for (check engine light)
  private static void registerColor () {
    if (!errors.isEmpty()) {
      alertColor = new Color(255, 0, 0); // Red
    }
    else if (!warnings.isEmpty()) {
      alertColor = new Color(255, 255, 0); // Yellow
    }
    else {
      alertColor = new Color(0, 255, 0); // Green
    }
    SmartDashboard.putString("Alert/Alerts", alertColor.toHexString());
  }

  private static void updateSmartDashboard () {
    SmartDashboard.putStringArray("Alert/errors", render(errors));
    SmartDashboard.putStringArray("Alert/warnings", render(warnings));
    SmartDashboard.putStringArray("Alert/info", render(infos));
    registerColor();
  }

  // Alerts every fault a kraken could have in just about the worst way possible, but there is no better way
  public static void alertKraken (TalonFX kraken) {
    int krakenId = kraken.getDeviceID();
    // Staggered by device ID: each of the ~26 signal reads below runs at ~4 Hz per device
    // instead of every device paying them every loop (~390 reads per 20 ms across the robot).
    if (tick % kFaultScanPeriodLoops != krakenId % kFaultScanPeriodLoops) {
      return;
    }
    if (!kraken.isAlive())
      registerError("Motor " + krakenId + " is not alive");
    if (!kraken.isConnected(connectedTimeout))
      registerError("Motor " + krakenId + " has been disconnected for " + connectedTimeout + " sec");
    if (kraken.getFault_BootDuringEnable().getValue())
      registerError("Motor " + krakenId + " Fault_BootDuringEnable");
    if (kraken.getFault_BridgeBrownout().getValue())
      registerError("Motor " + krakenId + " Fault_BridgeBrownout");
    if (kraken.getFault_DeviceTemp().getValue())
      registerError("Motor " + krakenId + " Fault_DeviceTemp");
    if (kraken.getFault_ForwardHardLimit().getValue())
      registerError("Motor " + krakenId + " Fault_ForwardHardLimit");
    if (kraken.getFault_ForwardSoftLimit().getValue())
      registerError("Motor " + krakenId + " Fault_ForwardSoftLimit");
    if (kraken.getFault_FusedSensorOutOfSync().getValue())
      registerError("Motor " + krakenId + " Fault_FusedSensorOutOfSync");
    if (kraken.getFault_Hardware().getValue())
      registerError("Motor " + krakenId + " Fault_Hardware");
    if (kraken.getFault_MissingDifferentialFX().getValue())
      registerError("Motor " + krakenId + " Fault_MissingDifferentialFX");
    if (kraken.getFault_MissingHardLimitRemote().getValue())
      registerError("Motor " + krakenId + " Fault_MissingHardLimitRemote");
    if (kraken.getFault_MissingSoftLimitRemote().getValue())
      registerError("Motor " + krakenId + " Fault_MissingSoftLimitRemote");
    if (kraken.getFault_OverSupplyV().getValue())
      registerError("Motor " + krakenId + " Fault_OverSupplyV");
    if (kraken.getFault_ProcTemp().getValue())
      registerError("Motor " + krakenId + " Fault_ProcTemp");
    if (kraken.getFault_RemoteSensorDataInvalid().getValue())
      registerError("Motor " + krakenId + " Fault_RemoteSensorDataInvalid");
    if (kraken.getFault_RemoteSensorPosOverflow().getValue())
      registerError("Motor " + krakenId + " Fault_RemoteSensorPosOverflow");
    if (kraken.getFault_RemoteSensorReset().getValue())
      registerError("Motor " + krakenId + " Fault_RemoteSensorReset");
    if (kraken.getFault_ReverseHardLimit().getValue())
      registerError("Motor " + krakenId + " Fault_ReverseHardLimit");
    if (kraken.getFault_ReverseSoftLimit().getValue())
      registerError("Motor " + krakenId + " Fault_ReverseSoftLimit");
    if (kraken.getFault_StaticBrakeDisabled().getValue())
      registerError("Motor " + krakenId + " Fault_StaticBrakeDisabled");
    if (kraken.getFault_StatorCurrLimit().getValue())
      registerError("Motor " + krakenId + " Fault_StatorCurrLimit");
    if (kraken.getFault_SupplyCurrLimit().getValue())
      registerError("Motor " + krakenId + " Fault_SupplyCurrLimit");
    if (kraken.getFault_Undervoltage().getValue())
      registerError("Motor " + krakenId + " Fault_Undervoltage");
    if (kraken.getFault_UnlicensedFeatureInUse().getValue())
      registerError("Motor " + krakenId + " Fault_UnlicensedFeatureInUse");
    if (kraken.getFault_UnstableSupplyV().getValue())
      registerError("Motor " + krakenId + " Fault_UnstableSupplyV");
    if (kraken.getFault_UsingFusedCANcoderWhileUnlicensed().getValue())
      registerError("Motor " + krakenId + " Fault_UsingFusedCANcoderWhileUnlicensed");
  }

  public static void alertNeoFaults (SparkMax neo) {
    int neoId = neo.getDeviceId();
    // One read of the struct rather than eight round trips to the controller.
    final SparkBase.Faults faults = neo.getFaults();
    if (faults.can)
      registerError("Motor " + neoId + " Fault: can");
    if (faults.escEeprom)
      registerError("Motor " + neoId + " Fault: escEeprom");
    if (faults.firmware)
      registerError("Motor " + neoId + " Fault: firmware");
    if (faults.gateDriver)
      registerError("Motor " + neoId + " Fault: gateDriver");
    if (faults.motorType)
      registerError("Motor " + neoId + " Fault: motorType");
    if (faults.other)
      registerError("Motor " + neoId + " Fault: other");
    if (faults.sensor)
      registerError("Motor " + neoId + " Fault: sensor");
    if (faults.temperature)
      registerError("Motor " + neoId + " Fault: temperature");
  }

  public static void alertNeoWarnings (SparkMax neo) {
    int neoId = neo.getDeviceId();
    final SparkBase.Warnings warnings = neo.getWarnings();
    if (warnings.brownout)
      registerWarning("Motor " + neoId + " Warning: brownout");
    if (warnings.escEeprom)
      registerWarning("Motor " + neoId + " Warning: escEeprom");
    if (warnings.extEeprom)
      registerWarning("Motor " + neoId + " Warning: extEeprom");
    if (warnings.hasReset)
      registerWarning("Motor " + neoId + " Warning: hasReset");
    if (warnings.other)
      registerWarning("Motor " + neoId + " Warning: other");
    if (warnings.overcurrent)
      registerWarning("Motor " + neoId + " Warning: overcurrent");
    if (warnings.sensor)
      registerWarning("Motor " + neoId + " Warning: sensor");
    if (warnings.stall)
      registerWarning("Motor " + neoId + " Warning: stall");
  }
}