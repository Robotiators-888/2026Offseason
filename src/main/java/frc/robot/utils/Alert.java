package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Alert {
  private static final Elastic.Notification notification = new Elastic.Notification();
  private static final Map<String, Integer> errorMap = Collections.synchronizedMap(new LinkedHashMap<>());
  private static final Map<String, Integer> warningMap = Collections.synchronizedMap(new LinkedHashMap<>());
  private static final Map<String, Integer> infoMap = Collections.synchronizedMap(new LinkedHashMap<>());
  
  private static Color alertColor = new Color(0, 255, 0); // Green
  private static final double connectedTimeout = .5; // Seconds
  private static boolean dirty = true;

  public Alert() {}

  public static void setup() {
    updateSmartDashboard();
  }

  public static void registerError(String alert) {
    if (!errorMap.containsKey(alert)) {
      errorMap.put(alert, 1);
      notifyError(alert);
      dirty = true;
      updateSmartDashboard();
    } else {
      int count = errorMap.get(alert) + 1;
      errorMap.put(alert, count);
    }
  }

  public static void registerWarning(String alert) {
    if (!warningMap.containsKey(alert)) {
      warningMap.put(alert, 1);
      notifyWarning(alert);
      dirty = true;
      updateSmartDashboard();
    } else {
      int count = warningMap.get(alert) + 1;
      warningMap.put(alert, count);
    }
  }

  public static void registerInfo(String alert) {
    if (!infoMap.containsKey(alert)) {
      infoMap.put(alert, 1);
      notifyInfo(alert);
      dirty = true;
      updateSmartDashboard();
    } else {
      int count = infoMap.get(alert) + 1;
      infoMap.put(alert, count);
    }
  }

  public static void notifyError(String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.ERROR)
      .withTitle("Error!")
      .withDescription(alert)
    );
  }

  public static void notifyWarning(String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.WARNING)
      .withTitle("Warning:")
      .withDescription(alert)
    );
  }

  public static void notifyInfo(String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.INFO)
      .withTitle("Info")
      .withDescription(alert)
    );
  }

  private static void registerColor() {
    if (!errorMap.isEmpty()) {
      alertColor = new Color(255, 0, 0); // Red
    } else if (!warningMap.isEmpty()) {
      alertColor = new Color(255, 255, 0); // Yellow
    } else {
      alertColor = new Color(0, 255, 0); // Green
    }
    SmartDashboard.putString("Alert/Alerts", alertColor.toHexString());
  }

  public static void updateSmartDashboard() {
    if (!dirty) return;
    
    List<String> errorsList = new ArrayList<>();
    for (Map.Entry<String, Integer> entry : errorMap.entrySet()) {
      errorsList.add(entry.getKey() + " (" + entry.getValue() + ")");
    }
    List<String> warningsList = new ArrayList<>();
    for (Map.Entry<String, Integer> entry : warningMap.entrySet()) {
      warningsList.add(entry.getKey() + " (" + entry.getValue() + ")");
    }
    List<String> infoList = new ArrayList<>();
    for (Map.Entry<String, Integer> entry : infoMap.entrySet()) {
      infoList.add(entry.getKey() + " (" + entry.getValue() + ")");
    }

    SmartDashboard.putStringArray("Alert/errors", errorsList.toArray(new String[0]));
    SmartDashboard.putStringArray("Alert/warnings", warningsList.toArray(new String[0]));
    SmartDashboard.putStringArray("Alert/info", infoList.toArray(new String[0]));
    registerColor();
    dirty = false;
  }

  /**
   * Checks Kraken/TalonFX faults efficiently using a single bitfield signal.
   */
  public static void alertKraken(TalonFX kraken) {
    int krakenId = kraken.getDeviceID();
    if (!kraken.isAlive()) {
      registerError("Motor " + krakenId + " is not alive");
      return;
    }
    if (!kraken.isConnected(connectedTimeout)) {
      registerError("Motor " + krakenId + " has been disconnected for " + connectedTimeout + " sec");
      return;
    }
    
    // Check all active faults in a single CAN signal call
    int faultField = kraken.getFaultField().getValue();
    if (faultField != 0) {
      if (kraken.getFault_Hardware().getValue()) registerError("Motor " + krakenId + " Fault_Hardware");
      if (kraken.getFault_DeviceTemp().getValue()) registerError("Motor " + krakenId + " Fault_DeviceTemp");
      if (kraken.getFault_BridgeBrownout().getValue()) registerError("Motor " + krakenId + " Fault_BridgeBrownout");
      if (kraken.getFault_Undervoltage().getValue()) registerError("Motor " + krakenId + " Fault_Undervoltage");
      if (kraken.getFault_BootDuringEnable().getValue()) registerError("Motor " + krakenId + " Fault_BootDuringEnable");
      if (kraken.getFault_StatorCurrLimit().getValue()) registerError("Motor " + krakenId + " Fault_StatorCurrLimit");
      if (kraken.getFault_SupplyCurrLimit().getValue()) registerError("Motor " + krakenId + " Fault_SupplyCurrLimit");
    }
  }

  public static void alertNeoFaults(SparkMax neo) {
    int neoId = neo.getDeviceId();
    var faults = neo.getFaults();
    if (faults.can) registerError("Motor " + neoId + " Fault: can");
    if (faults.escEeprom) registerError("Motor " + neoId + " Fault: escEeprom");
    if (faults.firmware) registerError("Motor " + neoId + " Fault: firmware");
    if (faults.gateDriver) registerError("Motor " + neoId + " Fault: gateDriver");
    if (faults.motorType) registerError("Motor " + neoId + " Fault: motorType");
    if (faults.other) registerError("Motor " + neoId + " Fault: other");
    if (faults.sensor) registerError("Motor " + neoId + " Fault: sensor");
    if (faults.temperature) registerError("Motor " + neoId + " Fault: temperature");
  }

  public static void alertNeoWarnings(SparkMax neo) {
    int neoId = neo.getDeviceId();
    var warnings = neo.getWarnings();
    if (warnings.brownout) registerWarning("Motor " + neoId + " Warning: brownout");
    if (warnings.escEeprom) registerWarning("Motor " + neoId + " Warning: escEeprom");
    if (warnings.extEeprom) registerWarning("Motor " + neoId + " Warning: extEeprom");
    if (warnings.hasReset) registerWarning("Motor " + neoId + " Warning: hasReset");
    if (warnings.other) registerWarning("Motor " + neoId + " Warning: other");
    if (warnings.overcurrent) registerWarning("Motor " + neoId + " Warning: overcurrent");
    if (warnings.sensor) registerWarning("Motor " + neoId + " Warning: sensor");
    if (warnings.stall) registerWarning("Motor " + neoId + " Warning: stall");
  }
}