package frc.robot.utils;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Alert {
  // ArrayList required akward casting to work so I created Vector
  // Hashmaps to check if keys are present in O(1) time!
  private static Elastic.Notification notification = new Elastic.Notification(); // Creates one notification object that can be method chained on to increase garbage collection performance
  private static Vector<String> error = new Vector<String>(new String());
  private static Vector<String> warning = new Vector<String>(new String());
  private static Vector<String> info = new Vector<String>(new String());
  private static HashMap<String, Integer> errorMap = new HashMap<String, Integer>();
  private static HashMap<String, Integer> warningMap = new HashMap<String, Integer>();
  private static HashMap<String, Integer> infoMap = new HashMap<String, Integer>();
  private static Color alertColor = new Color(0, 255, 0); // Green
  private static final double connectedTimeout = .5; // Seconds

  public Alert() {
  }

  public static void setup () {
    updateSmartDashboard();
  }

  public static void registerError (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!errorMap.containsKey(alert)) {
      errorMap.put(alert, 0);
      error.add(alert + " 0");
      notifyError(alert);
      updateSmartDashboard();
      // triggerStop();
    }
    else {
      // Stores the value at the hashmap of the alert
      Integer mapVal = errorMap.get(alert);
      // Updates the hashmap value
      errorMap.put(alert, mapVal+1);
      // Finds the alert in the vector and then updates it
      error.set(error.findFirst(comparees -> comparees.get_0().equals(comparees.get_1()), alert + " " + mapVal), alert + " " + (mapVal + 1));
      // Doesn't notify elastic as this would spam notifications
      updateSmartDashboard();
    }
  }

  public static void registerWarning (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!warningMap.containsKey(alert)) {
      warningMap.put(alert, 0);
      warning.add(alert + " 0");
      notifyWarning(alert);
      updateSmartDashboard();
    }
    else {
      // Stores the value at the hashmap of the alert
      Integer mapVal = warningMap.get(alert);
      // Updates the hashmap value
      warningMap.put(alert, mapVal+1);
      // Finds the alert in the vector and then updates it
      warning.set(warning.findFirst(comparees -> comparees.get_0().equals(comparees.get_1()), alert + " " + mapVal), alert + " " + (mapVal + 1));
      // Doesn't notify elastic as this would spam notifications
      updateSmartDashboard();
    }
  }

  public static void registerInfo (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!infoMap.containsKey(alert)) {
      infoMap.put(alert, 0);
      info.add(alert + " 0");
      notifyInfo(alert);
      updateSmartDashboard();
    }
    else {
      // Stores the value at the hashmap of the alert
      Integer mapVal = infoMap.get(alert);
      // Updates the hashmap value
      infoMap.put(alert, mapVal+1);
      // Finds the alert in the vector and then updates it
      info.set(info.findFirst(comparees -> comparees.get_0().equals(comparees.get_1()), alert + " " + mapVal), alert + " " + (mapVal + 1));
      // Doesn't notify elastic as this would spam notifications
      updateSmartDashboard();
    }
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
    if (!error.isEmpty()) {
      alertColor = new Color(255, 0, 0); // Red
    }
    else if (!warning.isEmpty()) {
      alertColor = new Color(255, 255, 0); // Yellow
    }
    else {
      alertColor = new Color(0, 255, 0); // Green
    }
    SmartDashboard.putString("Alert/Alerts", alertColor.toHexString());
  }

  // public static void triggerStop () { // Stops the robot from running if it has errors (This will not be used because errors shouldn't kill the robot)
    // Todo: implement this
  // }

  // private static void testcalls () {
  //   registerWarning("There may be an issue");
  //   notifyWarning("Photonvision has optional type idk");
  // }

  private static void updateSmartDashboard () {
    SmartDashboard.putStringArray("Alert/errors", error.toArray());
    SmartDashboard.putStringArray("Alert/warnings", warning.toArray());
    SmartDashboard.putStringArray("Alert/info", info.toArray());
    registerColor();
  }

  // Alerts every fault a kraken could have in just about the worst way possible, but there is no better way
  public static void alertKraken (TalonFX kraken) {
    int krakenId = kraken.getDeviceID();
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
}