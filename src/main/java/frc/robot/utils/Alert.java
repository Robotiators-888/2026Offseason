package frc.robot.utils;

import java.util.HashMap;

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

  public Alert() {
  }

  public static void setup () {
    updateSmartDashboard();
  }

  public static void registerError (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!errorMap.containsKey(alert)) {
      errorMap.put(alert, 0);
      error.add(alert);
      notifyError(alert);
      updateSmartDashboard();
      // triggerStop();
    }
  }

  public static void registerWarning (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!warningMap.containsKey(alert)) {
      warningMap.put(alert, 0);
      warning.add(alert);
      notifyWarning(alert);
      updateSmartDashboard();
    }
  }

  public static void registerInfo (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!infoMap.containsKey(alert)) {
      infoMap.put(alert, 0);
      info.add(alert);
      notifyInfo(alert);
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

  public static void triggerStop () { // Stops the robot from running if it has errors (This will not be used because errors shouldn't kill the robot)
    // Todo: implement this
  }

  private static void testcalls () {
    registerWarning("There may be an issue");
    notifyWarning("Photonvision has optional type idk");
  }

  private static void updateSmartDashboard () {
    SmartDashboard.putStringArray("Alert/errors", error.toArray());
    SmartDashboard.putStringArray("Alert/warnings", warning.toArray());
    SmartDashboard.putStringArray("Alert/info", info.toArray());
    registerColor();
  }
}