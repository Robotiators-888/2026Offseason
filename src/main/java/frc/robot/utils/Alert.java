package frc.robot.utils;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Alert {
  private static Alert INSTANCE = null;
  // Static is likley not needed
  private static Elastic.Notification notification = new Elastic.Notification(); // Creates one notification object that can be method chained on to increase garbage collection performance
  Vector<String> error;
  Vector<String> warning;
  Vector<String> info;
  HashMap<String, Integer> errorMap;
  HashMap<String, Integer> warningMap;
  HashMap<String, Integer> infoMap;
  Color alertColor;

  private Alert() {
    alertColor = new Color(0, 255, 0); // Green
    // ArrayList required akward casting to work so I created Vector
    error = new Vector<String>(new String());
    warning = new Vector<String>(new String());
    info = new Vector<String>(new String());
    // Hashmaps to check if keys are present in O(1) time!
    errorMap = new HashMap<String, Integer>();
    warningMap = new HashMap<String, Integer>();
    infoMap = new HashMap<String, Integer>();
    updateSmartDashboard();
    // testcalls();
  }

  public static Alert getInstance () {
    if (INSTANCE == null) {
      INSTANCE = new Alert();
    }
    return INSTANCE;
  }

  public void registerError (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!errorMap.containsKey(alert)) {
      errorMap.put(alert, 0);
      error.add(alert);
      notifyError(alert);
      updateSmartDashboard();
      // triggerStop();
    }
  }

  public void registerWarning (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!warningMap.containsKey(alert)) {
      warningMap.put(alert, 0);
      warning.add(alert);
      notifyWarning(alert);
      updateSmartDashboard();
    }
  }

  public void registerInfo (String alert) {
    // Makes sure there are no duplicates using the hashmap
    if (!infoMap.containsKey(alert)) {
      infoMap.put(alert, 0);
      info.add(alert);
      notifyInfo(alert);
      updateSmartDashboard();
    }
  }

  // Don't use direct notifications on things that spam them use them for things like telop init, things that happen multiple times, infrequently
  public void notifyError (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.ERROR)
      .withTitle("Error!")
      .withDescription(alert)
    );
  }

  public void notifyWarning (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.WARNING)
      .withTitle("Warning:")
      .withDescription(alert)
    );
  }

  public void notifyInfo (String alert) {
    Elastic.sendNotification(notification
      .withLevel(Elastic.Notification.NotificationLevel.INFO)
      .withTitle("Info")
      .withDescription(alert)
    );
  }

  // Sets the single color elastic object to the highest severity level (severity level meaning either info, error or warning) that the robot has at least one alert for (check engine light)
  private void registerColor () {
    if (!error.isEmpty()) {
      alertColor = new Color(255, 0, 0); // Red
    }
    else if (!warning.isEmpty()) {
      alertColor = new Color(255, 255, 0); // Yellow
    }
    else {
      alertColor = new Color(0, 255, 0); // Green
    }
    SmartDashboard.putString("Alerts", alertColor.toHexString());
  }

  public void triggerStop () { // Stops the robot from running if it has errors (This will not be used because errors shouldn't kill the robot)
    // Todo: implement this
  }

  private void testcalls () {
    registerWarning("There may be an issue");
    notifyWarning("Photonvision has optional type idk");
  }

  private void updateSmartDashboard () {
    SmartDashboard.putStringArray("errors", error.toArray());
    SmartDashboard.putStringArray("warnings", warning.toArray());
    SmartDashboard.putStringArray("info", info.toArray());
    registerColor();
  }
}