package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import java.util.HashMap;

/**
 * Utility class for logging, reporting, and dashboard management of robot alerts, warnings, and errors.
 *
 * <p>Integrates with NetworkTables and Elastic dashboard notification UI.
 */
public class Alert {
        // ArrayList required akward casting to work so I created Vector
        // Hashmaps to check if keys are present in O(1) time!
        private static Elastic.Notification notification =
            new Elastic.Notification();
        private static Vector<String> error = new Vector<String>(new String());
        private static Vector<String> warning = new Vector<String>(new String());
        private static Vector<String> info = new Vector<String>(new String());
        private static HashMap<String, Integer> errorMap = new HashMap<String, Integer>();
        private static HashMap<String, Integer> warningMap = new HashMap<String, Integer>();
        private static HashMap<String, Integer> infoMap = new HashMap<String, Integer>();
        private static Color alertColor = new Color(0, 255, 0); // Green
        private static final double connectedTimeout = .5; // Seconds

        /** Default constructor for Alert utility. */
        public Alert() {}

        /**
         * Initializes dashboard alert entries.
         */
        public static void setup() {
                updateSmartDashboard();
        }

        /**
         * Registers an error message in the error registry and sends Elastic notification.
         *
         * @param alert Error message string.
         */
        public static void registerError(String alert) {
                // Makes sure there are no duplicates using the hashmap
                if (!errorMap.containsKey(alert)) {
                        errorMap.put(alert, 0);
                        error.add(alert + " 0");
                        notifyError(alert);
                        updateSmartDashboard();
                } else {
                        // Stores the value at the hashmap of the alert
                        Integer mapVal = errorMap.get(alert);
                        // Updates the hashmap value
                        errorMap.put(alert, mapVal + 1);
                        // Finds the alert in the vector and then updates it
                        error.set(error.findFirst(comparees
                                      -> comparees.get_0().equals(comparees.get_1()),
                                      alert + " " + mapVal),
                            alert + " " + (mapVal + 1));
                        updateSmartDashboard();
                }
        }

        /**
         * Registers a warning message in the warning registry and sends Elastic notification.
         *
         * @param alert Warning message string.
         */
        public static void registerWarning(String alert) {
                // Makes sure there are no duplicates using the hashmap
                if (!warningMap.containsKey(alert)) {
                        warningMap.put(alert, 0);
                        warning.add(alert + " 0");
                        notifyWarning(alert);
                        updateSmartDashboard();
                } else {
                        // Stores the value at the hashmap of the alert
                        Integer mapVal = warningMap.get(alert);
                        // Updates the hashmap value
                        warningMap.put(alert, mapVal + 1);
                        // Finds the alert in the vector and then updates it
                        warning.set(warning.findFirst(comparees
                                        -> comparees.get_0().equals(comparees.get_1()),
                                        alert + " " + mapVal),
                            alert + " " + (mapVal + 1));
                        updateSmartDashboard();
                }
        }

        /**
         * Registers an informational message in the info registry and sends Elastic notification.
         *
         * @param alert Informational message string.
         */
        public static void registerInfo(String alert) {
                // Makes sure there are no duplicates using the hashmap
                if (!infoMap.containsKey(alert)) {
                        infoMap.put(alert, 0);
                        info.add(alert + " 0");
                        notifyInfo(alert);
                        updateSmartDashboard();
                } else {
                        // Stores the value at the hashmap of the alert
                        Integer mapVal = infoMap.get(alert);
                        // Updates the hashmap value
                        infoMap.put(alert, mapVal + 1);
                        // Finds the alert in the vector and then updates it
                        info.set(info.findFirst(comparees
                                     -> comparees.get_0().equals(comparees.get_1()),
                                     alert + " " + mapVal),
                            alert + " " + (mapVal + 1));
                        updateSmartDashboard();
                }
        }

        /**
         * Sends an explicit error notification banner to Elastic.
         *
         * @param alert Error description string.
         */
        public static void notifyError(String alert) {
                Elastic.sendNotification(
                    notification.withLevel(Elastic.Notification.NotificationLevel.ERROR)
                        .withTitle("Error!")
                        .withDescription(alert));
        }

        /**
         * Sends an explicit warning notification banner to Elastic.
         *
         * @param alert Warning description string.
         */
        public static void notifyWarning(String alert) {
                Elastic.sendNotification(
                    notification.withLevel(Elastic.Notification.NotificationLevel.WARNING)
                        .withTitle("Warning:")
                        .withDescription(alert));
        }

        /**
         * Sends an explicit info notification banner to Elastic.
         *
         * @param alert Informational description string.
         */
        public static void notifyInfo(String alert) {
                Elastic.sendNotification(
                    notification.withLevel(Elastic.Notification.NotificationLevel.INFO)
                        .withTitle("Info")
                        .withDescription(alert));
        }

        /**
         * Calculates and updates the highest alert status color indicator on SmartDashboard.
         */
        private static void registerColor() {
                if (!error.isEmpty()) {
                        alertColor = new Color(255, 0, 0); // Red
                } else if (!warning.isEmpty()) {
                        alertColor = new Color(255, 255, 0); // Yellow
                } else {
                        alertColor = new Color(0, 255, 0); // Green
                }
                SmartDashboard.putString("Alert/Alerts", alertColor.toHexString());
        }

        /**
         * Pushes current error, warning, and info lists to NetworkTables string arrays.
         */
        private static void updateSmartDashboard() {
                SmartDashboard.putStringArray("Alert/errors", error.toArray());
                SmartDashboard.putStringArray("Alert/warnings", warning.toArray());
                SmartDashboard.putStringArray("Alert/info", info.toArray());
                registerColor();
        }

        /**
         * Inspects CTRE TalonFX motor status for active hardware faults and registers any errors found.
         *
         * @param kraken Target CTRE TalonFX motor controller.
         */
        public static void alertKraken(TalonFX kraken) {
                int krakenId = kraken.getDeviceID();
                if (!kraken.isAlive())
                        registerError("Motor " + krakenId + " is not alive");
                if (!kraken.isConnected(connectedTimeout))
                        registerError("Motor " + krakenId + " has been disconnected for "
                            + connectedTimeout + " sec");
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
                        registerError(
                            "Motor " + krakenId + " Fault_UsingFusedCANcoderWhileUnlicensed");
        }

        /**
         * Inspects REV SPARK Max motor controller for active hardware faults and registers errors found.
         *
         * @param neo Target REV SPARK Max motor controller.
         */
        public static void alertNeoFaults(SparkMax neo) {
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

        /**
         * Inspects REV SPARK Max motor controller for active hardware warnings and registers warnings found.
         *
         * @param neo Target REV SPARK Max motor controller.
         */
        public static void alertNeoWarnings(SparkMax neo) {
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
