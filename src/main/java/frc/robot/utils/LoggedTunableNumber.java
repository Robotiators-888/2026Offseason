package frc.robot.utils;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a Tunable Number. Gets value from dashboard in tuning mode, returns default if not in
 * tuning mode or value not present on dashboard.
 */
public class LoggedTunableNumber implements DoubleSupplier {
    private static final String TABLE_KEY = "TunableNumbers";

    private final String key;
    private boolean hasDefault = false;
    private double defaultValue = 0.0;
    private LoggedNetworkNumber dashboardNumber;
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableNumber.
     *
     * @param dashboardKey Key on NetworkTables dashboard.
     */
    public LoggedTunableNumber(String dashboardKey) {
        this.key = TABLE_KEY + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with default value.
     *
     * @param dashboardKey Key on NetworkTables dashboard.
     * @param defaultValue Default value.
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. Can only be set once.
     *
     * @param defaultValue The default value.
     */
    public void initDefault(double defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.TUNING_MODE) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value (from dashboard if tuning mode is enabled, otherwise default).
     *
     * @return The current value.
     */
    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
        }
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    /**
     * Checks whether the number has changed since the last check for the given ID.
     *
     * @param id Unique identifier for the caller (e.g. hashCode()).
     * @return True if value has changed, false otherwise.
     */
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }
        return false;
    }

    /**
     * Runs an action if the value has changed.
     *
     * @param id Unique identifier for caller.
     * @param action Consumer called with the new value.
     */
    public void ifChanged(int id, Consumer<Double> action) {
        if (hasChanged(id)) {
            action.accept(get());
        }
    }
}
