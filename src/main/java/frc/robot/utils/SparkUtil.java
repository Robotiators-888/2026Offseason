package frc.robot.utils;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Utility for robust communication with REV Spark motor controllers.
 */
public final class SparkUtil {
    private SparkUtil() {}

    /** Stores whether any error has been detected by other utility methods. */
    public static boolean sparkStickyFault = false;

    /**
     * Consumes a value from a Spark sensor only if the reading had no bus error.
     *
     * @param spark Spark motor controller.
     * @param supplier Supplier of the double value.
     * @param consumer Consumer of the double value if valid.
     */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /**
     * Returns a sensor value from a Spark or a fallback default if the reading was invalid.
     *
     * @param spark Spark motor controller.
     * @param supplier Supplier of the double value.
     * @param defaultValue Default value returned upon error.
     * @return Value from supplier if successful, or defaultValue upon error.
     */
    public static double ifOkOrDefault(SparkBase spark, DoubleSupplier supplier, double defaultValue) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            return value;
        } else {
            sparkStickyFault = true;
            return defaultValue;
        }
    }

    /**
     * Attempts to run a configuration command repeatedly until no error is produced.
     *
     * @param spark Spark motor controller.
     * @param maxAttempts Maximum retry count.
     * @param command Command returning a {@link REVLibError}.
     * @return true if succeeded with kOk, false otherwise.
     */
    public static boolean tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i = 0; i < maxAttempts; i++) {
            REVLibError error = command.get();
            if (error == REVLibError.kOk) {
                return true;
            } else {
                sparkStickyFault = true;
            }
        }
        return false;
    }
}
