package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/**
 * Utility for robust communication with CTRE Phoenix 6 devices.
 */
public final class PhoenixUtil {
    private PhoenixUtil() {}

    /**
     * Attempts to run a Phoenix 6 command repeatedly until it succeeds or max attempts are reached.
     *
     * @param maxAttempts Maximum number of retry attempts.
     * @param command Supplier returning a {@link StatusCode}.
     * @return true if command succeeded with {@link StatusCode#isOK()}, false otherwise.
     */
    public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            StatusCode status = command.get();
            if (status.isOK()) {
                return true;
            }
        }
        return false;
    }
}
