package frc.robot.utils;

import java.util.function.DoubleSupplier;

/**
 * Latching fire gate for the feed path.
 *
 * <p>To <b>start</b> feeding, the arm condition must hold continuously for a debounce window — a
 * single 20 ms blip of "aligned and at speed" is jitter, not readiness. Once feeding, the gate
 * stays open on a looser sustain condition: ball contact drops the flywheel far outside the
 * arming tolerance, and cutting the indexer mid-ball is what jams the feed. When the sustain
 * condition fails the gate closes and must re-arm from scratch.
 *
 * <p>Time comes from an injected clock so tests can drive it deterministically.
 */
public class FeedGate {
    private final double armDebounceSeconds;
    private final DoubleSupplier clockSeconds;
    private double armStart = Double.NaN;
    private boolean feeding = false;

    /**
     * @param armDebounceSeconds how long the arm condition must hold before feeding starts
     * @param clockSeconds monotonic clock, e.g. {@code Timer::getFPGATimestamp}
     */
    public FeedGate (final double armDebounceSeconds, final DoubleSupplier clockSeconds) {
        this.armDebounceSeconds = armDebounceSeconds;
        this.clockSeconds = clockSeconds;
    }

    /** Forgets all state. Call from the owning command's initialize(). */
    public void reset () {
        armStart = Double.NaN;
        feeding = false;
    }

    /**
     * @param armReady the strict condition required to start feeding
     * @param sustainReady the looser condition required to keep feeding
     * @return true while the feed should run
     */
    public boolean update (final boolean armReady, final boolean sustainReady) {
        if (feeding) {
            feeding = sustainReady;
            if (!feeding) {
                armStart = Double.NaN;
            }
            return feeding;
        }
        if (!armReady) {
            armStart = Double.NaN;
            return false;
        }
        final double now = clockSeconds.getAsDouble();
        if (Double.isNaN(armStart)) {
            armStart = now;
        }
        feeding = now - armStart >= armDebounceSeconds;
        return feeding;
    }
}
