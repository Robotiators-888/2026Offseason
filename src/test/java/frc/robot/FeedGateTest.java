package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.FeedGate;

/**
 * The debounced, latching fire gate. Before it existed the aim commands re-evaluated the raw
 * ready condition every 20 ms, so a ball entering the wheels (which drops flywheel RPM far past
 * the arming tolerance) cut the indexer mid-ball, and a single-loop blip of "ready" opened the
 * feed on jitter.
 */
public class FeedGateTest {
  /** A hand-cranked clock so the debounce window is driven deterministically. */
  private double now = 0;

  private FeedGate newGate() {
    now = 0;
    return new FeedGate(0.10, () -> now);
  }

  private boolean tick(FeedGate gate, boolean arm, boolean sustain) {
    now += 0.020;
    return gate.update(arm, sustain);
  }

  @Test
  public void flickeringArmNeverOpensTheGate() {
    FeedGate gate = newGate();
    for (int i = 0; i < 50; i++) {
      assertFalse(tick(gate, i % 2 == 0, true), "gate opened on a flickering arm signal");
    }
  }

  @Test
  public void solidArmOpensAfterTheDebounceWindow() {
    FeedGate gate = newGate();
    boolean opened = false;
    double armedAt = Double.NaN;
    for (int i = 0; i < 20 && !opened; i++) {
      if (Double.isNaN(armedAt)) {
        armedAt = now;
      }
      opened = tick(gate, true, true);
    }
    assertTrue(opened, "gate never opened on a solid arm signal");
    assertTrue(now - armedAt >= 0.10, "gate opened before the debounce window elapsed");
  }

  @Test
  public void staysOpenThroughBallDroopWhileSustainHolds() {
    FeedGate gate = newGate();
    while (!tick(gate, true, true)) {
      // arm
    }
    // Ball contact: arm condition (tight RPM tolerance) fails, sustain (loose) holds.
    for (int i = 0; i < 10; i++) {
      assertTrue(tick(gate, false, true), "gate cut out mid-ball while sustain held");
    }
  }

  @Test
  public void sustainFailureClosesAndRequiresAFullReArm() {
    FeedGate gate = newGate();
    while (!tick(gate, true, true)) {
      // arm
    }
    assertFalse(tick(gate, true, false), "gate stayed open after sustain failed");
    // Even with arm immediately true again, the debounce must run from scratch.
    assertFalse(tick(gate, true, true), "gate re-opened without re-running the debounce");
    boolean reopened = false;
    double rearmedAt = now;
    for (int i = 0; i < 20 && !reopened; i++) {
      reopened = tick(gate, true, true);
    }
    assertTrue(reopened, "gate never re-armed");
    assertTrue(now - rearmedAt >= 0.10 - 0.020 - 1e-9, "re-arm skipped the debounce window");
  }

  @Test
  public void resetForgetsAnOpenGate() {
    FeedGate gate = newGate();
    while (!tick(gate, true, true)) {
      // arm
    }
    gate.reset();
    assertFalse(tick(gate, false, true), "reset gate stayed open on sustain alone");
    assertFalse(tick(gate, true, true), "reset gate skipped the debounce");
  }
}
