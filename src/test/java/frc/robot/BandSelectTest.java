package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.SUB_Shooter;

/**
 * The flywheel band selector. The original implementation's hysteresis guard divided where it
 * should have multiplied, which made the early return provably identical to the fallthrough scan
 * — so the flywheel stepped bands on every odometry jitter at a boundary, exactly what
 * kBAND_HYSTERESIS existed to prevent. These tests pin the intended behaviour.
 */
public class BandSelectTest {
  private static final double[] kBands = {1200, 1500, 1800};
  private static final double kHysteresis = 0.90;

  private int select(double needed, int current) {
    return SUB_Shooter.selectBand(kBands, needed, current, kHysteresis);
  }

  /** Stepping up must be immediate — a band that cannot reach is useless slack. */
  @Test
  public void stepsUpImmediatelyWhenTheBandCannotReach() {
    assertEquals(2, select(1501, 1));
    assertEquals(1, select(1201, 0));
    assertEquals(2, select(1801, 2), "nothing reaches: hold the top band");
  }

  /**
   * Coming back down, the selector must hold the higher band until the requirement drops below
   * 90% of the lower band's speed — not the instant the lower band can technically reach.
   */
  @Test
  public void holdsTheHigherBandThroughTheBoundaryRegion() {
    // needed just below the lower band's speed: old code stepped down here instantly.
    assertEquals(2, select(1499, 2), "must hold 1800 while needed is above 0.9*1500");
    assertEquals(2, select(1360, 2), "1360 > 1350 still holds");
    assertEquals(1, select(1340, 2), "below 0.9*1500 finally steps down");
    assertEquals(1, select(1199, 1), "must hold 1500 while needed is above 0.9*1200");
    assertEquals(0, select(1070, 1), "below 0.9*1200 steps down to the bottom band");
  }

  /** The exact jitter case: needed oscillating about the 1500 boundary must not change bands. */
  @Test
  public void boundaryJitterCausesNoBandChanges() {
    int band = select(1495, 1);
    assertEquals(1, band, "1500 reaches 1495");
    band = select(1505, band);
    assertEquals(2, band, "first excursion above steps up, as it must");
    // From here, ±5 RPM of jitter about the boundary must never leave band 2.
    for (double needed : new double[] {1495, 1505, 1498, 1502, 1490, 1510}) {
      band = select(needed, band);
      assertEquals(2, band, "band changed on jitter at needed=" + needed);
    }
  }

  /** The bottom band has no lower neighbour; it must simply hold while it reaches. */
  @Test
  public void bottomBandHoldsWithoutALowerNeighbour() {
    assertEquals(0, select(1000, 0));
    assertEquals(0, select(1200, 0), "exactly at the band speed still reaches");
  }

  /** A large drop skips intermediate bands rather than walking down one per call. */
  @Test
  public void largeDropGoesStraightToTheLowestReachingBand() {
    assertEquals(0, select(1000, 2));
  }
}
