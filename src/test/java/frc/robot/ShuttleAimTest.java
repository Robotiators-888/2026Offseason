package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import frc.robot.utils.AimUtil;

/**
 * The shuttle aim point math in {@link AimUtil}. Each test documents the bug it guards against:
 * the original in-command implementation had both offset signs inverted relative to its own doc
 * comment, and flipped sidelines with no hysteresis so the target jumped two offsets across the
 * field whenever odometry jittered about the centerline.
 */
public class ShuttleAimTest {
  private static final double kOffset = Units.inchesToMeters(100);
  private static final double kFieldWidth = Constants.Field.fieldWidth;
  private static final double kHysteresis = 0.5;

  /**
   * Blue's wall is at X = 0, so "back toward our own end" must be a negative X offset; a robot
   * low in Y lobs toward the near (Y = 0) sideline. The original code produced +X (toward
   * midfield) and +Y (far sideline).
   */
  @Test
  public void blueLowRobotLobsTowardBlueWallAndNearSideline() {
    Translation2d offset = AimUtil.shuttleOffset(false, -1, kOffset);
    assertEquals(-kOffset, offset.getX(), 1e-9, "blue X offset must point at the blue wall");
    assertEquals(-kOffset, offset.getY(), 1e-9, "low robot must lob toward the Y=0 sideline");
  }

  /** Mirror image for red: own end is +X (the far wall), high robot lobs to the far sideline. */
  @Test
  public void redHighRobotLobsTowardRedWallAndFarSideline() {
    Translation2d offset = AimUtil.shuttleOffset(true, 1, kOffset);
    assertEquals(kOffset, offset.getX(), 1e-9, "red X offset must point at the red wall");
    assertEquals(kOffset, offset.getY(), 1e-9, "high robot must lob toward the far sideline");
  }

  /**
   * End-to-end direction check against the real tag layout: for each alliance, the shuttle
   * target must sit between that alliance's hub and its own wall, i.e. strictly on the wall side
   * of the hub, and inside the field.
   */
  @Test
  public void shuttleTargetSitsBetweenHubAndOwnWall() {
    Translation2d blueHub = Constants.Field.kTagLayout.getTagPose(26).orElseThrow()
        .toPose2d().getTranslation().plus(new Translation2d(Units.inchesToMeters(23.5), 0));
    Translation2d blueTarget = blueHub.plus(AimUtil.shuttleOffset(false, -1, kOffset));
    assertTrue(blueTarget.getX() < blueHub.getX(), "blue target must be on the blue-wall side of the hub");
    assertTrue(blueTarget.getX() > 0 && blueTarget.getY() > 0, "blue target must stay inside the field");

    Translation2d redHub = Constants.Field.kTagLayout.getTagPose(10).orElseThrow()
        .toPose2d().getTranslation().plus(new Translation2d(-Units.inchesToMeters(23.5), 0));
    Translation2d redTarget = redHub.plus(AimUtil.shuttleOffset(true, 1, kOffset));
    assertTrue(redTarget.getX() > redHub.getX(), "red target must be on the red-wall side of the hub");
    assertTrue(redTarget.getX() < Constants.Field.fieldLength && redTarget.getY() < kFieldWidth,
        "red target must stay inside the field");
  }

  /** First call (previousSign 0) picks the nearest sideline outright. */
  @Test
  public void firstCallPicksNearestSideline() {
    assertEquals(-1, AimUtil.sidelineSign(1.0, kFieldWidth, 0, kHysteresis));
    assertEquals(1, AimUtil.sidelineSign(kFieldWidth - 1.0, kFieldWidth, 0, kHysteresis));
    // Even inside the hysteresis band, an unset sign must resolve to something.
    assertEquals(-1, AimUtil.sidelineSign(kFieldWidth / 2.0 - 0.1, kFieldWidth, 0, kHysteresis));
  }

  /** Jitter of ±0.2 m about the centerline must never flip an established side. */
  @Test
  public void centerlineJitterDoesNotFlipSideline() {
    final double half = kFieldWidth / 2.0;
    int sign = AimUtil.sidelineSign(1.0, kFieldWidth, 0, kHysteresis);
    assertEquals(-1, sign);
    for (double jitter = -0.2; jitter <= 0.2; jitter += 0.05) {
      sign = AimUtil.sidelineSign(half + jitter, kFieldWidth, sign, kHysteresis);
      assertEquals(-1, sign, "sign flipped inside the hysteresis band at jitter " + jitter);
    }
    // A genuine crossing well past the band flips exactly once and stays flipped.
    sign = AimUtil.sidelineSign(half + 0.6, kFieldWidth, sign, kHysteresis);
    assertEquals(1, sign);
    sign = AimUtil.sidelineSign(half + 0.2, kFieldWidth, sign, kHysteresis);
    assertEquals(1, sign, "returning toward the centerline must not flip back inside the band");
  }

  /**
   * The latched setpoint must ignore jitter inside the deadband and only move on real drift —
   * the original code re-commanded the interpolated RPM every loop, so the at-speed check
   * flickered and the feed gate chattered.
   */
  @Test
  public void latchedSetpointIgnoresJitterAndTracksDrift() {
    final double deadband = Constants.Shooter.kSHUTTLE_RPM_LATCH_DEADBAND;
    double setpoint = AimUtil.latchedSetpoint(0, 2000, deadband);
    assertEquals(2000, setpoint, 1e-9, "an unset setpoint must jump straight to the requirement");

    int changes = 0;
    // ±10 cm of range jitter maps to well under the deadband in RPM; the setpoint must not move.
    for (double required : new double[] {2040, 1960, 2100, 1900, 2050}) {
      double next = AimUtil.latchedSetpoint(setpoint, required, deadband);
      if (next != setpoint) {
        changes++;
      }
      setpoint = next;
    }
    assertEquals(0, changes, "setpoint chased jitter inside the deadband");

    setpoint = AimUtil.latchedSetpoint(setpoint, 2200, deadband);
    assertEquals(2200, setpoint, 1e-9, "real drift beyond the deadband must move the setpoint");
  }
}
