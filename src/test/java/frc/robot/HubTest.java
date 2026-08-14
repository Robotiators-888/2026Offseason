package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Hub;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Covers the hub-targeting helpers that replaced five hand-copied versions of the same tag-lookup
 * block scattered across the commands.
 *
 * <p>A bare JVM test reports no alliance, so these resolve to the blue hub. That is enough to pin
 * the geometry: the offset direction, the derivation from the tag layout, and the distance math.
 */
class HubTest {

  private static final double kTagToHubCenterMeters = Units.inchesToMeters(23.5);

  /** With no DriverStation alliance, the code must fall back to blue rather than throwing. */
  @Test
  void defaultsToBlueHubWithoutAnAlliance() {
    assertEquals(26, Hub.getGoalTagId());
  }

  /** The goal must resolve, since the tag it is derived from is in the layout. */
  @Test
  void goalTranslationIsPresent() {
    assertTrue(Hub.getGoalTranslation().isPresent());
  }

  /**
   * The goal is the hub tag shifted along +X by the tag-to-center offset for blue. This is the
   * relationship the five duplicated copies each re-implemented by hand.
   */
  @Test
  void goalIsTheTagOffsetTowardFieldCenter() {
    Translation2d tag =
        Constants.Field.kTagLayout.getTagPose(26).orElseThrow().toPose2d().getTranslation();
    Translation2d goal = Hub.getGoalTranslation().orElseThrow();

    assertEquals(tag.getX() + kTagToHubCenterMeters, goal.getX(), 1e-9);
    assertEquals(tag.getY(), goal.getY(), 1e-9);
  }

  /** The goal must land inside the field, which a sign error on the offset would break. */
  @Test
  void goalLiesWithinTheField() {
    Translation2d goal = Hub.getGoalTranslation().orElseThrow();
    assertTrue(goal.getX() > 0 && goal.getX() < Constants.Field.fieldLength,
        "goal X off the field: " + goal.getX());
    assertTrue(goal.getY() > 0 && goal.getY() < Constants.Field.fieldWidth,
        "goal Y off the field: " + goal.getY());
  }

  /** Distance is measured from the supplied pose, and is zero when standing on the goal. */
  @Test
  void distanceIsMeasuredFromTheGivenPose() {
    Translation2d goal = Hub.getGoalTranslation().orElseThrow();

    Optional<Double> atGoal =
        Hub.getDistanceToGoal(new Pose2d(goal, Rotation2d.kZero));
    assertEquals(0.0, atGoal.orElseThrow(), 1e-9);

    Optional<Double> offset =
        Hub.getDistanceToGoal(new Pose2d(goal.plus(new Translation2d(3, 4)), Rotation2d.kZero));
    assertEquals(5.0, offset.orElseThrow(), 1e-9);
  }

  // ---------------------------------------------------------------------------------------------
  // Shift schedule. Per the 2026 game data spec, the game-specific message names the alliance
  // whose hub goes INACTIVE first — that alliance's hub is active in shifts 2 and 4, the other's
  // in shifts 1 and 3; both are active in the transition shift and endgame.
  // ---------------------------------------------------------------------------------------------

  private static final edu.wpi.first.wpilibj.DriverStation.Alliance kRed =
      edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
  private static final edu.wpi.first.wpilibj.DriverStation.Alliance kBlue =
      edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

  /** Red out-scored auto ("R"): Blue's hub is active in shifts 1 and 3, Red's in 2 and 4. */
  @Test
  void shiftAlternationFollowsTheGameDataSpec() {
    assertEquals(Optional.of(kBlue), Hub.activeAllianceAt(120, "R", kBlue), "shift 1");
    assertEquals(Optional.of(kRed), Hub.activeAllianceAt(95, "R", kBlue), "shift 2");
    assertEquals(Optional.of(kBlue), Hub.activeAllianceAt(70, "R", kBlue), "shift 3");
    assertEquals(Optional.of(kRed), Hub.activeAllianceAt(45, "R", kBlue), "shift 4");
    // And mirrored for "B".
    assertEquals(Optional.of(kRed), Hub.activeAllianceAt(120, "B", kBlue), "shift 1, B data");
    assertEquals(Optional.of(kBlue), Hub.activeAllianceAt(95, "B", kBlue), "shift 2, B data");
  }

  /** Transition shift and endgame report our own alliance: both hubs are active. */
  @Test
  void bothActivePhasesReportOurAlliance() {
    assertEquals(Optional.of(kBlue), Hub.activeAllianceAt(135, "R", kBlue), "transition");
    assertEquals(Optional.of(kBlue), Hub.activeAllianceAt(10, "R", kBlue), "endgame");
    assertEquals(Optional.of(kRed), Hub.activeAllianceAt(135, "R", kRed), "transition, red us");
  }

  /** No game data yet (FMS publishes it ~3 s into teleop) means no answer, not a guess. */
  @Test
  void missingGameDataYieldsEmpty() {
    assertEquals(Optional.empty(), Hub.activeAllianceAt(120, "", kBlue));
    assertEquals(Optional.empty(), Hub.activeAllianceAt(120, null, kBlue));
  }

  /**
   * The countdown to the next change must be continuous, non-negative, and reset exactly at each
   * boundary across the whole teleop period. The original arithmetic was correct but written so
   * opaquely it carried an "Idk if this is right" comment.
   */
  @Test
  void timeUntilChangeCountsDownToEveryBoundary() {
    final double[] boundaries = {130, 105, 80, 55, 30, 0};
    double previous = Double.NaN;
    for (double t = 140; t >= 0; t -= 0.5) {
      double remaining = Hub.timeUntilNextChangeAt(t);
      assertTrue(remaining >= 0, "negative countdown at t=" + t);
      assertTrue(remaining <= 25 || t > 130 || t < 30,
          "alliance-shift countdown exceeded the shift length at t=" + t);
      boolean atBoundary = false;
      for (double boundary : boundaries) {
        if (Math.abs(t - boundary) < 1e-9) {
          atBoundary = true;
        }
      }
      if (atBoundary) {
        assertEquals(0.0, remaining, 1e-9, "countdown must hit zero exactly at t=" + t);
      } else if (!Double.isNaN(previous) && previous > 0.4) {
        assertEquals(previous - 0.5, remaining, 1e-9, "countdown must fall smoothly at t=" + t);
      }
      previous = remaining;
    }
  }
}
