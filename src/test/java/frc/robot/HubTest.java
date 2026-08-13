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
}
