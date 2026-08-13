package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.AllianceFlipUtil.FieldFlipType;
import org.junit.jupiter.api.Test;

/**
 * Field-geometry checks that need no hardware.
 *
 * <p>Note that {@link AllianceFlipUtil#shouldFlip()} reads the DriverStation, which reports no
 * alliance in a bare JVM test, so {@code apply()} is a no-op here. These tests therefore pin down
 * the two things that do not depend on alliance: that the field dimensions are self-consistent with
 * the AprilTag layout, and that the flip is its own inverse.
 */
class AllianceFlipUtilTest {

  /**
   * The field dimensions must come from the loaded tag layout. They were previously hardcoded as
   * 16.532 x 8.001, which matched no real field and skewed every red-alliance mirror.
   */
  @Test
  void fieldDimensionsMatchTheLoadedTagLayout() {
    assertEquals(Constants.Field.kTagLayout.getFieldLength(), Constants.Field.fieldLength, 1e-9);
    assertEquals(Constants.Field.kTagLayout.getFieldWidth(), Constants.Field.fieldWidth, 1e-9);
  }

  /** Sanity-check the layout is a full-size field and not something degenerate. */
  @Test
  void fieldDimensionsArePlausible() {
    assertTrue(Constants.Field.fieldLength > 15 && Constants.Field.fieldLength < 18,
        "field length out of range: " + Constants.Field.fieldLength);
    assertTrue(Constants.Field.fieldWidth > 7 && Constants.Field.fieldWidth < 9,
        "field width out of range: " + Constants.Field.fieldWidth);
  }

  /** Both hub tags the aiming code targets must exist in the layout. */
  @Test
  void bothHubTagsExistInTheLayout() {
    assertTrue(Constants.Field.kTagLayout.getTagPose(10).isPresent(), "red hub tag 10 missing");
    assertTrue(Constants.Field.kTagLayout.getTagPose(26).isPresent(), "blue hub tag 26 missing");
  }

  /** Flipping twice must return the original point, whichever flip type is used. */
  @Test
  void flipIsItsOwnInverse() {
    Translation2d point = new Translation2d(3.25, 2.75);

    for (FieldFlipType type : FieldFlipType.values()) {
      Translation2d roundTripped = AllianceFlipUtil.apply(AllianceFlipUtil.apply(point, type), type);
      assertEquals(point.getX(), roundTripped.getX(), 1e-9, "X not restored for " + type);
      assertEquals(point.getY(), roundTripped.getY(), 1e-9, "Y not restored for " + type);
    }
  }
}
