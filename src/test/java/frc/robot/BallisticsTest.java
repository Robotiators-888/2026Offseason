package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Shooter;
import org.junit.jupiter.api.Test;

/**
 * Covers the two pure static solvers behind every shot. Both are plain math with no HAL or CAN
 * dependency, so they run on the JVM without a roboRIO.
 *
 * <p>These exist because the pair previously disagreed about units: {@code findoptimalangle}
 * returned radians while {@code findoptimalRPM} fed the same value through
 * {@code Units.degreesToRadians()} for one term and used it raw in another.
 */
class BallisticsTest {

  private static final double kTargetHeightMeters =
      Units.inchesToMeters(Constants.Hood.ScoreHeight);

  /**
   * The minimum-launch-speed angle is {@code pi/4 + atan(h/d)/2}, which is always strictly between
   * 45 and 90 degrees for a target above the shooter.
   */
  @Test
  void optimalAngleStaysInPhysicalRange() {
    for (double distance = 1.0; distance <= 12.0; distance += 0.5) {
      double angle = SUB_Hood.findoptimalangle(distance);
      assertTrue(angle > Math.PI / 4.0, "angle must exceed 45 deg at d=" + distance);
      assertTrue(angle < Math.PI / 2.0, "angle must stay under 90 deg at d=" + distance);
    }
  }

  /** Further away means a flatter shot, asymptotically approaching 45 degrees. */
  @Test
  void optimalAngleDecreasesWithDistance() {
    double previous = SUB_Hood.findoptimalangle(1.0);
    for (double distance = 1.5; distance <= 12.0; distance += 0.5) {
      double angle = SUB_Hood.findoptimalangle(distance);
      assertTrue(angle < previous, "angle should flatten as range grows, broke at d=" + distance);
      previous = angle;
    }
  }

  /**
   * The angle solver guarantees {@code d*tan(theta) > h}, which is exactly the condition that keeps
   * the velocity solve's square root real. Pairing the two must therefore never produce NaN.
   */
  @Test
  void optimalAnglePairedWithRpmNeverProducesNaN() {
    for (double distance = 0.5; distance <= 15.0; distance += 0.25) {
      double angle = SUB_Hood.findoptimalangle(distance);
      double rpm = SUB_Shooter.findoptimalRPM(distance, angle);
      assertTrue(Double.isFinite(rpm), "RPM must be finite at d=" + distance + ", got " + rpm);
      assertTrue(rpm > 0, "RPM must be positive at d=" + distance + ", got " + rpm);
    }
  }

  /** A shot that cannot reach the target returns 0 rather than NaN from a negative square root. */
  @Test
  void unreachableShotReturnsZeroRatherThanNaN() {
    // Nearly flat launch at close range: the projectile can never climb to the target.
    double rpm = SUB_Shooter.findoptimalRPM(0.5, Units.degreesToRadians(1));
    assertEquals(0.0, rpm, "an unreachable shot should return 0");
  }

  /** Longer range needs a faster flywheel. */
  @Test
  void requiredRpmIncreasesWithDistance() {
    double previous = 0;
    for (double distance = 2.0; distance <= 12.0; distance += 0.5) {
      double rpm = SUB_Shooter.findoptimalRPM(distance, SUB_Hood.findoptimalangle(distance));
      assertTrue(rpm > previous, "RPM should grow with range, broke at d=" + distance);
      previous = rpm;
    }
  }

  /**
   * Independent check of the whole chain against hand-computed physics, which is what catches a
   * wrong unit-conversion constant. The old code used a literal 720 where the inches-per-meter
   * conversion needs 60 * 39.37, leaving the answer low by a factor of ~3.3.
   */
  @Test
  void rpmMatchesHandComputedPhysics() {
    final double distance = 4.0;
    final double angle = SUB_Hood.findoptimalangle(distance);

    // v = sec(theta) * sqrt( g*d^2 / (2*(d*tan(theta) - h)) )
    final double expectedVelocity =
        (1 / Math.cos(angle))
            * Math.sqrt(
                (Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance)
                    / (2 * (distance * Math.tan(angle) - kTargetHeightMeters)));

    // Surface speed -> RPM: v * 60 / (pi * D), with D converted from inches to meters.
    final double diameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
    final double expectedRpm =
        (expectedVelocity * 60.0)
            / (Math.PI * diameterMeters * Constants.Shooter.kSHOOTER_COMPRESSION_RATIO);

    assertEquals(expectedRpm, SUB_Shooter.findoptimalRPM(distance, angle), 1e-6);
  }
}
