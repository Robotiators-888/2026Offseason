package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Shooter;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

/**
 * Covers the inverse ballistics solve behind the banded-RPM scheme: with the flywheel held at a
 * standing speed, the hood is what aims, so {@code angleForLaunchSpeed} has to be right.
 */
class HoodAngleSolveTest {

  private static final double kHeight = Units.inchesToMeters(Constants.Hood.ScoreHeight);
  private static final double kG = Constants.Shooter.kGRAVITATIONAL_CONSTANT;

  /** Where the ball actually lands, forward-simulated from a launch speed and angle. */
  private static double landingRange(double speed, double angle) {
    // h = d*tan(t) - g*d^2 / (2*v^2*cos^2(t))  ->  solve the quadratic for d.
    double a = kG / (2 * speed * speed * Math.cos(angle) * Math.cos(angle));
    double b = -Math.tan(angle);
    double c = kHeight;
    double disc = b * b - 4 * a * c;
    if (disc < 0) {
      return Double.NaN;
    }
    // A lobbed ball crosses the target height twice. Take the far root — the descending branch —
    // since that is the one the goal sits on.
    return (-b + Math.sqrt(disc)) / (2 * a);
  }

  /**
   * The whole point of the solver: shooting at the angle it returns must actually land the ball at
   * the requested range.
   */
  @Test
  void solvedAngleLandsAtTheRequestedRange() {
    for (double distance = 1.5; distance <= 5.0; distance += 0.25) {
      // Comfortably above the minimum so both arcs exist.
      double speed = SUB_Shooter.minLaunchSpeed(distance) * 1.15;
      OptionalDouble angle = SUB_Hood.angleForLaunchSpeed(distance, speed);
      assertTrue(angle.isPresent(), "no solution at d=" + distance);

      assertEquals(distance, landingRange(speed, angle.getAsDouble()), 1e-6,
          "ball misses at d=" + distance);
    }
  }

  /** The returned root must be the lobbed one, i.e. steeper than the minimum-energy angle. */
  @Test
  void returnsTheSteeperOfTheTwoArcs() {
    for (double distance = 1.5; distance <= 5.0; distance += 0.5) {
      double speed = SUB_Shooter.minLaunchSpeed(distance) * 1.15;
      double angle = SUB_Hood.angleForLaunchSpeed(distance, speed).orElseThrow();
      assertTrue(angle > SUB_Hood.findoptimalangle(distance),
          "expected the lobbed arc at d=" + distance);
      assertTrue(angle < Math.PI / 2.0, "angle must stay under vertical at d=" + distance);
    }
  }

  /** Exactly at the minimum speed the two arcs merge onto the minimum-energy angle. */
  @Test
  void atMinimumSpeedTheSolutionIsTheMinimumEnergyAngle() {
    double distance = 3.0;
    double angle = SUB_Hood.angleForLaunchSpeed(distance, SUB_Shooter.minLaunchSpeed(distance))
        .orElseThrow();
    assertEquals(SUB_Hood.findoptimalangle(distance), angle, 1e-6);
  }

  /** Below the minimum speed no angle exists, and the solver must say so rather than guess. */
  @Test
  void belowMinimumSpeedThereIsNoSolution() {
    double distance = 4.0;
    double tooSlow = SUB_Shooter.minLaunchSpeed(distance) * 0.95;
    assertTrue(SUB_Hood.angleForLaunchSpeed(distance, tooSlow).isEmpty(),
        "an unreachable range must return empty, not an angle");
  }

  /** More speed at a fixed range means a higher lob, which is what makes the hood the aiming axis. */
  @Test
  void moreSpeedMeansASteeperLob() {
    double distance = 3.0;
    double base = SUB_Shooter.minLaunchSpeed(distance);
    double previous = 0;
    for (double factor = 1.02; factor <= 1.40; factor += 0.02) {
      double angle = SUB_Hood.angleForLaunchSpeed(distance, base * factor).orElseThrow();
      assertTrue(angle > previous, "lob should steepen with speed, broke at factor " + factor);
      previous = angle;
    }
  }

  /** An RPM below the table's tuned value for that range cannot reach it at any angle. */
  @Test
  void rpmBelowTheTunedValueHasNoSolution() {
    double distance = 4.0;
    double tuned = 1600;
    assertTrue(SUB_Hood.angleForRPM(distance, tuned * 0.9, tuned).isEmpty());
  }

  /** At exactly the tuned RPM the hood sits at the minimum-energy angle, matching the LUT's anchor. */
  @Test
  void atTunedRpmTheHoodSitsAtTheMinimumEnergyAngle() {
    double distance = 4.0;
    double tuned = 1600;
    assertEquals(SUB_Hood.findoptimalangle(distance),
        SUB_Hood.angleForRPM(distance, tuned, tuned).orElseThrow(), 1e-6);
  }

  /** Guard against a zero or negative tuned value producing a bogus angle. */
  @Test
  void rejectsNonPositiveTunedRpm() {
    assertFalse(SUB_Hood.angleForRPM(4.0, 1800, 0).isPresent());
  }
}
