package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import frc.robot.utils.AimUtil;

/** The shared aiming math: stiction ramp, range-scaled tolerance, and motion compensation. */
public class AimUtilTest {
  private static final double kMaxRate = 4.7;
  private static final double kHeadingTol = Units.degreesToRadians(0.75);
  private static final double kStiction = Units.degreesToRadians(9);
  private static final double kRamp = Units.degreesToRadians(2);

  private double rate(double omega, double error) {
    return AimUtil.rotationalRate(omega, error, kMaxRate, kHeadingTol, kStiction, kRamp);
  }

  /** Inside the heading tolerance the output is exactly zero so the robot can settle. */
  @Test
  public void zeroInsideTheTolerance() {
    assertEquals(0.0, rate(0.3, kHeadingTol * 0.5), 1e-12);
    assertEquals(0.0, rate(-0.3, 0.0), 1e-12);
  }

  /**
   * The stiction term must fade in past the tolerance rather than switch on — the old step of
   * the full 9 deg/s at the boundary was a limit cycle: settle, drift out a hair, get kicked.
   */
  @Test
  public void stictionRampsInsteadOfStepping() {
    double omega = 0.05;
    double justOutside = rate(omega, kHeadingTol + 1e-6);
    assertTrue(justOutside - omega < kStiction * 0.01,
        "stiction term stepped in at the boundary instead of ramping: " + justOutside);

    // Halfway through the ramp, half the stiction term; past it, all of it.
    assertEquals(omega + kStiction * 0.5, rate(omega, kHeadingTol + kRamp / 2), 1e-9);
    assertEquals(omega + kStiction, rate(omega, kHeadingTol + kRamp * 3), 1e-9);
  }

  /** Output magnitude is clamped, and the clamp applies before the stiction term is added. */
  @Test
  public void outputIsClamped() {
    assertEquals(kMaxRate + kStiction, rate(100, Math.PI), 1e-9);
    assertEquals(-(kMaxRate + kStiction), rate(-100, Math.PI), 1e-9);
  }

  /** Tolerance shrinks with range, tracks the geometry, and respects its clamps. */
  @Test
  public void alignedToleranceScalesWithRange() {
    double halfWidth = Constants.Shooter.kAIM_HALF_WIDTH_METERS;
    double factor = Constants.Shooter.kAIM_SAFETY_FACTOR_TELEOP;

    double at3 = AimUtil.alignedToleranceRads(3.0, halfWidth, factor);
    assertEquals(Math.atan(halfWidth / 3.0) * factor, at3, 1e-9);

    double previous = Double.MAX_VALUE;
    for (double d = 1.0; d <= 12.0; d += 0.5) {
      double tol = AimUtil.alignedToleranceRads(d, halfWidth, factor);
      assertTrue(tol <= previous, "tolerance grew with range at d=" + d);
      assertTrue(tol >= Units.degreesToRadians(2) - 1e-12, "below the floor at d=" + d);
      assertTrue(tol <= Units.degreesToRadians(10) + 1e-12, "above the ceiling at d=" + d);
      previous = tol;
    }
  }

  /** With zero chassis velocity the virtual target is the goal — aiming is unchanged at rest. */
  @Test
  public void virtualTargetIsIdentityAtRest() {
    Translation2d goal = new Translation2d(4.6, 4.0);
    Translation2d virtual = AimUtil.virtualTarget(goal, new Translation2d(2.0, 2.0),
        0.0, 0.0, d -> d * 0.2153 + 0.7538);
    assertEquals(goal.getX(), virtual.getX(), 1e-12);
    assertEquals(goal.getY(), virtual.getY(), 1e-12);
  }

  /** A constant TOF makes the shift exactly -v*t, and it must oppose the velocity. */
  @Test
  public void virtualTargetLeadsOppositeTheVelocity() {
    Translation2d goal = new Translation2d(4.6, 4.0);
    Translation2d virtual = AimUtil.virtualTarget(goal, new Translation2d(2.0, 2.0),
        1.5, -0.5, d -> 1.2);
    assertEquals(goal.getX() - 1.5 * 1.2, virtual.getX(), 1e-9);
    assertEquals(goal.getY() + 0.5 * 1.2, virtual.getY(), 1e-9);
  }

  /**
   * With the real distance-dependent TOF model, the returned point must satisfy its own
   * fixed-point equation: re-deriving the TOF from the answer's distance and shifting the goal
   * again reproduces the answer. A fixed one- or two-iteration scheme (what the shuttle used to
   * hand-roll) leaves this equation violated by up to a metre at speed.
   */
  @Test
  public void virtualTargetSatisfiesItsFixedPointEquation() {
    Translation2d goal = new Translation2d(11.9, 4.0);
    Translation2d shooter = new Translation2d(7.0, 1.5);
    java.util.function.DoubleUnaryOperator tof = d -> d * 0.215298795 + 0.753755412;

    Translation2d virtual = AimUtil.virtualTarget(goal, shooter, 2.0, 1.0, tof);
    double t = tof.applyAsDouble(shooter.getDistance(virtual));
    Translation2d reproduced = new Translation2d(goal.getX() - 2.0 * t, goal.getY() - 1.0 * t);
    assertTrue(virtual.getDistance(reproduced) < 0.02,
        "fixed-point equation violated by " + virtual.getDistance(reproduced) + " m");
  }
}
