package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SUB_Hood;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.TrajectorySolver;
import frc.robot.utils.TrajectorySolver.TrajectoryResult;

public class PhysicsAndMathTest {

    private static final double SCORE_HEIGHT_METERS = Units.inchesToMeters(Constants.Hood.ScoreHeight);
    private static final double GRAVITY = Constants.Shooter.kGRAVITATIONAL_CONSTANT;
    private static final double TOLERANCE_HEIGHT = 1e-4; // 1e-4 meters height tolerance
    private static final double TOLERANCE_MATH = 1e-4;

    @Nested
    @DisplayName("Tier 1: Trajectory Solving Accuracy & Kinematics")
    class Tier1_TrajectoryAccuracyTests {

        @ParameterizedTest(name = "Solve launch angle at distance {0}m")
        @ValueSource(doubles = { 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0 })
        void testLaunchAngleAccuracyAcrossOperatingRange(double distanceMeters) {
            // Pick an exit velocity well above v_min for each distance to ensure solvable arc
            double vMin = TrajectorySolver.calculateMinimumVelocity(distanceMeters, SCORE_HEIGHT_METERS);
            double exitVelocity = vMin + 2.0; // 2.0 m/s margin above minimum

            OptionalDouble angleOpt = TrajectorySolver.solveLaunchAngle(exitVelocity, distanceMeters, SCORE_HEIGHT_METERS);
            assertTrue(angleOpt.isPresent(), "Launch angle solution must exist for distance " + distanceMeters + "m at v0=" + exitVelocity);

            double theta = angleOpt.getAsDouble();
            assertTrue(theta > 0.0 && theta < Math.PI / 2.0, "Derived angle must be in first quadrant (0, pi/2)");

            // Ascending condition guard: d * tan(theta) > h
            double tanTheta = Math.tan(theta);
            assertTrue(distanceMeters * tanTheta > SCORE_HEIGHT_METERS,
                "Ascending condition d*tan(theta) > h must be satisfied for d=" + distanceMeters);

            // Kinematic forward trajectory validation: y(d) = d*tan(theta) - (g*d^2)/(2*v0^2*cos^2(theta))
            double cosTheta = Math.cos(theta);
            double calculatedHeight = distanceMeters * tanTheta
                - (GRAVITY * distanceMeters * distanceMeters) / (2.0 * exitVelocity * exitVelocity * cosTheta * cosTheta);

            assertEquals(SCORE_HEIGHT_METERS, calculatedHeight, TOLERANCE_HEIGHT,
                "Forward kinematics y(d) must equal target height |y(d)-h| < 10^-4 m for d=" + distanceMeters);
        }

        @Test
        @DisplayName("Velocity and RPM roundtrip conversion accuracy")
        void testVelocityRPMConversions() {
            double testRPM = 3000.0;
            double exitVelocity = TrajectorySolver.rpmToExitVelocity(testRPM);
            assertTrue(exitVelocity > 5.0 && exitVelocity < 15.0, "Exit velocity for 3000 RPM should be realistic, got " + exitVelocity);

            double roundtripRPM = TrajectorySolver.exitVelocityToRPM(exitVelocity);
            assertEquals(testRPM, roundtripRPM, TOLERANCE_MATH, "RPM -> v0 -> RPM roundtrip conversion must match within tolerance");
        }
    }

    @Nested
    @DisplayName("Tier 2: Domain Validation & Edge Case Fallbacks")
    class Tier2_DomainValidationAndFallbackTests {

        @Test
        @DisplayName("Complex roots detection when velocity is below minimum")
        void testComplexRootsInsufficientVelocity() {
            double distance = 5.0;
            double vMin = TrajectorySolver.calculateMinimumVelocity(distance, SCORE_HEIGHT_METERS);
            double lowVelocity = vMin - 0.5; // Strictly below minimum velocity

            OptionalDouble angleOpt = TrajectorySolver.solveLaunchAngle(lowVelocity, distance, SCORE_HEIGHT_METERS);
            assertFalse(angleOpt.isPresent(), "solveLaunchAngle must return OptionalDouble.empty() when v0 < vMin (Delta < 0)");
        }

        @Test
        @DisplayName("Minimum velocity calculation correctness and threshold behavior")
        void testMinimumVelocityThreshold() {
            double d = 4.0;
            double vMin = TrajectorySolver.calculateMinimumVelocity(d, SCORE_HEIGHT_METERS);

            // Expected theoretical vMin = sqrt(g * (h + sqrt(d^2 + h^2)))
            double expectedVMin = Math.sqrt(GRAVITY * (SCORE_HEIGHT_METERS + Math.sqrt(d * d + SCORE_HEIGHT_METERS * SCORE_HEIGHT_METERS)));
            assertEquals(expectedVMin, vMin, TOLERANCE_MATH, "vMin must strictly match theoretical formula");

            // Solve just above vMin
            OptionalDouble angleAbove = TrajectorySolver.solveLaunchAngle(vMin + 0.05, d, SCORE_HEIGHT_METERS);
            assertTrue(angleAbove.isPresent(), "Solving angle just above vMin must succeed");

            // Solve just below vMin
            OptionalDouble angleBelow = TrajectorySolver.solveLaunchAngle(vMin - 0.05, d, SCORE_HEIGHT_METERS);
            assertFalse(angleBelow.isPresent(), "Solving angle just below vMin must return empty");
        }

        @Test
        @DisplayName("Zero and negative input guards")
        void testZeroAndNegativeInputs() {
            assertFalse(TrajectorySolver.solveLaunchAngle(0.0, 3.0, SCORE_HEIGHT_METERS).isPresent());
            assertFalse(TrajectorySolver.solveLaunchAngle(-10.0, 3.0, SCORE_HEIGHT_METERS).isPresent());
            assertFalse(TrajectorySolver.solveLaunchAngle(10.0, 0.0, SCORE_HEIGHT_METERS).isPresent());
            assertFalse(TrajectorySolver.solveLaunchAngle(10.0, -2.0, SCORE_HEIGHT_METERS).isPresent());

            assertEquals(0.0, TrajectorySolver.exitVelocityToRPM(0.0), TOLERANCE_MATH);
            assertEquals(0.0, TrajectorySolver.exitVelocityToRPM(-100.0), TOLERANCE_MATH);
            assertEquals(0.0, TrajectorySolver.rpmToExitVelocity(0.0), TOLERANCE_MATH);
            assertEquals(0.0, TrajectorySolver.rpmToExitVelocity(-500.0), TOLERANCE_MATH);
        }

        @Test
        @DisplayName("NaN and Infinity safe fallbacks")
        void testNaNAndInfinityHandling() {
            assertFalse(TrajectorySolver.solveLaunchAngle(Double.NaN, 3.0, SCORE_HEIGHT_METERS).isPresent());
            assertFalse(TrajectorySolver.solveLaunchAngle(10.0, Double.NaN, SCORE_HEIGHT_METERS).isPresent());
            assertFalse(TrajectorySolver.solveLaunchAngle(Double.POSITIVE_INFINITY, 3.0, SCORE_HEIGHT_METERS).isPresent());

            TrajectoryResult nanResult = TrajectorySolver.calculateTrajectory(3000, Double.NaN);
            assertFalse(nanResult.isValid(), "NaN distance must yield isValid = false");
            assertTrue(nanResult.isUnreachable(), "NaN distance must yield isUnreachable = true");

            TrajectoryResult infResult = TrajectorySolver.calculateTrajectory(Double.POSITIVE_INFINITY, 3.0);
            assertFalse(infResult.isValid(), "Infinity RPM must yield isValid = false");
            assertTrue(infResult.isUnreachable(), "Infinity RPM must yield isUnreachable = true");
        }

        @Test
        @DisplayName("Distance outer bounds validation (<1.5m or >7.0m)")
        void testDistanceOuterBounds() {
            TrajectoryResult resultTooClose = TrajectorySolver.calculateTrajectory(3000, 1.4);
            assertFalse(resultTooClose.isValid(), "Distance < 1.5m must be invalid");
            assertTrue(resultTooClose.isUnreachable(), "Distance < 1.5m must be unreachable");

            TrajectoryResult resultTooFar = TrajectorySolver.calculateTrajectory(3000, 7.1);
            assertFalse(resultTooFar.isValid(), "Distance > 7.0m must be invalid");
            assertTrue(resultTooFar.isUnreachable(), "Distance > 7.0m must be unreachable");

            TrajectoryResult resultMinBound = TrajectorySolver.calculateTrajectory(3000, 1.5);
            assertTrue(resultMinBound.isValid(), "Exact distance 1.5m must be valid");

            TrajectoryResult resultMaxBound = TrajectorySolver.calculateTrajectory(4000, 6.5);
            assertTrue(resultMaxBound.isValid(), "Distance 6.5m must be valid");
        }
    }

    @Nested
    @DisplayName("Tier 3: Flywheel Stability & Hysteresis Envelope")
    class Tier3_FlywheelStabilityTests {

        @Test
        @DisplayName("Distance perturbations within envelope maintain constant RPM setpoint")
        void testFlywheelRPMStabilityWithinAdjustmentEnvelope() {
            double baseRPM = 3000.0;
            double baseDistance = 3.0; // Nominal distance

            TrajectoryResult baseResult = TrajectorySolver.calculateTrajectory(baseRPM, baseDistance);
            assertTrue(baseResult.isValid(), "Base trajectory must be valid");
            assertFalse(baseResult.flywheelAdjusted(), "Base flywheel should not adjust if initial speed works");
            assertEquals(baseRPM, baseResult.targetFlywheelRPM(), TOLERANCE_MATH);

            // Small distance perturbations (+/- 0.1m, +/- 0.2m) around 3.0m
            double[] perturbations = { 2.8, 2.9, 3.0, 3.1, 3.2 };
            double lastHoodAngle = -1.0;

            for (double d : perturbations) {
                TrajectoryResult res = TrajectorySolver.calculateTrajectory(baseRPM, d);
                assertTrue(res.isValid(), "Perturbed trajectory at d=" + d + "m must be valid");
                assertFalse(res.flywheelAdjusted(), "Flywheel RPM MUST remain fixed (flywheelAdjusted=false) for distance " + d + "m");
                assertEquals(baseRPM, res.targetFlywheelRPM(), TOLERANCE_MATH, "Target RPM must hold at " + baseRPM);
                assertTrue(res.desiredHoodAngleRad() != lastHoodAngle, "Hood angle must adjust to compensate for distance perturbation");
                lastHoodAngle = res.desiredHoodAngleRad();
            }
        }
    }

    @Nested
    @DisplayName("Tier 4: Flywheel Adjustment & Limits Verification")
    class Tier4_FlywheelAdjustmentAndLimitsTests {

        @Test
        @DisplayName("Flywheel setpoint adjusts when initial RPM cannot reach target within hood limits")
        void testFlywheelAdjustmentWhenOutsideEnvelope() {
            double lowRPM = 1000.0; // Too low for 5.0m
            double distance = 5.0;

            TrajectoryResult result = TrajectorySolver.calculateTrajectory(lowRPM, distance);
            assertTrue(result.isValid(), "Trajectory should be valid after flywheel RPM adjustment");
            assertTrue(result.flywheelAdjusted(), "Flywheel RPM MUST be adjusted when lowRPM is insufficient for distance");
            assertTrue(result.targetFlywheelRPM() > lowRPM, "Target RPM must increase above initial low RPM");
            assertFalse(result.isUnreachable(), "Target must remain reachable after RPM adjustment");
        }

        @Test
        @DisplayName("Unreachable trajectory when distance requires speed beyond maximum RPM limit")
        void testUnreachableTrajectoryExtremeDistance() {
            // Distance 7.1m is beyond MAX_DISTANCE_METERS (7.0m)
            double distance = 7.1;
            TrajectoryResult result = TrajectorySolver.calculateTrajectory(1000, distance);
            assertFalse(result.isValid(), "Out of bounds distance 7.1m must be invalid");
            assertTrue(result.isUnreachable(), "Out of bounds distance 7.1m must be marked unreachable");
        }
    }

    @Nested
    @DisplayName("Subsystem Heuristics & Utilities Tests")
    class LegacyAndUtilityTests {

        @Test
        @DisplayName("SUB_Hood findoptimalangle basic checks")
        void testOptimalAngleAndRPMCalculations() {
            for (double distance = 1.0; distance <= 8.0; distance += 0.5) {
                double angleRad = SUB_Hood.findoptimalangle(distance);
                assertTrue(angleRad > 0 && angleRad < Math.PI / 2.0, "Angle should be physically valid in radians");

                double rpm = SUB_Shooter.findoptimalRPM(distance, angleRad);
                assertFalse(Double.isNaN(rpm), "RPM should not be NaN for distance " + distance);
                assertFalse(Double.isInfinite(rpm), "RPM should not be Infinite for distance " + distance);
                assertTrue(rpm > 0 && rpm < 7000, "RPM should be positive and realistic for distance " + distance + ", got " + rpm);
            }
        }

        @Test
        @DisplayName("AllianceFlipUtil basic check")
        void testAllianceFlipUtil() {
            Translation2d blueTranslation = new Translation2d(2.0, 3.0);
            Translation2d flipped = AllianceFlipUtil.apply(blueTranslation);
            assertTrue(flipped.getX() >= 0);
            assertTrue(flipped.getY() >= 0);
        }

        @Test
        @DisplayName("Alert system registration check")
        void testAlertSystem() {
            Alert.registerInfo("Test Alert Info");
            Alert.registerWarning("Test Alert Warning");
            Alert.registerError("Test Alert Error");
            Alert.updateSmartDashboard();
        }
    }
}
