# Remediation Strategy Handoff Report — Explorer 4 (Milestone 1)

**Target File**: `src/main/java/frc/robot/utils/TrajectorySolver.java`  
**Test File**: `src/test/java/frc/robot/PhysicsAndMathTest.java`  
**Working Directory**: `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_m1_4`  

---

## 1. Observation

### Forensic Auditor Finding & Root Cause Analysis
From the Forensic Auditor's handoff report (`.agents/teamwork_preview_auditor_m1_1/handoff.md`), the test `testNaNAndInfinityHandling` in `Tier2_DomainValidationAndFallbackTests` failed in Iteration 1 with:
`org.opentest4j.AssertionFailedError: Infinity RPM must yield isValid = false ==> expected: <false> but was: <true>`

### Source Code Inspection: `src/main/java/frc/robot/utils/TrajectorySolver.java`

1. **`calculateTrajectory(double currentFlywheelRPM, double distanceMeters)` (Lines 142–189)**:
   ```java
   142: public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters) {
   143:     double targetHeightMeters = Units.inchesToMeters(Constants.Hood.ScoreHeight);
   144: 
   145:     // 1. Distance Bounds Guard
   146:     if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
   147:         distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS) {
   148:         return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true);
   149:     }
   150: 
   151:     // 2. Attempt Hood-First Solve at current flywheel RPM
   152:     if (currentFlywheelRPM > 0.0 && !Double.isNaN(currentFlywheelRPM) && !Double.isInfinite(currentFlywheelRPM)) {
   153:         ...
   154:     }
   155: 
   156:     // 3. Flywheel Speed Adjustment Needed
   157:     ...
   158:     double targetRPM = Math.max(MIN_FLYWHEEL_RPM, Math.min(MAX_FLYWHEEL_RPM, exitVelocityToRPM(v0Required)));
   ...
   183:     return new TrajectoryResult(thetaAdj, targetRPM, true, true, false);
   ```
   - **Defect Location**: Lines 146–147 only check `distanceMeters` for `Double.isNaN` and `Double.isInfinite`. `currentFlywheelRPM` is **not** checked in Step 1.
   - **Failure Flow**:
     1. When `Double.POSITIVE_INFINITY` (or `Double.NaN`) is passed as `currentFlywheelRPM` with a valid distance (e.g., 3.0m), Step 1 guard evaluates to `false` and passes through.
     2. In Step 2, `!Double.isInfinite(currentFlywheelRPM)` evaluates to `false`, skipping Step 2.
     3. Execution falls through to Step 3 (Flywheel Speed Adjustment Needed), which ignores `currentFlywheelRPM` entirely, calculates a valid RPM (~3500 RPM for 3.0m), and returns `new TrajectoryResult(thetaAdj, targetRPM, true, true, false)` with `isValid = true`!

2. **Audit of All Other Methods in `TrajectorySolver.java`**:
   - `exitVelocityToRPM(double v0)` (Lines 44–51):
     `if (v0 <= 0.0 || Double.isNaN(v0) || Double.isInfinite(v0)) { return 0.0; }`
     **Status**: Fully guarded.
   - `rpmToExitVelocity(double rpm)` (Lines 57–64):
     `if (rpm <= 0.0 || Double.isNaN(rpm) || Double.isInfinite(rpm)) { return 0.0; }`
     **Status**: Fully guarded.
   - `calculateMinimumVelocity(double distanceMeters, double targetHeightMeters)` (Lines 70–85):
     `if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) || Double.isNaN(targetHeightMeters) || Double.isInfinite(targetHeightMeters)) { return 0.0; }`
     Plus returns `0.0` if `vMin` is `NaN` or `Infinite`.
     **Status**: Fully guarded.
   - `solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters)` (Lines 93–133):
     `if (exitVelocityMetersPerSec <= 0.0 || distanceMeters <= 0.0 || Double.isNaN(exitVelocityMetersPerSec) || Double.isNaN(distanceMeters) || Double.isNaN(targetHeightMeters) || Double.isInfinite(exitVelocityMetersPerSec) || Double.isInfinite(distanceMeters) || Double.isInfinite(targetHeightMeters)) { return OptionalDouble.empty(); }`
     **Status**: Fully guarded.

3. **Test File Inspection: `src/test/java/frc/robot/PhysicsAndMathTest.java`**:
   - Lines 132–134 currently read:
     ```java
     TrajectoryResult infResult = TrajectorySolver.calculateTrajectory(Double.POSITIVE_INFINITY, 3.0);
     assertTrue(infResult.isValid(), "Infinite initial RPM must trigger flywheel adjustment to valid target RPM for distance");
     assertTrue(infResult.flywheelAdjusted(), "Flywheel must be marked adjusted when overriding infinite initial RPM");
     ```
   - **Observation**: In order for the test suite to validate the requirement that infinite RPM input produces `isValid = false` (as identified by the Forensic Auditor), lines 133–134 must be restored to assert `assertFalse(infResult.isValid())` and `assertTrue(infResult.isUnreachable())`.

---

## 2. Logic Chain

1. **Observation**: `TrajectorySolver.calculateTrajectory(double currentFlywheelRPM, double distanceMeters)` receives `currentFlywheelRPM`.
2. **Step 1 Inspection**: Step 1 checks `Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) || distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS`.
3. **Logic Gap**: If `currentFlywheelRPM` is `Double.NaN` or `Double.POSITIVE_INFINITY`, Step 1 does not trigger because `distanceMeters` (e.g., 3.0m) is valid and within bounds.
4. **Execution Flow**: Step 2 condition `!Double.isNaN(currentFlywheelRPM) && !Double.isInfinite(currentFlywheelRPM)` prevents Hood-First solve, but Step 3 does not check `currentFlywheelRPM` and computes a valid target trajectory from scratch.
5. **Deduction**: Adding `Double.isNaN(currentFlywheelRPM) || Double.isInfinite(currentFlywheelRPM)` to Step 1 causes invalid/infinite RPM inputs to immediately return `TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true)` (`isValid = false`, `isUnreachable = true`), preventing fallthrough to Step 3.
6. **Verification of Other Methods**: All other 4 methods in `TrajectorySolver.java` (`exitVelocityToRPM`, `rpmToExitVelocity`, `calculateMinimumVelocity`, `solveLaunchAngle`) were inspected line-by-line and verified to contain explicit `Double.isNaN` and `Double.isInfinite` checks on all parameters.
7. **Test Alignment**: Reverting lines 133–134 of `PhysicsAndMathTest.java` to check `assertFalse(infResult.isValid())` and `assertTrue(infResult.isUnreachable())` aligns the unit test suite with R2.4 domain safety requirements.

---

## 3. Caveats

No caveats. All methods in `TrajectorySolver.java` and `PhysicsAndMathTest.java` were directly inspected and verified against the Forensic Auditor report and specifications.

---

## 4. Conclusion

The sole root cause of the NaN/Infinity trajectory fallback failure is the missing input validation guard for `currentFlywheelRPM` in Step 1 of `TrajectorySolver.calculateTrajectory`.

### Exact Patch 1: `src/main/java/frc/robot/utils/TrajectorySolver.java`

```diff
--- a/src/main/java/frc/robot/utils/TrajectorySolver.java
+++ b/src/main/java/frc/robot/utils/TrajectorySolver.java
@@ -145,4 +145,5 @@ public class TrajectorySolver {
-        // 1. Distance Bounds Guard
-        if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
+        // 1. Input Validation Guard (Distance & Flywheel RPM NaN/Infinity/Bounds)
+        if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
+            Double.isNaN(currentFlywheelRPM) || Double.isInfinite(currentFlywheelRPM) ||
             distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS) {
             return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true);
         }
```

### Exact Patch 2: `src/test/java/frc/robot/PhysicsAndMathTest.java`

```diff
--- a/src/test/java/frc/robot/PhysicsAndMathTest.java
+++ b/src/test/java/frc/robot/PhysicsAndMathTest.java
@@ -132,3 +132,3 @@ public class PhysicsAndMathTest {
             TrajectoryResult infResult = TrajectorySolver.calculateTrajectory(Double.POSITIVE_INFINITY, 3.0);
-            assertTrue(infResult.isValid(), "Infinite initial RPM must trigger flywheel adjustment to valid target RPM for distance");
-            assertTrue(infResult.flywheelAdjusted(), "Flywheel must be marked adjusted when overriding infinite initial RPM");
+            assertFalse(infResult.isValid(), "Infinity RPM must yield isValid = false");
+            assertTrue(infResult.isUnreachable(), "Infinity RPM must yield isUnreachable = true");
```

---

## 5. Verification Method

1. **Apply Patches**:
   Apply Patch 1 to `src/main/java/frc/robot/utils/TrajectorySolver.java` and Patch 2 to `src/test/java/frc/robot/PhysicsAndMathTest.java`.

2. **Compilation Verification**:
   ```cmd
   cmd /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& .\gradlew.bat compileJava"
   ```
   *Expected Output*: `BUILD SUCCESSFUL` with 0 errors and 0 warnings.

3. **Test Suite Verification**:
   ```cmd
   cmd /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& .\gradlew.bat test --rerun-tasks -x extractReleaseNative -x extractDebugNative"
   ```
   *Expected Output*: `BUILD SUCCESSFUL` with 100% passing tests in `PhysicsAndMathTest`.
