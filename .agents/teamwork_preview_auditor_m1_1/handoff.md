# Forensic Audit & Handoff Report — Milestone 1

**Work Product**: `src/main/java/frc/robot/utils/TrajectorySolver.java`, `src/test/java/frc/robot/PhysicsAndMathTest.java`  
**Profile**: General Project  
**Integrity Mode**: `development` (per `ORIGINAL_REQUEST.md`)  
**Verdict**: `INTEGRITY VIOLATION`

---

## 1. Observation

### Command Executions & Results
1. **Compilation Command**:
   - Command: `cmd /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& .\gradlew.bat compileJava"`
   - Result: `BUILD SUCCESSFUL in 1s` (0 errors, 0 warnings).

2. **Test Command**:
   - Command: `cmd /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& .\gradlew.bat test --rerun-tasks -x extractReleaseNative -x extractDebugNative"`
   - Result: `BUILD FAILED` with 1 failing test in `Tier2_DomainValidationAndFallbackTests`.

### Verbatim Test Failure Log
- **File**: `build/test-results/test/TEST-frc.robot.PhysicsAndMathTest$Tier2_DomainValidationAndFallbackTests.xml`
- **Failure Output**:
  ```xml
  <testcase name="NaN and Infinity safe fallbacks" classname="frc.robot.PhysicsAndMathTest$Tier2_DomainValidationAndFallbackTests" time="0.038">
    <failure message="org.opentest4j.AssertionFailedError: Infinity RPM must yield isValid = false ==&gt; expected: &lt;false&gt; but was: &lt;true&gt;" type="org.opentest4j.AssertionFailedError">org.opentest4j.AssertionFailedError: Infinity RPM must yield isValid = false ==&gt; expected: &lt;false&gt; but was: &lt;true&gt;
  	at app//org.junit.jupiter.api.AssertionFailureBuilder.build(AssertionFailureBuilder.java:151)
  	at app//org.junit.jupiter.api.AssertionFailureBuilder.buildAndThrow(AssertionFailureBuilder.java:132)
  	at app//org.junit.jupiter.api.AssertFalse.failNotFalse(AssertFalse.java:63)
  	at app//org.junit.jupiter.api.AssertFalse.assertFalse(AssertFalse.java:36)
  	at app//org.junit.jupiter.api.Assertions.assertFalse(Assertions.java:239)
  	at app//frc.robot.PhysicsAndMathTest$Tier2_DomainValidationAndFallbackTests.testNaNAndInfinityHandling(PhysicsAndMathTest.java:133)
  ```

### Static Code Observations
1. **`TrajectorySolver.java` lines 142–153**:
   ```java
   public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters) {
       double targetHeightMeters = Units.inchesToMeters(Constants.Hood.ScoreHeight);

       // 1. Distance Bounds Guard
       if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
           distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS) {
           return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true);
       }
   ```
   - Step 1 checks `distanceMeters` for `NaN` and `Infinity`, but does **not** guard against `Double.isNaN(currentFlywheelRPM)` or `Double.isInfinite(currentFlywheelRPM)`.

2. **`TrajectorySolver.java` lines 152–163**:
   ```java
       // 2. Attempt Hood-First Solve at current flywheel RPM
       if (currentFlywheelRPM > 0.0 && !Double.isNaN(currentFlywheelRPM) && !Double.isInfinite(currentFlywheelRPM)) {
           ...
       }
   ```
   - When `currentFlywheelRPM = Double.POSITIVE_INFINITY`, `!Double.isInfinite(currentFlywheelRPM)` evaluates to `false`, causing Step 2 (hood-first calculation) to be skipped.

3. **`TrajectorySolver.java` lines 165–186**:
   ```java
       // 3. Flywheel Speed Adjustment Needed
       double optAngle = (Math.PI / 4.0) + 0.5 * Math.atan2(targetHeightMeters, distanceMeters);
       ...
       double targetRPM = Math.max(MIN_FLYWHEEL_RPM, Math.min(MAX_FLYWHEEL_RPM, exitVelocityToRPM(v0Required)));
       double v0Adjusted = rpmToExitVelocity(targetRPM);
       OptionalDouble adjustedAngleOpt = solveLaunchAngle(v0Adjusted, distanceMeters, targetHeightMeters);

       if (adjustedAngleOpt.isPresent()) {
           double thetaAdj = adjustedAngleOpt.getAsDouble();
           if (thetaAdj >= MIN_HOOD_ANGLE_RAD && thetaAdj <= MAX_HOOD_ANGLE_RAD) {
               return new TrajectoryResult(thetaAdj, targetRPM, true, true, false);
           }
       }
   ```
   - Because Step 2 was skipped, execution falls through to Step 3, which recalculates an adjusted RPM (~3500 RPM for 3.0m), ignoring the invalid `currentFlywheelRPM` input, and successfully returns `isValid = true`.

4. **`PhysicsAndMathTest.java` lines 123–134**:
   ```java
   @Test
   @DisplayName("NaN and Infinity safe fallbacks")
   void testNaNAndInfinityHandling() {
       ...
       TrajectoryResult infResult = TrajectorySolver.calculateTrajectory(Double.POSITIVE_INFINITY, 3.0);
       assertFalse(infResult.isValid(), "Infinity RPM must yield isValid = false");
   }
   ```
   - The test asserts that `calculateTrajectory(Double.POSITIVE_INFINITY, 3.0)` must return `isValid = false`, but due to the missing input validation guard in `calculateTrajectory`, it returns `isValid = true`.

5. **Static Code Inspection for Cheating / Facades**:
   - `TrajectorySolver.java` contains genuine closed-form physics calculations:
     - Minimum velocity: $v_{\min} = \sqrt{g(h + \sqrt{d^2 + h^2})}$
     - Closed-form angle solution: $\theta = \arctan\left(\frac{v_0^2 - \sqrt{v_0^4 - g(gd^2 + 2hv_0^2)}}{gd}\right)$
     - Velocity <-> RPM conversions using wheel diameter and compression ratio.
   - No hardcoded lookup tables, dummy returns, or test-input-tailored conditional branches were found.
   - `PhysicsAndMathTest.java` contains genuine mathematical assertions verifying forward projectile kinematics $y(d) = h$, unit conversion roundtrips, domain boundaries, and stability. No tautological assertions (`assertTrue(true)`) were found.

---

## 2. Logic Chain

1. **Premise**: Per the Integrity Forensics framework, a work product must build successfully and pass all unit tests without failure. A single test failure constitutes a verification failure.
2. **Observation**: Executing `./gradlew.bat test --rerun-tasks -x extractReleaseNative -x extractDebugNative` resulted in `BUILD FAILED` with 1 failing unit test: `testNaNAndInfinityHandling`.
3. **Analysis**: The failure occurs because `TrajectorySolver.calculateTrajectory(double currentFlywheelRPM, double distanceMeters)` fails to guard against `Double.isNaN(currentFlywheelRPM)` or `Double.isInfinite(currentFlywheelRPM)` in its initial validation phase (Step 1). As a result, when passed `Double.POSITIVE_INFINITY`, it skips Step 2 and proceeds to Step 3 where it overrides the RPM to a valid finite value (~3500 RPM) and returns `isValid = true` instead of returning `isValid = false` with fallback values.
4. **Conclusion**: Although the static implementation of `TrajectorySolver.java` contains genuine physics logic with no facades or hardcoding, the work product fails behavioral test execution. Therefore, the work product must be rejected with a verdict of `INTEGRITY VIOLATION`.

---

## 3. Caveats

- No caveats. All source files, test suites, and Gradle execution outputs were directly inspected and empirically verified.

---

## 4. Conclusion

**Verdict**: `INTEGRITY VIOLATION`

The implementation in `TrajectorySolver.java` passes static code integrity analysis (dynamic math calculations, no lookup tables or facades), and `PhysicsAndMathTest.java` contains authentic physics tests. However, runtime verification failed because `calculateTrajectory(Double.POSITIVE_INFINITY, 3.0)` returns `isValid = true` due to an unhandled infinite RPM input guard bug in `TrajectorySolver.java:146`.

**Action Needed by Implementer**:
Add input guard in `TrajectorySolver.calculateTrajectory`:
```java
if (Double.isNaN(currentFlywheelRPM) || Double.isInfinite(currentFlywheelRPM)) {
    return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, false);
}
```
or in Step 1 distance guard:
```java
if (Double.isNaN(distanceMeters) || Double.isInfinite(distanceMeters) ||
    Double.isNaN(currentFlywheelRPM) || Double.isInfinite(currentFlywheelRPM) ||
    distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS) {
    return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true);
}
```

---

## 5. Verification Method

To independently verify this finding:

1. **Compilation**:
   ```cmd
   cmd /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& .\gradlew.bat compileJava"
   ```
   Expect: `BUILD SUCCESSFUL`

2. **Test Execution**:
   ```cmd
   cmd /c "set JAVA_HOME=C:\Users\Public\wpilib\2026\jdk&& .\gradlew.bat test --rerun-tasks -x extractReleaseNative -x extractDebugNative"
   ```
   Expect: `BUILD FAILED` with error `Infinity RPM must yield isValid = false ==> expected: <false> but was: <true>` in `TEST-frc.robot.PhysicsAndMathTest$Tier2_DomainValidationAndFallbackTests.xml`.
