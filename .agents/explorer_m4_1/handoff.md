# Milestone 4 Exploration Report: Codebase & JUnit 5 Test Strategy Analysis

## 1. Observation

### 1.1 Mandatory Context & Requirement Specs
- **ORIGINAL_REQUEST.md** (`C:\Users\Robotiators\Documents\GitHub\2026Offseason\ORIGINAL_REQUEST.md`):
  - R1: Hood-First Trajectory Controller — Maintain current flywheel speed and adjust hood angle for distance fluctuations; only adjust flywheel speed when trajectory cannot be achieved within hood angle limits or across major distance boundaries.
  - R2: Ballistic Trajectory & Domain Validation — Solve launch angle $\theta = \arctan\left(\frac{v^2 \pm \sqrt{v^4 - g(g d^2 + 2 h v^2)}}{g d}\right)$, guarantee $d \tan\theta > h$, and handle `NaN`, infinite, or out-of-bounds mechanical values with safe fallback.
  - R4 / Acceptance Criteria: JUnit 5 test suite verifying trajectory solving accuracy across 1.5m to 7.0m, boundary handling without `NaN`/negative square roots, and flywheel setpoint stability under distance perturbations.
- **PROJECT.md** (`C:\Users\Robotiators\Documents\GitHub\2026Offseason\PROJECT.md`):
  - Interface contracts defined for `TrajectorySolver` (`solveLaunchAngle`, `calculateTrajectory`, `calculateMinimumVelocity`, `rpmToExitVelocity`, `exitVelocityToRPM`), `SUB_Hood` (`setToAngle`, `getHoodAngleRadians`, `atDesiredAngle`, `isLimitReached`), `SUB_Shooter` (`setRPM`, `getZoneTargetRPM`), and `HubTargetUtil` (`getDistanceToHub`).
- **TEST_INFRA.md** (`C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\testing_orch_m4\TEST_INFRA.md`):
  - Test location: `src/test/java/frc/robot/PhysicsAndMathTest.java`.
  - Required execution commands:
    - Build: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava`
    - Test: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative`
  - Required 4-tier test breakdown (Fixed-$v_0$ solver, Domain validation & fallback, Inertia-preserving controllers, Telemetry & readiness gate).

### 1.2 Current Subsystem Codebase Inspection
- **`src/main/java/frc/robot/Constants.java`**:
  - `Constants.Shooter`: `kSHOOTER_FLYWHEEL_RPM` = 1000, `ShooterDiameter` = 3.0 inches, `kSHOOTER_COMPRESSION_RATIO` = 0.8, `kGRAVITATIONAL_CONSTANT` = 9.80665.
  - `Constants.Hood`: `ScoreHeight` = 55 inches (1.397 m).
- **`src/main/java/frc/robot/subsystems/SUB_Hood.java`**:
  - Line 45-48: `public static double findoptimalangle(final double distance)` calculates `(Math.PI / 4.0) + 0.5 * Math.atan2(height, distance)`.
  - Lacks TalonFX gear ratio conversion (`kHOOD_GEAR_RATIO`), physical angle PID/soft limit enforcement $[\theta_{\min}, \theta_{\max}]$, and `TrajectorySolver` integration.
- **`src/main/java/frc/robot/subsystems/SUB_Shooter.java`**:
  - Line 111-133: `public static double findoptimalRPM(final double distance, final double angleRadians)` implements inverse velocity calculation for a given angle $\theta$. Handles `denom <= 0.001`, `cosAngle < 0.001`, and `NaN`/`Infinite` by falling back to `kSHOOTER_FLYWHEEL_RPM`.
  - Line 36-57: Uses `InterpolatingDoubleTreeMap distanceToRPM` for linear lookup.
  - Lacks discrete zone setpoint logic with hysteresis ($\pm 0.3$m) and speed-holding inertia preservation.
- **`src/main/java/frc/robot/commands/CMD_AimBot.java`**:
  - Line 156: Calls `hood.setToPosition(SUB_Hood.findoptimalangle(distance))`.
  - Line 120: Sets `SUB_Shooter.isShooting = true`.

### 1.3 Existing Test Suite Inspection
- **`src/test/java/frc/robot/PhysicsAndMathTest.java`**:
  - Lines 19-30: `testOptimalAngleAndRPMCalculations()` loops `distance` from 1.0m to 8.0m in steps of 0.5m. Asserts `angleRad > 0 && angleRad < Math.PI / 2.0`, `!Double.isNaN(rpm)`, `!Double.isInfinite(rpm)`, and `0 < rpm < 7000`.
  - Lines 32-39: `testAllianceFlipUtil()`.
  - Lines 41-47: `testAlertSystem()`.
  - **Deficiencies**: The existing test suite only tests basic range sanity for legacy heuristic formulas (`findoptimalangle` and `findoptimalRPM`). It contains zero test cases verifying fixed-$v_0$ closed-form quadratic angle solutions, trajectory trajectory height error $|y(d)-h| < 10^{-4}$ m, complex roots ($\Delta < 0$), zero/negative distance, NaN/Infinity fallback, inertia-preserving distance perturbation holding within $\pm 0.3$m hysteresis, or hood soft limit setpoint adjustments.

---

## 2. Logic Chain

1. **Premise 1**: `TEST_INFRA.md` and `PROJECT.md` establish a 4-tier testing hierarchy to validate the fixed-$v_0$ projectile physics and hood-dominant trajectory control system.
2. **Premise 2**: Existing test code in `PhysicsAndMathTest.java` only performs high-level range checks on legacy formulas (`SUB_Hood.findoptimalangle` and `SUB_Shooter.findoptimalRPM`).
3. **Step 1 (Tier 1 Formulation)**: To guarantee trajectory solving accuracy across the operating range of 1.5m to 7.0m (R2.2, R4.1), test cases must directly call `TrajectorySolver.solveLaunchAngle(v0, d, h)` at discrete distances (1.5m, 2.5m, 4.0m, 5.5m, 7.0m). For each derived angle $\theta$, the forward projectile equation $y(d) = d \tan\theta - \frac{g d^2}{2 v_0^2 \cos^2\theta}$ must be evaluated to confirm $|y(d) - h| < 10^{-4}\text{ m}$ and $d \tan\theta > h$.
4. **Step 2 (Tier 2 Formulation)**: To satisfy R2.1, R2.4, and R2.5, tests must cover non-physical and edge-case inputs:
   - **Complex Roots ($\Delta < 0$)**: When $v_0 < v_{\min} = \sqrt{g (h + \sqrt{d^2 + h^2})}$, `solveLaunchAngle` must return `OptionalDouble.empty()` and `calculateTrajectory` must return `TrajectoryResult` with `isValid = false` and `isUnreachable = true`.
   - **Zero & Negative Inputs**: Distance $d \le 0$ or velocity $v_0 \le 0$ must yield empty/invalid results without arithmetic exceptions.
   - **NaN / Infinity Inputs**: Passing `Double.NaN` or `Double.POSITIVE_INFINITY` must return safe fallback defaults (`isValid = false`, default RPM, safe stowed angle).
   - **Outer Bounds**: Distance $d < 1.5$m or $d > 7.0$m must correctly trigger boundary handling.
5. **Step 3 (Tier 3 Formulation)**: To satisfy R1.1, R1.2, R1.4, and R4.3, distance perturbations within the hood adjustment envelope ($\pm 0.3$m around zone center) must keep the flywheel target RPM constant (`flywheelAdjusted = false`) while the hood angle dynamically updates to maintain $|y(d) - h| < 10^{-4}$ m.
6. **Step 4 (Tier 4 Formulation)**: When distance changes exceed the hood angle limits $[\theta_{\min}, \theta_{\max}]$ or cross major zone boundaries ($> \pm 0.3$m), `calculateTrajectory` must update `targetFlywheelRPM` (`flywheelAdjusted = true`) to bring the required launch angle back into the valid mechanical range $[\theta_{\min}, \theta_{\max}]$.
7. **Conclusion**: Expanding `PhysicsAndMathTest.java` with structured JUnit 5 test methods corresponding to Tiers 1-4 will satisfy all acceptance criteria in `ORIGINAL_REQUEST.md`, `PROJECT.md`, and `TEST_INFRA.md`.

---

## 3. Caveats

- **Source Code Implementation Dependency**: `TrajectorySolver.java` and `HubTargetUtil.java` are contract interfaces specified in `PROJECT.md` for Milestone 1 / Milestone 3. Their full source implementation will be provided by Milestone 1/2 workers. Test suite expansion must target these contract signatures.
- **Environment Dependency**: Test execution requires the WPILib 2026 JDK (`C:\Users\Public\wpilib\2026\jdk`) and proper flags (`-x extractReleaseNative -x extractDebugNative`) to prevent native Phoenix6 / WPILib loading errors on Windows.

---

## 4. Conclusion & Recommended Test Strategy

### 4.1 Recommended JUnit 5 Test Structure in `PhysicsAndMathTest.java`

```
PhysicsAndMathTest
├── @Nested Tier1_TrajectoryAccuracyTests
│   ├── testLaunchAngleSolving_1_5m_to_7_0m()      # Tests 1.5m, 2.5m, 4.0m, 5.5m, 7.0m; asserts |y(d)-h| < 10^-4 m
│   ├── testAscendingCondition_dTanTheta_Gt_h()     # Asserts d * tan(theta) > h for all valid solutions
│   └── testRPMToExitVelocityConversion()           # Validates rpmToExitVelocity and exitVelocityToRPM round-trip
│
├── @Nested Tier2_DomainValidationAndFallbackTests
│   ├── testComplexRoots_InsufficientVelocity()     # v0 < vMin yields OptionalDouble.empty() & isValid = false
│   ├── testMinimumVelocityCalculation()            # vMin formula check and boundary behavior (vMin +/- epsilon)
│   ├── testZeroAndNegativeInputs()                 # d <= 0, v0 <= 0 yield empty/invalid trajectory result
│   ├── testNaNAndInfinityHandling()                # NaN, Inf distance/velocity yield safe fallbacks
│   └── testDistanceOuterBounds()                   # d < 1.5m and d > 7.0m boundary flag behavior
│
├── @Nested Tier3_FlywheelStabilityTests
│   ├── testDistancePerturbationWithinEnvelope()    # +/- 0.1m, +/- 0.2m, +/- 0.29m holds constant RPM
│   ├── testHoodAngleAdjustsWhileRPMFixed()        # Hood angle changes to maintain |y(d)-h| < 10^-4 m
│   └── testMultiZoneStabilityHolding()             # Verify RPM holding across different baseline distances
│
└── @Nested Tier4_FlywheelAdjustmentAndLimitsTests
    ├── testAdjustmentWhenExceedingHoodLimits()     # Required theta outside [theta_min, theta_max] triggers RPM change
    ├── testMajorZoneBoundaryTransition()           # Distance change > 0.3m updates target RPM
    ├── testHysteresisThresholdBehavior()           # Crossing zone boundary respects hysteresis offset
    └── testUnreachableTrajectoryEscalation()       # Extreme distance beyond max RPM/hood flags isUnreachable = true
```

---

## 5. Verification Method

### 5.1 Verification Commands
1. **Compilation Verification**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava
   ```
   *Expected result*: `BUILD SUCCESSFUL` with 0 warnings, 0 errors.

2. **Test Suite Verification**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
   *Expected result*: All tests in `PhysicsAndMathTest.java` pass cleanly.

### 5.2 Files to Inspect
- `src/test/java/frc/robot/PhysicsAndMathTest.java`
- `src/main/java/frc/robot/utils/TrajectorySolver.java`
- `src/main/java/frc/robot/subsystems/SUB_Hood.java`
- `src/main/java/frc/robot/subsystems/SUB_Shooter.java`

### 5.3 Invalidation Conditions
- Any derived launch angle $\theta$ producing $|y(d) - h| \ge 10^{-4}$ m for solvable inputs.
- Uncaught `ArithmeticException` or `NaN`/`Infinity` output on zero/negative or complex-root inputs.
- Target RPM fluctuation when distance moves by less than $\pm 0.3$m within valid hood limits.
- Failure of `./gradlew.bat compileJava` or `./gradlew.bat test`.
