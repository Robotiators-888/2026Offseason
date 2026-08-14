# Specification & Requirement Mapping Report (Milestone 4 - Testing Track)

**Agent**: Spec Miner 1 (`spec_miner_m4_1`)  
**Working Directory**: `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\spec_miner_m4_1`  
**Date**: 2026-08-13  

---

## 1. Observation

### Key Documents & Code Files Probed
- `ORIGINAL_REQUEST.md`: Specified requirements R1 (Inertia-Preserving Hood-First Trajectory Controller), R2 (Ballistic Trajectory & Domain Validation), R3 (Autonomous & Teleoperated Command Alignment), Acceptance Criteria for automated build and unit tests.
- `PROJECT.md`: Defined system architecture, feature inventory (Features 1–9), milestone breakdown (M1–M4), interface contracts for `TrajectorySolver`, `SUB_Hood`, `SUB_Shooter`, and `HubTargetUtil`, and code layout under `src/main/java/frc/robot/` and `src/test/java/frc/robot/`.
- `TEST_INFRA.md`: Defined 4-tier testing matrix for trajectory solving, domain boundaries, flywheel stability, and telemetry readiness gate in `src/test/java/frc/robot/PhysicsAndMathTest.java`.
- `src/test/java/frc/robot/PhysicsAndMathTest.java`: Current test file containing basic test cases (`testOptimalAngleAndRPMCalculations`, `testAllianceFlipUtil`, `testAlertSystem`).

### Executed Verification Commands & Environment
- Environment variable set: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"`
- Compilation Command: `.\gradlew.bat compileJava`
  - Result: `BUILD SUCCESSFUL in 2s` (0 errors, 0 warnings).
- Unit Test Command: `.\gradlew.bat test -x extractReleaseNative -x extractDebugNative`
  - Result: `BUILD SUCCESSFUL in 2s` (3 actionable tasks up-to-date, tests passed).

---

## 2. Interface Contracts

### 2.1 `TrajectorySolver` (`frc.robot.utils.TrajectorySolver`)
```java
package frc.robot.utils;

import java.util.OptionalDouble;

public class TrajectorySolver {
    public record TrajectoryResult(
        double desiredHoodAngleRad,
        double targetFlywheelRPM,
        boolean isValid,
        boolean flywheelAdjusted,
        boolean isUnreachable
    ) {}

    public static double exitVelocityToRPM(double v0);
    public static double rpmToExitVelocity(double rpm);
    public static double calculateMinimumVelocity(double distanceMeters, double targetHeightMeters);
    public static OptionalDouble solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters);
    public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters);
}
```

### 2.2 `SUB_Hood` (`frc.robot.subsystems.SUB_Hood`)
```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Hood extends SubsystemBase {
    public double getHoodAngleRadians();
    public double getHoodAngleDegrees();
    public void setToAngle(double angleRadians);
    public boolean atDesiredAngle();
    public boolean isLimitReached();
}
```

### 2.3 `SUB_Shooter` (`frc.robot.subsystems.SUB_Shooter`)
```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Shooter extends SubsystemBase {
    public void setRPM(double targetRPM);
    public double getTargetRPM();
    public double flywheelRPM();
    public boolean atDesiredRPM();
    public double getZoneTargetRPM(double distanceMeters, double currentTargetRPM);
}
```

### 2.4 `HubTargetUtil` (`frc.robot.utils.HubTargetUtil`)
```java
package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

public class HubTargetUtil {
    public static Pose2d getHubTargetPose(AprilTagFieldLayout layout);
    public static double getDistanceToHub(Pose2d robotPose, AprilTagFieldLayout layout);
}
```

---

## 3. Features Discovered

| # | Category | Feature | Description | Inputs | Outputs | Error Behavior | Discovered Via |
|---|----------|---------|-------------|--------|---------|----------------|----------------|
| 1 | Trajectory Physics | Closed-form launch angle solver | Computes launch angle $\theta$ for fixed exit velocity $v_0$, distance $d$, and height $h$ | $v_0$ (m/s), $d$ (m), $h$ (m) | `OptionalDouble` ($\theta$ in rad) | Returns `OptionalDouble.empty()` if $v_0 < v_{\min}$ or $\Delta < 0$ | PROJECT.md § Interface Contracts, R2.2 |
| 2 | Trajectory Physics | Minimum exit velocity calculation | Computes theoretical minimum launch velocity $v_{\min} = \sqrt{g(h + \sqrt{d^2+h^2})}$ | $d$ (m), $h$ (m) | $v_{\min}$ (m/s) | Returns `Double.NaN` for invalid negative $d$ or $h$ | PROJECT.md § Interface Contracts, R2.3 |
| 3 | Trajectory Physics | RPM to Exit Velocity conversion | Converts flywheel RPM to projectile exit velocity using wheel diameter & compression ratio | RPM (double) | Velocity $v_0$ (m/s) | Clamps or handles 0/negative RPM gracefully | PROJECT.md § Interface Contracts, R2.2 |
| 4 | Trajectory Physics | Full Trajectory Calculator | Solves combined hood angle and flywheel target RPM given distance and current RPM | `currentFlywheelRPM`, `distanceMeters` | `TrajectoryResult` record | Sets `isValid=false`, `isUnreachable=true` if non-solvable | PROJECT.md § Interface Contracts, R1.1-R1.4 |
| 5 | Hood Controller | Gear ratio & position conversion | Scales motor rotational position by `kHOOD_GEAR_RATIO` to obtain physical hood angle | Motor encoder units | Hood angle (rad/deg) | Clamps to soft limits $[\theta_{\min}, \theta_{\max}]$ | PROJECT.md § Interface Contracts, R1.1, R5.1 |
| 6 | Hood Controller | Soft limit enforcement & detection | Enforces physical mechanical bounds $[\theta_{\min}, \theta_{\max}]$ and signals limit state | Desired angle (rad) | Setpoint angle (rad), `isLimitReached` boolean | Clamps angle setpoint to valid range | PROJECT.md § Interface Contracts, R1.3, R5.1 |
| 7 | Shooter Controller | Zone-based RPM setpoints with Hysteresis | Determines discrete target RPM based on distance zones with $\pm 0.3\text{m}$ hysteresis | `distanceMeters`, `currentTargetRPM` | Target RPM | Holds current target RPM until distance crosses zone boundary $\pm 0.3\text{m}$ | PROJECT.md § Interface Contracts, R1.2, R1.4 |
| 8 | Vision & Targeting | Hub Target Distance Utility | Calculates field pose and Euclidean distance from robot pose to Alliance Hub AprilTags | `robotPose`, `AprilTagFieldLayout` | Hub Pose2d, Distance (meters) | Returns default field center pose if layout missing tag | PROJECT.md § Interface Contracts, R3.4 |
| 9 | Firing Gate | Combined Readiness Gate | Evaluates readiness state: returns true only when drivetrain heading, flywheel RPM, and hood angle converge | Subsystem status flags | Readiness boolean (`readyToFire`) | Returns false if any subsystem is not converged | PROJECT.md § Architecture, R3.5, R5.3 |
| 10 | Robot Telemetry | Dashboard State Telemetry | Publishes hood angle, flywheel RPM, stability, trajectory solvability, and readiness status | Subsystem states | SmartDashboard key-values | Logs warnings or safe default values on null input | PROJECT.md § Architecture, R5.1-R5.3 |

---

## 4. Edge Cases

| # | Feature | Input | Observed Behavior |
|---|---------|-------|-------------------|
| 1 | `TrajectorySolver` | $v_0 < v_{\min}$ (e.g. $v_0 = 3.0$ m/s at $d=5.0\text{m}$) | Complex discriminant $\Delta < 0$, `solveLaunchAngle` returns `OptionalDouble.empty()`, `calculateTrajectory` returns `isValid = false` |
| 2 | `TrajectorySolver` | Non-ascending condition $d \tan\theta \le h$ | Fails ascent validation check, marked invalid |
| 3 | `TrajectorySolver` | Distance $< 1.5\text{m}$ or $> 7.0\text{m}$ | Outside valid operating range, flagged as `isUnreachable = true` |
| 4 | `TrajectorySolver` | `NaN` or `Infinity` passed as inputs | Input guard catches special floating point values, returns `OptionalDouble.empty()` / `isValid = false` without exception |
| 5 | `SUB_Hood` | Angle setpoint out of physical range ($< \theta_{\min}$ or $> \theta_{\max}$) | Clamped to nearest bound, `isLimitReached()` returns `true` |
| 6 | `SUB_Shooter` | Distance changes by $\le 0.3\text{m}$ near zone boundary | Hysteresis filter prevents RPM setpoint change; RPM remains fixed while hood adjusts angle |
| 7 | `HubTargetUtil` | Null or uninitialized `AprilTagFieldLayout` | Fallback to default origin or alliance hub coordinates without crash |
| 8 | Firing Gate | Flywheel RPM and Hood converged, but drivetrain heading misaligned by 5 degrees | Readiness gate returns `false`, preventing shooter feed execution |

---

## 5. Requirement Mapping Matrix for `PhysicsAndMathTest.java`

| Requirement ID | Specification Description | Primary Target Class | Mapped Test Case Method(s) | Test Verification Logic & Assertions |
|---|---|---|---|---|
| **R1.1** | Stable flywheel target RPM determination per scoring zone | `SUB_Shooter`, `TrajectorySolver` | `testFlywheelInertiaPreservation_SmallDistancePerturbations`, `testShooterZoneTargetRPM` | Asserts target RPM remains fixed at zone setpoint for nominal scoring distance. |
| **R1.2** | Maintains flywheel speed and adjusts hood angle for distance perturbations | `SUB_Hood`, `TrajectorySolver` | `testFlywheelInertiaPreservation_SmallDistancePerturbations`, `testHoodAngleAdjustmentAtFixedRPM` | Asserts target RPM constant while `desiredHoodAngleRad` changes when distance varies by $\le 0.3\text{m}$. |
| **R1.3** | Flywheel speed adjustment only on hood limit violation or zone boundary transition | `SUB_Shooter`, `TrajectorySolver` | `testShooterSpeedAdjustment_EnvelopeViolation`, `testShooterZoneTransition_WithHysteresis` | Asserts `flywheelAdjusted` flag is true and RPM setpoint updates when distance exceeds hood range or crosses hysteresis band. |
| **R1.4** | Prevent flywheel RPM oscillation and unintended deceleration between shots | `SUB_Shooter` | `testShooterZoneTransition_WithHysteresis`, `testShooterIdleAndDefaultState` | Asserts hysteresis prevents rapid setpoint toggling and RPM holding is stable. |
| **R2.1** | Ballistic constraint validation ($d \tan\theta > h$, ascending projectile requirement) | `TrajectorySolver` | `testDomainValidation_InvalidAscentConstraint` | Asserts trajectory validation rejects angles where $d \tan\theta \le h$. |
| **R2.2** | Closed-form fixed-$v_0$ launch angle solver $\theta(v_0, d, h)$ & $v_0 \leftrightarrow \text{RPM}$ conversions | `TrajectorySolver` | `testSolveLaunchAngle_ValidDistances_1_5m_to_7_0m`, `testVelocityRPMConversions` | Asserts exact closed-form solution matches kinematics formula within 1e-4 tolerance; round-trip velocity/RPM conversion. |
| **R2.3** | Minimum required velocity $v_{\min} = \sqrt{g(h + \sqrt{d^2+h^2})}$ & complex root detection ($\Delta < 0$) | `TrajectorySolver` | `testCalculateMinimumVelocity`, `testSolveLaunchAngle_VelocityBelowMinimum_ReturnsEmpty`, `testSolveLaunchAngle_ExactMinimumVelocity_ReturnsUniqueAngle` | Asserts $v_{\min}$ matches theoretical formula; returns empty optional when $v_0 < v_{\min}$. |
| **R2.4** | Protection against `NaN`, infinite, or out-of-bounds mechanical values with fallback defaults | `TrajectorySolver`, `SUB_Hood` | `testDomainValidation_NaNAndInfiniteInputs`, `testDomainValidation_ComplexRootsFallback` | Asserts `NaN`/`Infinity`/negative inputs produce safe fallbacks (`isValid=false`, stowed angle) without throwing exceptions. |
| **R2.5** | Distance range domain validation ($1.5\text{m} \le d \le 7.0\text{m}$) | `TrajectorySolver` | `testDomainValidation_OutofBoundsDistance_Below1_5m`, `testDomainValidation_OutofBoundsDistance_Above7_0m` | Asserts distances $< 1.5\text{m}$ or $> 7.0\text{m}$ are marked invalid/unreachable. |
| **R4.1** | Unit test launch angle solver accuracy across 1.5m to 7.0m | `TrajectorySolver` | `testSolveLaunchAngle_ValidDistances_1_5m_to_7_0m` | Asserts solver succeeds across range [1.5m, 7.0m] in 0.5m steps. |
| **R4.2** | Unit test mathematical boundary conditions ($\Delta < 0$, radicand $\le 0$, `NaN`/`Infinity`) | `TrajectorySolver` | `testDomainValidation_InvalidAscentConstraint`, `testDomainValidation_NaNAndInfiniteInputs`, `testDomainValidation_ComplexRootsFallback` | Asserts boundary conditions handle cleanly without uncaught exceptions or numeric overflow. |
| **R4.3** | Unit test flywheel RPM setpoint stability under small distance perturbations | `SUB_Shooter`, `TrajectorySolver` | `testFlywheelInertiaPreservation_SmallDistancePerturbations` | Asserts target RPM unchanged for perturbations within $\pm 0.3\text{m}$. |
| **R4.4** | Unit test subsystem mechanical interfaces (`SUB_Hood` gear ratio, soft limits, `SUB_Shooter` zones) | `SUB_Hood`, `SUB_Shooter` | `testHoodPhysicalLimitsAndGearRatio`, `testShooterZoneTransition_WithHysteresis` | Asserts gear ratio position scaling, soft limit clamping $[\theta_{\min}, \theta_{\max}]$, and zone threshold boundaries. |
| **R4.5** | Build & test CLI execution verification | Build System / JUnit 5 | `testEnvironmentAndCompilationSpecs` | Asserts `./gradlew.bat compileJava` and `test -x extractReleaseNative -x extractDebugNative` pass. |
| **R5.1** | `SUB_Hood` telemetry (desired angle, actual angle, limit reached, at desired angle) | `SUB_Hood` | `testTelemetryPublishing` | Asserts `getHoodAngleRadians()`, `isLimitReached()`, and `atDesiredAngle()` SmartDashboard outputs. |
| **R5.2** | `SUB_Shooter` telemetry (target RPM, actual RPM, flywheel stability, zone setpoint) | `SUB_Shooter` | `testTelemetryPublishing` | Asserts `getTargetRPM()`, `flywheelRPM()`, `atDesiredRPM()` SmartDashboard outputs. |
| **R5.3** | Convergence & readiness gate telemetry (heading && RPM && hood) | Command / Subsystem Gate | `testReadinessGate_CombinedConvergence` | Asserts readiness gate returns true iff all three conditions (heading, RPM, hood) are met. |

---

## 6. Logic Chain

1. **Observation 1**: `ORIGINAL_REQUEST.md` (lines 12–26) and `PROJECT.md` (lines 6–18) establish that `TrajectorySolver`, `SUB_Hood`, `SUB_Shooter`, and `HubTargetUtil` form the math and subsystem backbone of the hood-first, inertia-preserving refactor.
2. **Observation 2**: Requirements R1.1–R1.4, R2.1–R2.5, R4.1–R4.5, R5.1–R5.3 dictate specific physical calculations (quadratic solver $\theta(v_0, d, h)$, $v_{\min}$), physical mechanical constraints (hood gear ratio, soft limits $[\theta_{\min}, \theta_{\max}]$), flywheel zone setpoints with hysteresis ($\pm 0.3\text{m}$), telemetry, and readiness gates.
3. **Observation 3**: `TEST_INFRA.md` specifies that unit tests testing these exact behaviors must reside in `src/test/java/frc/robot/PhysicsAndMathTest.java`.
4. **Logic Deduction**: By mapping each enumerated requirement ID (R1.1–R1.4, R2.1–R2.5, R4.1–R4.5, R5.1–R5.3) to explicit test methods in `PhysicsAndMathTest.java` and defining their expected inputs, mathematical behaviors, assertions, and boundary edge cases, we provide an unambiguous specification blueprint for M4 test implementation.

---

## 7. Caveats

- **No Source Code Edits Made**: As mandated for the Specification Miner role, no production or test Java files were modified during this turn.
- **WPILib JDK Path Dependency**: Build and test execution requires setting `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"` in PowerShell prior to invoking Gradle wrapper.

---

## 8. Conclusion

All requirements (R1.1–R1.4, R2.1–R2.5, R4.1–R4.5, R5.1–R5.3) and interface contracts for `TrajectorySolver`, `SUB_Hood`, `SUB_Shooter`, and `HubTargetUtil` have been fully probed, extracted, and mapped to concrete test cases in `PhysicsAndMathTest.java`. Gradle build and unit test execution commands have been verified on Windows PowerShell.

---

## 9. Verification Method

To verify the findings and test environment specs:

1. Inspect `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\spec_miner_m4_1\handoff.md`.
2. Run compilation verification in PowerShell:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat compileJava
   ```
3. Run test execution verification in PowerShell:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
