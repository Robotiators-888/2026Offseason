# Handoff Report — Explorer 2 (Milestone 1: TrajectorySolver & Domain Math)

## 1. Observation

### 1.1 Source Code and Requirements Analysis
1. **`ORIGINAL_REQUEST.md`**:
   - Requirement R1: Hood-first trajectory controller prioritizing hood angle adjustment over flywheel velocity changes to preserve flywheel inertia and reduce battery draw.
   - Requirement R2: Closed-form launch angle solver $\theta = \arctan\left(\frac{v_0^2 - \sqrt{v_0^4 - g(gd^2 + 2hv_0^2)}}{gd}\right)$, domain validation ($d\tan\theta > h$, discriminant $\Delta \ge 0$), distance bounds $[1.5\text{m}, 7.0\text{m}]$, and protection against `NaN`/`Infinity`.

2. **`PROJECT.md` & `SCOPE.md`**:
   - Assigns Feature 1 (TrajectorySolver Physics Utility) and Feature 2 (Trajectory Domain Validation & Safety) to Milestone 1.
   - Mandates interface contract for `frc.robot.utils.TrajectorySolver`:
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

3. **`Constants.java`**:
   - `Shooter.ShooterDiameter` = `3.0` inches ($\approx 0.0762\text{ m}$)
   - `Shooter.kSHOOTER_COMPRESSION_RATIO` = `0.8`
   - `Shooter.kGRAVITATIONAL_CONSTANT` = `9.80665\text{ m/s}^2`
   - `Shooter.kSHOOTER_FLYWHEEL_RPM` = `1000.0`
   - `Hood.ScoreHeight` = `55.0` inches ($\approx 1.397\text{ m}$)

4. **Build & Test Infrastructure**:
   - Environment JDK path: `C:\Users\Public\wpilib\2026\jdk`
   - Verification command: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava` succeeds with 0 errors/warnings.
   - Unit test command: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative` executes and passes cleanly.

---

## 2. Logic Chain

### 2.1 Wheel Speed & Exit Velocity Math Derivations

#### Surface Speed vs. Exit Velocity with Compression:
- Wheel diameter $D = 3.0\text{ in} = 0.0762\text{ m}$.
- Compression ratio $C_{\text{comp}} = 0.8$.
- Wheel angular speed at $N\text{ RPM}$:
  $$\omega = \frac{N \times 2\pi}{60}\text{ rad/s}$$
- Surface linear speed:
  $$v_{\text{surface}} = \omega \cdot \frac{D}{2} = \frac{N \cdot \pi \cdot D}{60}$$
- Projectile launch exit velocity ($v_0$) accounting for compression:
  $$v_0 = v_{\text{surface}} \cdot C_{\text{comp}} = \frac{N \cdot \pi \cdot D \cdot C_{\text{comp}}}{60}$$

#### Forward Conversion (`rpmToExitVelocity`):
$$v_0(\text{rpm}) = \text{rpm} \times \frac{\pi \cdot D \cdot C_{\text{comp}}}{60} = \text{rpm} \times \frac{\pi \cdot 0.0762 \cdot 0.8}{60} \approx \text{rpm} \times 0.003191858136\text{ m/s}$$
If $\text{rpm} \le 0$, return $0.0$.

#### Inverse Conversion (`exitVelocityToRPM`):
$$\text{RPM}(v_0) = \frac{60 \cdot v_0}{\pi \cdot D \cdot C_{\text{comp}}} = v_0 \times \frac{60}{\pi \cdot 0.0762 \cdot 0.8} \approx v_0 \times 313.297433\text{ RPM per m/s}$$
If $v_0 \le 0$, return $0.0$.

---

### 2.2 Minimum Launch Velocity Math Derivation (`calculateMinimumVelocity`)

The 2D projectile kinematic equation:
$$h = d \tan(\theta) - \frac{g d^2}{2 v_0^2 \cos^2(\theta)}$$
Using $\frac{1}{\cos^2(\theta)} = 1 + \tan^2(\theta)$:
$$\left(\frac{g d^2}{2 v_0^2}\right) \tan^2(\theta) - d \tan(\theta) + \left( h + \frac{g d^2}{2 v_0^2} \right) = 0$$

Setting discriminant $\Delta = 0$ gives the absolute minimum exit velocity $v_{\min}$ required to reach $(d, h)$:
$$\Delta = d^2 - 4 \left(\frac{g d^2}{2 v_0^2}\right) \left( h + \frac{g d^2}{2 v_0^2} \right) = 0$$
$$v_0^4 - 2 g h v_0^2 - g^2 d^2 = 0$$

Solving quadratic in $u = v_0^2$:
$$u = g h + g \sqrt{d^2 + h^2} = g \left( h + \sqrt{d^2 + h^2} \right)$$
$$v_{\min} = \sqrt{g \left( h + \sqrt{d^2 + h^2} \right)}$$

For `calculateMinimumVelocity(double distanceMeters, double targetHeightMeters)`:
- If $distanceMeters \le 0$ and $targetHeightMeters \le 0$, return $0.0$.
- Standard result: `Math.sqrt(g * (targetHeightMeters + Math.sqrt(distanceMeters * distanceMeters + targetHeightMeters * targetHeightMeters)))`.

---

### 2.3 Closed-Form Fixed-Velocity Launch Angle Derivation (`solveLaunchAngle`)

Solving the quadratic in $\tan(\theta)$:
$$\tan(\theta) = \frac{v_0^2 \pm \sqrt{v_0^4 - g(g d^2 + 2 h v_0^2)}}{g d}$$

Selecting the lower trajectory angle (minus sign `-`) to minimize time-of-flight:
$$\theta = \arctan\left( \frac{v_0^2 - \sqrt{v_0^4 - g(g d^2 + 2 h v_0^2)}}{g d} \right)$$

#### Domain & Guard Checks for `solveLaunchAngle`:
1. **Invalid Parameters**: If $v_0 \le 0$ or $d \le 0$, return `OptionalDouble.empty()`.
2. **Discriminant Check**: Let $\Delta = v_0^4 - g(g d^2 + 2 h v_0^2)$.
   - If $\Delta < 0$, real solutions do not exist ($v_0 < v_{\min}$). Return `OptionalDouble.empty()`.
3. **Ascending Target Guard**:
   - Check $d \tan(\theta) > h$. If $d \tan(\theta) \le h$ or $\theta \le 0$ or $\theta \ge \frac{\pi}{2}$, return `OptionalDouble.empty()`.
4. **NaN/Infinity Guard**:
   - If `Double.isNaN(theta)` or `Double.isInfinite(theta)`, return `OptionalDouble.empty()`.
5. Return `OptionalDouble.of(theta)`.

---

### 2.4 Trajectory Solver Algorithm & Inertia-Preserving Logic (`calculateTrajectory`)

#### Core Objectives:
- Priority 1: Keep current flywheel speed fixed (`flywheelAdjusted = false`) whenever distance perturbations can be accommodated by adjusting the hood angle within $[\theta_{\min}, \theta_{\max}]$.
- Priority 2: Only change flywheel RPM when current speed cannot physically reach target ($\Delta < 0$) or requires hood angle outside $[\theta_{\min}, \theta_{\max}]$.

#### Decision Tree for `calculateTrajectory(double currentFlywheelRPM, double distanceMeters)`:

```
[Input: currentFlywheelRPM, distanceMeters]
                      │
        ┌─────────────┴─────────────┐
 [d < 1.5m or d > 7.0m?]       [1.5m <= d <= 7.0m]
        │                           │
        ▼                           ▼
[Return invalid fallback]    [Compute v_min = calculateMinimumVelocity(d, h)]
(isValid=false,             [Compute rpm_min = exitVelocityToRPM(v_min)]
 isUnreachable=true)                │
                                    ▼
                     [Can current RPM solve launch angle?]
                     theta = solveLaunchAngle(v0_current, d, h)
                                    │
                     ┌──────────────┴──────────────┐
             [theta exists &                [theta does NOT exist OR
        theta in [theta_min, theta_max]]   theta NOT in [theta_min, theta_max]]
                     │                             │
                     ▼                             ▼
       ┌───────────────────────────┐ ┌───────────────────────────┐
       │ HOOD-FIRST INERTIA HOLD   │ │ FLYWHEEL RPM ADJUSTMENT   │
       │ targetRPM = currentRPM    │ │ Compute RPM_target for d  │
       │ flywheelAdjusted = false  │ │ flywheelAdjusted = true   │
       │ isValid = true            │ │ Re-solve theta_new        │
       │ isUnreachable = false     │ │ If valid: isValid=true    │
       └───────────────────────────┘ └───────────────────────────┘
```

#### Detailed Algorithm Steps:
1. **Target Height Constant**: $h = \text{Units.inchesToMeters}(55.0) \approx 1.397\text{ m}$.
2. **Bounds Enforcement**:
   - If $d < 1.5\text{ m}$ or $d > 7.0\text{ m}$, return `TrajectoryResult(DEFAULT_HOOD_ANGLE, DEFAULT_RPM, false, false, true)`.
3. **Attempt Inertia-Preserving Solve (Fixed Flywheel Speed)**:
   - Convert `currentFlywheelRPM` to exit velocity $v_{0,\text{curr}} = \text{rpmToExitVelocity}(\text{currentFlywheelRPM})$.
   - Call `solveLaunchAngle(v0_curr, distanceMeters, h)`.
   - If present, let $\theta_{\text{calc}} = \text{result.getAsDouble()}$.
   - Check if $\theta_{\text{min}} \le \theta_{\text{calc}} \le \theta_{\text{max}}$ (where $\theta_{\text{min}} = 15^\circ \approx 0.2618\text{ rad}$, $\theta_{\text{max}} = 75^\circ \approx 1.3090\text{ rad}$):
     - **Match found!** Return `TrajectoryResult(theta_calc, currentFlywheelRPM, true, false, false)`.
4. **Flywheel Speed Adjustment Needed**:
   - If current speed cannot hit target or requires out-of-bounds hood angle:
     - Determine ideal target launch angle $\theta_{\text{opt}} = \frac{\pi}{4} + 0.5 \arctan\left(\frac{h}{d}\right)$.
     - Calculate required exit velocity $v_{\text{target}} = \frac{1}{\cos(\theta_{\text{opt}})} \sqrt{\frac{g d^2}{2 (d \tan(\theta_{\text{opt}}) - h)}}$.
     - Convert $v_{\text{target}}$ to $\text{RPM}_{\text{target}} = \text{exitVelocityToRPM}(v_{\text{target}})$.
     - Clamp $\text{RPM}_{\text{target}}$ between `MIN_FLYWHEEL_RPM` (500.0) and `MAX_FLYWHEEL_RPM` (6000.0).
     - Re-solve launch angle $\theta_{\text{new}} = \text{solveLaunchAngle}(\text{rpmToExitVelocity}(\text{RPM}_{\text{target}}), d, h)$.
     - If $\theta_{\text{new}}$ is present and within $[\theta_{\min}, \theta_{\max}]$:
       - Return `TrajectoryResult(theta_new.getAsDouble(), RPM_target, true, true, false)`.
     - Else:
       - Target is physically unreachable within constraints.
       - Return `TrajectoryResult(DEFAULT_HOOD_ANGLE, DEFAULT_RPM, false, true, true)`.

---

## 3. Caveats

1. **Read-Only Scope**: This analysis and code specification are provided for implementation by Builder agents. No project files under `src/main/java/frc/robot/` were directly modified.
2. **Ideal Ballistic Kinematics**:
   - Model assumes parabolic trajectory in vacuum (zero air resistance / drag).
   - In real FRC field testing, effective compression ratio $C_{\text{comp}} = 0.8$ serves as the empirical tuning knob to account for surface friction and air resistance.
3. **Hood Limit Constants**:
   - $[\theta_{\min}, \theta_{\max}]$ default to $[15^\circ, 75^\circ]$ ($[0.2618\text{ rad}, 1.3090\text{ rad}]$). These soft limits should align with physical mechanical stops configured in `Constants.java`.

---

## 4. Conclusion & Complete Proposed Code Specification

### Proposed Source File: `src/main/java/frc/robot/utils/TrajectorySolver.java`

```java
package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.OptionalDouble;

/**
 * TrajectorySolver handles 2D ballistic kinematics for fixed-flywheel hood aiming.
 * Centralizes velocity conversions, minimum velocity calculations, closed-form
 * launch angle quadratic solving, domain boundary checks, and inertia-preserving
 * flywheel/hood trajectory planning.
 */
public class TrajectorySolver {

    // Physical Hood Limits (Radians)
    public static final double MIN_HOOD_ANGLE_RAD = Units.degreesToRadians(15.0);
    public static final double MAX_HOOD_ANGLE_RAD = Units.degreesToRadians(75.0);
    public static final double DEFAULT_HOOD_ANGLE_RAD = Units.degreesToRadians(45.0);

    // Scoring Distance Limits (Meters)
    public static final double MIN_DISTANCE_METERS = 1.5;
    public static final double MAX_DISTANCE_METERS = 7.0;

    // Flywheel RPM Limits
    public static final double MIN_FLYWHEEL_RPM = 500.0;
    public static final double MAX_FLYWHEEL_RPM = 6000.0;
    public static final double DEFAULT_FLYWHEEL_RPM = Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;

    /**
     * Data record representing trajectory computation output.
     */
    public record TrajectoryResult(
        double desiredHoodAngleRad,
        double targetFlywheelRPM,
        boolean isValid,
        boolean flywheelAdjusted,
        boolean isUnreachable
    ) {}

    /**
     * Converts exit velocity v0 (m/s) to flywheel RPM.
     * RPM = (60 * v0) / (pi * D * C_comp)
     */
    public static double exitVelocityToRPM(double v0) {
        if (v0 <= 0.0 || Double.isNaN(v0) || Double.isInfinite(v0)) {
            return 0.0;
        }
        double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
        double compressionRatio = Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
        return (60.0 * v0) / (Math.PI * wheelDiameterMeters * compressionRatio);
    }

    /**
     * Converts flywheel RPM to exit velocity v0 (m/s).
     * v0 = (RPM * pi * D * C_comp) / 60
     */
    public static double rpmToExitVelocity(double rpm) {
        if (rpm <= 0.0 || Double.isNaN(rpm) || Double.isInfinite(rpm)) {
            return 0.0;
        }
        double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
        double compressionRatio = Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
        return (rpm * Math.PI * wheelDiameterMeters * compressionRatio) / 60.0;
    }

    /**
     * Calculates absolute minimum exit velocity required to hit target height h at distance d.
     * v_min = sqrt(g * (h + sqrt(d^2 + h^2)))
     */
    public static double calculateMinimumVelocity(double distanceMeters, double targetHeightMeters) {
        if (distanceMeters <= 0.0 && targetHeightMeters <= 0.0) {
            return 0.0;
        }
        double g = Constants.Shooter.kGRAVITATIONAL_CONSTANT;
        double d = Math.max(0.0, distanceMeters);
        double h = targetHeightMeters;
        double radicand = d * d + h * h;
        return Math.sqrt(g * (h + Math.sqrt(radicand)));
    }

    /**
     * Solves closed-form launch angle theta for a fixed exit velocity v0:
     * theta = atan((v0^2 - sqrt(v0^4 - g*(g*d^2 + 2*h*v0^2))) / (g*d))
     *
     * @return OptionalDouble containing launch angle in radians, or empty if complex root or out of bounds.
     */
    public static OptionalDouble solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters) {
        if (exitVelocityMetersPerSec <= 0.0 || distanceMeters <= 0.0 || Double.isNaN(exitVelocityMetersPerSec) || Double.isNaN(distanceMeters)) {
            return OptionalDouble.empty();
        }

        double v0 = exitVelocityMetersPerSec;
        double d = distanceMeters;
        double h = targetHeightMeters;
        double g = Constants.Shooter.kGRAVITATIONAL_CONSTANT;

        double v0Sq = v0 * v0;
        double v0Quad = v0Sq * v0Sq;
        double radicand = v0Quad - g * (g * d * d + 2.0 * h * v0Sq);

        // Complex roots condition (v0 < v_min)
        if (radicand < 0.0) {
            return OptionalDouble.empty();
        }

        double tanTheta = (v0Sq - Math.sqrt(radicand)) / (g * d);
        double theta = Math.atan(tanTheta);

        // Domain & safety validation checks
        if (Double.isNaN(theta) || Double.isInfinite(theta)) {
            return OptionalDouble.empty();
        }

        // Guard condition: d * tan(theta) > h
        if (d * tanTheta <= h) {
            return OptionalDouble.empty();
        }

        if (theta <= 0.0 || theta >= Math.PI / 2.0) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(theta);
    }

    /**
     * Calculates trajectory setpoints maintaining current flywheel speed when possible (hood-first).
     *
     * @param currentFlywheelRPM Current flywheel RPM of shooter.
     * @param distanceMeters Distance to target hub in meters.
     * @return TrajectoryResult record containing hood angle, target RPM, and status flags.
     */
    public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters) {
        double targetHeightMeters = Units.inchesToMeters(Constants.Hood.ScoreHeight);

        // 1. Distance Bounds Guard
        if (distanceMeters < MIN_DISTANCE_METERS || distanceMeters > MAX_DISTANCE_METERS || Double.isNaN(distanceMeters)) {
            return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, false, true);
        }

        // 2. Attempt Hood-First Solve at current flywheel RPM
        double v0Current = rpmToExitVelocity(currentFlywheelRPM);
        OptionalDouble angleOpt = solveLaunchAngle(v0Current, distanceMeters, targetHeightMeters);

        if (angleOpt.isPresent()) {
            double theta = angleOpt.getAsDouble();
            if (theta >= MIN_HOOD_ANGLE_RAD && theta <= MAX_HOOD_ANGLE_RAD) {
                // SUCCESS: Hood angle accommodates distance at fixed current RPM!
                return new TrajectoryResult(theta, currentFlywheelRPM, true, false, false);
            }
        }

        // 3. Flywheel Speed Adjustment Needed
        double optAngle = (Math.PI / 4.0) + 0.5 * Math.atan2(targetHeightMeters, distanceMeters);
        double cosOpt = Math.cos(optAngle);
        double denom = 2.0 * (distanceMeters * Math.tan(optAngle) - targetHeightMeters);

        if (denom <= 0.001 || Math.abs(cosOpt) < 0.001) {
            return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, true, true);
        }

        double g = Constants.Shooter.kGRAVITATIONAL_CONSTANT;
        double v0Required = (1.0 / cosOpt) * Math.sqrt((g * distanceMeters * distanceMeters) / denom);
        double targetRPM = Math.max(MIN_FLYWHEEL_RPM, Math.min(MAX_FLYWHEEL_RPM, exitVelocityToRPM(v0Required)));

        double v0Adjusted = rpmToExitVelocity(targetRPM);
        OptionalDouble adjustedAngleOpt = solveLaunchAngle(v0Adjusted, distanceMeters, targetHeightMeters);

        if (adjustedAngleOpt.isPresent()) {
            double thetaAdj = adjustedAngleOpt.getAsDouble();
            if (thetaAdj >= MIN_HOOD_ANGLE_RAD && thetaAdj <= MAX_HOOD_ANGLE_RAD) {
                return new TrajectoryResult(thetaAdj, targetRPM, true, true, false);
            }
        }

        // 4. Target Unreachable Fallback
        return new TrajectoryResult(DEFAULT_HOOD_ANGLE_RAD, DEFAULT_FLYWHEEL_RPM, false, true, true);
    }
}
```

---

## 5. Verification Method

### 5.1 Verification Commands
1. **Compilation**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat compileJava
   ```
   *Expected*: `BUILD SUCCESSFUL` with zero errors.

2. **Unit Test Execution**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
   *Expected*: `BUILD SUCCESSFUL`, all tests pass.

### 5.2 Specific File Targets to Inspect
- `src/main/java/frc/robot/utils/TrajectorySolver.java`
- `src/test/java/frc/robot/PhysicsAndMathTest.java`

### 5.3 Invalidation Conditions
- Changes to `ShooterDiameter` (3.0 in), `kSHOOTER_COMPRESSION_RATIO` (0.8), or `ScoreHeight` (55.0 in) in `Constants.java`.
