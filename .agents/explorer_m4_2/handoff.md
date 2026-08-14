# Handoff Report — Explorer 2 (Milestone 4 Testing Track)

## 1. Observation

### 1.1 Existing Codebase Implementation vs Refactor Specification
- **`src/main/java/frc/robot/Constants.java`**:
  - `Shooter.kGRAVITATIONAL_CONSTANT = 9.80665` (Line 49)
  - `Shooter.ShooterDiameter = 3.0` inches = $0.0762\text{ m}$ (Line 35)
  - `Shooter.kSHOOTER_COMPRESSION_RATIO = 0.8` (Line 47)
  - `Shooter.kSHOOTER_FLYWHEEL_RPM = 1000.0` (Line 31)
  - `Hood.ScoreHeight = 55` inches = $1.397\text{ m}$ (Line 136)
  - *Gap*: Missing constants specified in `PROJECT.md`: `kHOOD_GEAR_RATIO`, physical hood soft limits $[\theta_{\min}, \theta_{\max}]$, discrete distance zone thresholds, and hysteresis band $\pm 0.3\text{m}$.

- **`src/main/java/frc/robot/subsystems/SUB_Hood.java`**:
  - `findoptimalangle(final double distance)` (Lines 45–48):
    ```java
    double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
    return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
    ```
    This calculates an angle based on a variable-velocity minimum-energy arc rather than solving for a launch angle given a fixed flywheel velocity $v_0$.

- **`src/main/java/frc/robot/subsystems/SUB_Shooter.java`**:
  - `findoptimalRPM(final double distance, final double angleRadians)` (Lines 111–133):
    ```java
    double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
    double denom = 2.0 * (distance * Math.tan(angleRadians) - height);
    if (denom <= 0.001 || distance <= 0.1) {
        return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
    }
    double cosAngle = Math.cos(angleRadians);
    if (Math.abs(cosAngle) < 0.001) {
        return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
    }
    double exitVelocity = (1.0 / cosAngle) * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance) / denom);
    double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
    double surfaceSpeed = exitVelocity / Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
    double rps = surfaceSpeed / (Math.PI * wheelDiameterMeters);
    double exitRPM = rps * 60.0;
    if (Double.isNaN(exitRPM) || Double.isInfinite(exitRPM)) {
        return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
    }
    return exitRPM;
    ```
  - `distanceToRPM` (Lines 36, 52–57): Uses an `InterpolatingDoubleTreeMap` calibration curve rather than holding fixed RPM setpoints within discrete distance zones with hysteresis ($\pm 0.3\text{m}$).

- **`src/test/java/frc/robot/PhysicsAndMathTest.java`**:
  - `testOptimalAngleAndRPMCalculations()` (Lines 19–30): Basic loop from 1.0m to 8.0m verifying non-NaN/non-Infinite results, but lacks parameter sets for fixed-$v_0$ angle precision, complex root handling ($\Delta < 0$), hysteresis velocity holding, and boundary transition triggers.

### 1.2 `TrajectorySolver` Target Physics Interface (`PROJECT.md`)
```java
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

---

## 2. Logic Chain

### 2.1 Fixed Velocity Launch Angle Mathematics
From standard projectile motion equations:
1. Horizontal displacement: $x(t) = v_0 \cos\theta \cdot t \implies t = \frac{d}{v_0 \cos\theta}$
2. Vertical displacement: $y(t) = v_0 \sin\theta \cdot t - \frac{1}{2} g t^2 = h$
3. Substitute $t$: $h = d \tan\theta - \frac{g d^2}{2 v_0^2 \cos^2\theta}$
4. Using $\frac{1}{\cos^2\theta} = 1 + \tan^2\theta$:
   $$h = d \tan\theta - \frac{g d^2}{2 v_0^2} (1 + \tan^2\theta)$$
5. Rearranging into a quadratic equation in $T = \tan\theta$:
   $$\left(\frac{g d^2}{2 v_0^2}\right) T^2 - d T + \left(h + \frac{g d^2}{2 v_0^2}\right) = 0$$
   Multiplying by $\frac{2 v_0^2}{g d}$:
   $$\tan\theta = \frac{v_0^2 \pm \sqrt{v_0^4 - g(g d^2 + 2 h v_0^2)}}{g d}$$
6. **Discriminant**:
   $$\Delta = v_0^4 - g(g d^2 + 2 h v_0^2) = v_0^4 - 2 g h v_0^2 - g^2 d^2$$
7. **Velocity to RPM conversion**:
   $$v_{\text{wheel}} = \pi \cdot D_{\text{wheel}} \cdot \frac{\text{RPM}}{60}$$
   $$v_0 = v_{\text{wheel}} \cdot k_{\text{compression}} \implies \text{RPM} = \frac{60 \cdot v_0}{\pi \cdot D_{\text{wheel}} \cdot k_{\text{compression}}}$$
   With $D_{\text{wheel}} = 0.0762\text{ m}$ (3.0 inches) and $k_{\text{compression}} = 0.8$:
   $$\text{RPM} = \frac{60 \cdot v_0}{\pi \cdot 0.0762 \cdot 0.8} = \frac{60 \cdot v_0}{0.1915115} \approx 313.296 \cdot v_0$$
   $$v_0 = \text{RPM} \times \frac{0.1915115}{60} \approx 0.00319186 \cdot \text{RPM}$$

### 2.2 Domain Boundaries, Edge Cases, and Fallbacks
- **$\Delta < 0$ (Complex Roots)**: Occurs when $v_0 < v_{\min}$.
  - Minimum velocity requirement: $v_{\min} = \sqrt{g (h + \sqrt{d^2 + h^2})}$.
  - When $\Delta < 0$, `solveLaunchAngle` returns `OptionalDouble.empty()`, and `calculateTrajectory` sets `isValid = false`, `isUnreachable = true`, and calculates required RPM from $v_{\min}$.
- **Ascent Condition ($d \tan\theta > h$)**:
  - Direct requirement from R2.1 to guarantee projectile reaches height $h$.
  - Selecting root $\theta_+$ or verifying lower root $\theta_-$ satisfies $d \tan\theta > h$.
- **Division by Zero Protection**:
  - $d \le 0.1\text{m}$ or $v_0 \le 0.1\text{ m/s}$ returns invalid result safely without producing `Infinity` or `NaN`.
- **Mechanical Hood Limits**:
  - Hood physical limits $[\theta_{\min}, \theta_{\max}]$, e.g., $[15^\circ, 75^\circ]$ ($[0.2618\text{ rad}, 1.3090\text{ rad}]$). If solved $\theta \notin [\theta_{\min}, \theta_{\max}]$, flag `isUnreachable = true` and force RPM adjustment.

### 2.3 Flywheel Hysteresis and Boundary Transitions
- Discrete distance zones (e.g. Zone 1: 1.5m–3.0m, base 2500 RPM; Zone 2: 3.0m–5.0m, base 3500 RPM; Zone 3: 5.0m–7.0m, base 4800 RPM).
- Hysteresis band: $\pm 0.3\text{m}$.
- When current distance fluctuates within zone $\pm 0.3\text{m}$, `flywheelAdjusted` is `false`, target RPM remains fixed, and hood angle adjusts to distance.
- When distance moves beyond $3.0\text{m} - 0.3\text{m} = 2.7\text{m}$ (when moving down from Zone 2) or $5.0\text{m} + 0.3\text{m} = 5.3\text{m}$ (when moving up from Zone 2), a boundary transition triggers: `flywheelAdjusted` becomes `true` and target RPM shifts to new zone setpoint.

---

## 3. Caveats

- **Air Resistance / Drag**: The mathematical model uses idealized ballistic trajectory equations. In physical testing, Magnus effect and drag slightly alter trajectory, which can be compensated by fine-tuning target height $h_{\text{effective}}$ or compression ratio $k_{\text{compression}}$.
- **Root Selection Strategy**: Quadratic equation yields two roots ($\theta_+$ high arc and $\theta_-$ direct trajectory). The test suite assumes lower root $\theta_-$ within mechanical hood envelope $[15^\circ, 75^\circ]$ unless $\theta_-$ fails $d\tan\theta > h$.
- **Source Code Immutability**: Read-only investigation mode maintained; no changes were made to source files during analysis.

---

## 4. Conclusion & Recommended JUnit 5 Test Suite Specifications

We recommend expanding `src/test/java/frc/robot/PhysicsAndMathTest.java` with four parameterized JUnit 5 test groups:

### 4.1 Group 1: Launch Angle Precision Parameter Sets ($1.5\text{m} - 7.0\text{m}$)
Assertion Tolerance: `1e-4` radians ($0.0057^\circ$).

| Distance $d$ (m) | Flywheel RPM | Exit Velocity $v_0$ (m/s) | Target Height $h$ (m) | Expected Lower Root $\theta_-$ (rad) | Expected Lower Root $\theta_-$ (deg) | Assertion Tolerance |
|:----------------:|:------------:|:-------------------------:|:---------------------:|:------------------------------------:|:------------------------------------:|:-------------------:|
| 1.5              | 3759.5       | 12.000                    | 1.397                 | 0.8038                               | $46.05^\circ$                        | `1e-4` rad          |
| 2.0              | 3133.0       | 10.000                    | 1.397                 | 0.7165                               | $41.05^\circ$                        | `1e-4` rad          |
| 3.0              | 3759.5       | 12.000                    | 1.397                 | 0.5441                               | $31.18^\circ$                        | `1e-4` rad          |
| 5.0              | 4699.4       | 15.000                    | 1.397                 | 0.3859                               | $22.11^\circ$                        | `1e-4` rad          |
| 7.0              | 5639.3       | 18.000                    | 1.397                 | 0.3061                               | $17.54^\circ$                        | `1e-4` rad          |

### 4.2 Group 2: Delta < 0 & Out-of-Bounds Fallback Test Cases

| Scenario | Input ($d$, $v_0$ or RPM) | Condition | Expected Result / Output | Assertion |
|:---------|:--------------------------|:----------|:-------------------------|:----------|
| Insufficient Velocity | $d = 7.0\text{ m}$, $v_0 = 5.0\text{ m/s}$ (1566 RPM) | $\Delta = -4772.3 < 0$ | `solveLaunchAngle` returns `OptionalDouble.empty()` | `assertTrue(result.isEmpty())` |
| Zero Distance Guard | $d = 0.0\text{ m}$, $v_0 = 12.0\text{ m/s}$ | $d \le 0.1\text{ m}$ | `solveLaunchAngle` returns `OptionalDouble.empty()` | `assertTrue(result.isEmpty())` |
| Zero Velocity Guard | $d = 3.0\text{ m}$, $v_0 = 0.0\text{ m/s}$ | $v_0 \le 0.1\text{ m/s}$ | `solveLaunchAngle` returns `OptionalDouble.empty()` | `assertTrue(result.isEmpty())` |
| Minimum Velocity Edge | $d = 4.0\text{ m}$, $v_0 = 7.433\text{ m/s}$ | $\Delta = 0$ | Single real root $\theta = 0.9535\text{ rad}$ ($54.63^\circ$) | `assertEquals(0.9535, angle, 1e-3)` |
| Below Minimum Velocity | $d = 4.0\text{ m}$, $v_0 = 7.000\text{ m/s}$ | $v_0 < v_{\min}$ | `calculateTrajectory` flags `isUnreachable = true`, `isValid = false` | `assertFalse(traj.isValid()); assertTrue(traj.isUnreachable());` |

### 4.3 Group 3: Flywheel Hysteresis Band Test Parameter Sequence (Zone 2: 3.5m, Base 3500 RPM, Hysteresis $\pm 0.3\text{m}$)

| Step | Distance $d$ (m) | Previous RPM | Expected Target RPM | Expected `flywheelAdjusted` | Description |
|:----:|:----------------:|:------------:|:-------------------:|:--------------------------:|:------------|
| 1    | 3.5              | 0            | 3500.0              | `true`                     | Initial zone entry |
| 2    | 3.7              | 3500.0       | 3500.0              | `false`                    | $+0.2\text{m}$ step within zone |
| 3    | 3.3              | 3500.0       | 3500.0              | `false`                    | $-0.2\text{m}$ step within zone |
| 4    | 3.1              | 3500.0       | 3500.0              | `false`                    | $-0.4\text{m}$ step within zone |
| 5    | 2.9              | 3500.0       | 3500.0              | `false`                    | Below Zone 2 min (3.0m) but within hysteresis ($3.0 - 0.3 = 2.7\text{m}$) |

### 4.4 Group 4: Boundary Transition Forcing RPM Adjustment Sequence

| Step | Distance $d$ (m) | Previous RPM | Expected Target RPM | Expected `flywheelAdjusted` | Description |
|:----:|:----------------:|:------------:|:-------------------:|:--------------------------:|:------------|
| 1    | 2.6              | 3500.0       | 2500.0              | `true`                     | Crosses lower hysteresis bound ($< 2.7\text{m}$) $\to$ transition to Zone 1 |
| 2    | 5.4              | 3500.0       | 4800.0              | `true`                     | Crosses upper hysteresis bound ($> 5.3\text{m}$) $\to$ transition to Zone 3 |

---

## 5. Verification Method

### 5.1 Compilation and Execution Commands
Execute standard WPILib JDK Gradle build commands:
```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
```

### 5.2 Verification Checklist for Implementation Worker
1. `TrajectorySolver.java` correctly implements `solveLaunchAngle` using $\tan\theta = \frac{v_0^2 \pm \sqrt{v_0^4 - g(g d^2 + 2 h v_0^2)}}{g d}$.
2. `PhysicsAndMathTest.java` includes parameterized tests for distances $1.5\text{m}, 2.0\text{m}, 3.0\text{m}, 5.0\text{m}, 7.0\text{m}$ with `1e-4` tolerance.
3. Complex root test cases verify zero `NaN` or `Infinity` returns.
4. Hysteresis holding test confirms `flywheelAdjusted == false` for distance perturbations within $\pm 0.3\text{m}$.
5. Boundary transition test confirms `flywheelAdjusted == true` and target RPM update when hysteresis threshold is exceeded.
