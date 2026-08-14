# Handoff Report — Explorer Survey 3 (Trajectory Math & Build/Test Infra Focus)

## 1. Observation

### 1.1 Existing Math and Trajectory Calculations
Investigation of the existing codebase identified the following locations where ballistic calculations and shooter setpoints currently reside:

1. **`SUB_Hood.java` (Lines 45–48)**:
   ```java
   public static double findoptimalangle(final double distance) {
       double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
       return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
   }
   ```
   *Observation*: This computes $\theta_{\text{opt}} = \frac{\pi}{4} + \frac{1}{2}\arctan\left(\frac{h}{d}\right)$, which is the angle that minimizes launch speed $v$ for target height $h$ and horizontal distance $d$. It does **not** solve for launch angle $\theta$ given a fixed/current flywheel velocity $v$.

2. **`SUB_Shooter.java` (Lines 111–133)**:
   ```java
   public static double findoptimalRPM(final double distance, final double angleRadians) {
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
   }
   ```
   *Observation*: This calculates exit velocity $v_0$ and RPM given distance $d$ and launch angle $\theta$. It uses physical constants from `Constants.java`:
   - `Constants.Hood.ScoreHeight` = `55` inches ($\approx 1.397\text{ m}$)
   - `Constants.Shooter.ShooterDiameter` = `3.0` inches ($\approx 0.0762\text{ m}$)
   - `Constants.Shooter.kSHOOTER_COMPRESSION_RATIO` = `0.8`
   - `Constants.Shooter.kGRAVITATIONAL_CONSTANT` = `9.80665\text{ m/s}^2`
   - Default fallback RPM: `Constants.Shooter.kSHOOTER_FLYWHEEL_RPM` = `1000.0`

3. **`SUB_Shooter.java` (Lines 36, 51–57, 160–163)**:
   ```java
   private final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
   // Map values: 1.63m -> 1050 RPM, 2.49m -> 1250 RPM, 3.03m -> 1390 RPM, 4.11m -> 1600 RPM, 5.35m -> 1790 RPM, 10.5m -> 2400 RPM
   ```
   *Observation*: `CMD_AimBotAuto` and `CMD_Shuttle` bypass physics trajectory calculations and call `shooter.shootMeters(distance)` which queries this empirical lookup table.

4. **Command Subsystem Dependencies**:
   - `CMD_AimBot.java` (Line 96): `addRequirements(drivetrain, metering, index, hood);` — **Does not require `SUB_Shooter`**! At Line 156 it sets hood angle `hood.setToPosition(SUB_Hood.findoptimalangle(distance))`, but never calls `shooter.setRPM(...)`.
   - `CMD_AimBotAuto.java` (Line 69): `addRequirements(drivetrain, shooter, index);` — **Does not require `SUB_Hood`**! At Line 126 it calls `shooter.shootMeters(distance)`.
   - `RobotContainer.java` (Lines 161–173): Default command on shooter sets RPM every loop iteration using `SUB_Shooter.findoptimalRPM(distance, SUB_Hood.findoptimalangle(distance))`. Default command on hood calls `hood.resetSafe()`.

5. **`PhysicsAndMathTest.java` (Lines 19–30)**:
   ```java
   @Test
   public void testOptimalAngleAndRPMCalculations() {
       for (double distance = 1.0; distance <= 8.0; distance += 0.5) {
           double angleRad = SUB_Hood.findoptimalangle(distance);
           assertTrue(angleRad > 0 && angleRad < Math.PI / 2.0, "Angle should be physically valid in radians");

           double rpm = SUB_Shooter.findoptimalRPM(distance, angleRad);
           assertFalse(Double.isNaN(rpm), "RPM should not be NaN for distance " + distance);
           assertFalse(Double.isInfinite(rpm), "RPM should not be Infinite for distance " + distance);
           assertTrue(rpm > 0 && rpm < 7000, "RPM should be positive and realistic (e.g. 500-6000 RPM) for distance " + distance + ", got " + rpm);
       }
   }
   ```
   *Observation*: Current unit test suite only verifies that `findoptimalangle` and `findoptimalRPM` produce non-NaN/non-Infinite results between 1.0m and 8.0m. It does not test fixed-flywheel angle solving, domain validation, complex root handling, or flywheel setpoint stability.

### 1.2 Build & Unit Testing Infrastructure
- **`build.gradle` configuration**:
  - Plugin: `id "edu.wpi.first.GradleRIO" version "2026.2.1"`
  - Java version: Java 17 (`JavaVersion.VERSION_17`)
  - Test framework: JUnit 5 (`org.junit.jupiter:junit-jupiter:5.10.1`)
  - Test configuration block (Lines 78–81):
    ```groovy
    test {
        useJUnitPlatform()
        systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
    }
    ```
- **Command Verification**:
  - Environment note: Execution requires `JAVA_HOME` pointing to WPILib JDK (`C:\Users\Public\wpilib\2026\jdk`).
  - Command 1: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava`
    - Result: **BUILD SUCCESSFUL**, 0 errors, 0 warnings.
  - Command 2: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative`
    - Result: **BUILD SUCCESSFUL**, all unit tests executed and passed.

---

## 2. Logic Chain

### 2.1 Derivation of Fixed-Velocity Launch Angle Physics Equation
To satisfy Requirement R2.2, we derive the exact launch angle $\theta$ given a fixed projectile exit velocity $v_0$, horizontal distance $d$, and target height differential $h$.

#### Equations of Motion:
$$x(t) = v_0 \cos(\theta) t \implies t = \frac{d}{v_0 \cos(\theta)}$$
$$y(t) = v_0 \sin(\theta) t - \frac{1}{2} g t^2$$

Substituting $t$ into $y(t) = h$:
$$h = d \tan(\theta) - \frac{g d^2}{2 v_0^2 \cos^2(\theta)}$$

Using trigonometric identity $\frac{1}{\cos^2(\theta)} = 1 + \tan^2(\theta)$:
$$h = d \tan(\theta) - \frac{g d^2}{2 v_0^2} \left(1 + \tan^2(\theta)\right)$$

Rearranging into standard quadratic form in $T = \tan(\theta)$:
$$\left(\frac{g d^2}{2 v_0^2}\right) T^2 - d T + \left(h + \frac{g d^2}{2 v_0^2}\right) = 0$$

Let $a = \frac{g d^2}{2 v_0^2}$, $b = -d$, $c = h + \frac{g d^2}{2 v_0^2}$.
Applying quadratic formula $T = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}$:
$$T = \frac{d \pm \sqrt{d^2 - 4 \left(\frac{g d^2}{2 v_0^2}\right) \left(h + \frac{g d^2}{2 v_0^2}\right)}}{2 \left(\frac{g d^2}{2 v_0^2}\right)}$$

Factoring out terms inside the square root and simplifying:
$$T = \tan(\theta) = \frac{v_0^2 \pm \sqrt{v_0^4 - g\left(g d^2 + 2 h v_0^2\right)}}{g d}$$

Taking the inverse tangent yields the required launch angle:
$$\theta = \arctan\left( \frac{v_0^2 \pm \sqrt{v_0^4 - g\left(g d^2 + 2 h v_0^2\right)}}{g d} \right)$$

#### Conversion Between Flywheel RPM and Exit Velocity $v_0$:
- Flywheel surface speed: $v_{\text{surface}} = \left(\frac{\text{RPM}}{60}\right) \cdot \pi \cdot D_{\text{wheel}}$
- Projectile exit velocity: $v_0 = v_{\text{surface}} \cdot C_{\text{comp}}$
- With $D_{\text{wheel}} = 3.0\text{ in} = 0.0762\text{ m}$ and $C_{\text{comp}} = 0.8$:
  $$v_0 = \text{RPM} \times \left(\frac{\pi \cdot 0.0762 \cdot 0.8}{60}\right) \approx \text{RPM} \times 0.003191858\text{ m/s}$$
  $$\text{RPM} = \frac{60 \cdot v_0}{\pi \cdot D_{\text{wheel}} \cdot C_{\text{comp}}} \approx v_0 \times 313.297\text{ RPM per m/s}$$

#### Trajectory Root Selection (Low vs. High Trajectory):
The quadratic yields two mathematical solutions:
1. **Low Trajectory** (minus sign `-`): $\theta_1 = \arctan\left(\frac{v_0^2 - \sqrt{\Delta}}{g d}\right)$
2. **High Trajectory** (plus sign `+`): $\theta_2 = \arctan\left(\frac{v_0^2 + \sqrt{\Delta}}{g d}\right)$

*Selection Rationale*: Low trajectory ($\theta_1$) is selected as primary setpoint because it minimizes time-of-flight (reducing drift due to robot translation during shot), maintains a flatter trajectory, and avoids ceiling/truss collisions. If $\theta_1$ violates the hood's physical minimum limit $\theta_{\min}$, $\theta_2$ can be evaluated as an alternative.

---

### 2.2 Domain Validation Analysis (Requirement R2)

#### 1. Discriminant / Complex Roots Analysis
The radicand (discriminant) is defined as:
$$\Delta(v_0, d, h) = v_0^4 - g\left(g d^2 + 2 h v_0^2\right)$$

- **$\Delta > 0$**: Two distinct real solutions exist ($\theta_1, \theta_2$).
- **$\Delta = 0$**: Exactly one solution exists (the absolute minimum launch speed for distance $d$ and height $h$).
- **$\Delta < 0$**: Real angle solutions **do not exist** (complex roots). The current flywheel velocity $v_0$ is physically insufficient to reach $(d, h)$.

#### Minimum Required Velocity ($v_{\min}$):
Setting $\Delta = 0$ and solving for $v_0^2$:
$$v_0^4 - 2 g h v_0^2 - g^2 d^2 = 0 \implies v_{\min} = \sqrt{g\left(h + \sqrt{d^2 + h^2}\right)}$$

If current flywheel speed $v_0 < v_{\min}$, the target is unreachable at $v_0$. The system must detect $\Delta < 0$ and trigger a flywheel speed increase rather than attempting an impossible hood angle calculation.

#### 2. Ascending Target Constraint ($d \tan(\theta) > h$)
Requirement R2.1 specifies $d \tan(\theta) > h$. From the trajectory equation:
$$h = d \tan(\theta) - \frac{g d^2}{2 v_0^2 \cos^2(\theta)} \implies d \tan(\theta) - h = \frac{g d^2}{2 v_0^2 \cos^2(\theta)} > 0$$
Since $g, d, v_0, \cos^2(\theta) > 0$, any real trajectory solution physically satisfies $d \tan(\theta) > h$ automatically. Checking $d \tan(\theta) > h$ serves as an explicit mathematical guard condition to reject degenerate angles.

#### 3. Distance Bounds ($1.5\text{m} \le d \le 7.0\text{m}$)
- **Lower bound ($d < 1.5\text{m}$)**: Distance is extremely short; required launch angles approach steep angles. Check against hood maximum mechanical stop $\theta_{\max}$.
- **Upper bound ($d > 7.0\text{m}$)**: Distance exceeds standard scoring zone. Required $v_0$ increases; check flywheel RPM against motor/battery safety ceiling (e.g. 6000 RPM).

#### 4. Mechanical Hood Limits & Flywheel Stability Envelope
Let $[\theta_{\min}, \theta_{\max}]$ be physical hood limits.
- **Inside envelope ($\theta_{\text{calc}} \in [\theta_{\min}, \theta_{\max}]$)**: **Keep flywheel RPM fixed!** Adjust hood to $\theta_{\text{calc}}$.
- **Below lower limit ($\theta_{\text{calc}} < \theta_{\min}$)**: Required angle is too flat for hood. Must **decrease flywheel RPM** (or clamp hood to $\theta_{\min}$ and recompute required $v_0$).
- **Above upper limit ($\theta_{\text{calc}} > \theta_{\max}$)**: Required angle is too steep for hood. Must **increase flywheel RPM** (or clamp hood to $\theta_{\max}$ and recompute required $v_0$).

#### 5. Graceful Fallback Strategy for `NaN` / Infinity
If inputs are invalid ($d \le 0$, $v_0 \le 0$, $\Delta < 0$, or floating-point errors yield `NaN`/`Infinity`):
1. Immediately flag `isTrajectoryValid = false`.
2. Do **not** pass `NaN` to TalonFX motor controllers (which causes CTRE API error / unpredicted motor behavior).
3. Fall back to nominal distance optimal angle $\theta_{\text{default}} = \left(\frac{\pi}{4}\right) + 0.5 \arctan\left(\frac{h}{d}\right)$ or home position.
4. Report status to telemetry (`SmartDashboard.putBoolean("Trajectory/Valid", false)`).

---

## 3. Caveats

1. **Read-Only Scope**: In strict compliance with guidelines, no project source files were modified during this exploration survey. All structural proposals are provided as recommendations for implementation.
2. **Physical Kinematic Model Assumptions**:
   - Neglects air resistance (aerodynamic drag) and ball spin lift (Magnus effect). In real-world FRC game piece flight, an empirical tuning factor (e.g., effective compression ratio adjustment or drag coefficient multiplier) may be required during field calibration.
   - Assumes launch height offset $h$ relative to scoring target is fixed at $55\text{ inches} - \text{robot launch height}$.
3. **Subsystem Requirements Decoupling**:
   - `CMD_AimBot` currently does not claim `SUB_Shooter` in `addRequirements()`, while `CMD_AimBotAuto` does not claim `SUB_Hood`. In order to execute unified hood-first aiming, both commands must require both `SUB_Hood` and `SUB_Shooter`.
4. **Environment Execution Requirements**:
   - Running `./gradlew.bat` in this environment requires setting `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"`.

---

## 4. Conclusion

### 4.1 Recommended Utility Architecture: `frc.robot.utils.TrajectorySolver`
To centralize ballistic physics, eliminate duplicated code, and ensure strict unit testability, create a dedicated utility class `TrajectorySolver.java` in `src/main/java/frc/robot/utils/`.

#### Class Responsibilities:
1. `solveLaunchAngle(double v0, double distance, double height)`: Returns double array or `OptionalDouble` for low/high trajectory angles given fixed $v_0$.
2. `solveRequiredVelocity(double theta, double distance, double height)`: Computes required $v_0$ given fixed angle $\theta$.
3. `isTargetReachable(double v0, double distance, double height)`: Checks $\Delta \ge 0$ and $v_0 \ge v_{\min}$.
4. `rpmToExitVelocity(double rpm)` & `exitVelocityToRPM(double v0)`: Encapsulates wheel diameter and compression ratio conversions.
5. `calculateTrajectory(double currentRPM, double distance, double hoodMinRad, double hoodMaxRad)`: Encapsulates full hood-first decision logic and returns a structured data object `TrajectoryResult`:
   - `double desiredHoodAngle`
   - `double targetFlywheelRPM`
   - `boolean isValid`
   - `boolean flywheelAdjusted`
   - `boolean isConverged`

### 4.2 Telemetry Design Recommendations (Requirement R5)
In `RobotTelemetry.java` / `SUB_Shooter` / `SUB_Hood` / `CMD_AimBot`, publish the following standardized SmartDashboard keys:
- `Trajectory/Desired Hood Angle (Deg)`
- `Trajectory/Target Flywheel RPM`
- `Trajectory/Current Flywheel RPM`
- `Trajectory/Is Target Reachable`
- `Trajectory/Convergence Status` (True when `isThetaErrorCorrect` AND `atDesiredRPM` AND `hoodAtPosition`)

---

### 4.3 Detailed Unit Test Suite Recommendations (Requirement R4)
Expand `src/test/java/frc/robot/PhysicsAndMathTest.java` with the following comprehensive test cases:

| Test Case Name | Target Distance $d$ | Velocity / RPM Input | Expected Behavior / Verification Assertion |
| :--- | :--- | :--- | :--- |
| `testAngleSolverAcrossDistanceRange` | $1.5\text{m}$ to $7.0\text{m}$ (step $0.5\text{m}$) | Fixed $2500\text{ RPM}$ ($v_0 \approx 7.98\text{ m/s}$) | Solves $\theta_1$. Verify forward consistency: $|y(d) - h| < 10^{-4}\text{m}$. Assert $\theta_1 \in (0, \pi/2)$. |
| `testUnreachableTargetComplexRoots` | $7.0\text{m}$ | Low $1000\text{ RPM}$ ($v_0 \approx 3.19\text{ m/s}$) | $\Delta < 0$. Verify solver returns `Optional.empty()` or `NaN` without exception, and flags `isValid = false`. |
| `testZeroAndNegativeInputs` | $0.0\text{m}$, $-2.0\text{m}$ | $2000\text{ RPM}$, $-500\text{ RPM}$ | Returns graceful fallback setpoint; no `ArithmeticException` or unhandled `NaN`. |
| `testFlywheelStabilityInHoodEnvelope` | Perturb $d$ from $3.0\text{m} \to 3.2\text{m} \to 3.5\text{m}$ | Initial $2500\text{ RPM}$ | $\theta_{\text{calc}}$ remains within $[\theta_{\min}, \theta_{\max}]$. Assert target RPM setpoint remains **unchanged at 2500 RPM**. |
| `testFlywheelAdjustmentOutsideHoodEnvelope` | $6.5\text{m}$ (large distance) | Initial $1200\text{ RPM}$ (too slow) | $\theta_{\text{calc}} > \theta_{\max}$ or $\Delta < 0$. Assert system increases target RPM setpoint until $\theta \le \theta_{\max}$. |

---

## 5. Verification Method

### 5.1 Independent Build and Test Execution
To verify the existing project infrastructure and any subsequent implementation:

1. **Compilation Check**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat compileJava
   ```
   *Success Condition*: Command completes with `BUILD SUCCESSFUL` and 0 compiler warnings/errors.

2. **Unit Test Suite Execution**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
   *Success Condition*: All unit tests in `PhysicsAndMathTest.java` pass cleanly.

### 5.2 Source Files to Inspect
- `src/main/java/frc/robot/subsystems/SUB_Hood.java`
- `src/main/java/frc/robot/subsystems/SUB_Shooter.java`
- `src/main/java/frc/robot/commands/CMD_AimBot.java`
- `src/main/java/frc/robot/commands/CMD_AimBotAuto.java`
- `src/main/java/frc/robot/commands/CMD_Shuttle.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/test/java/frc/robot/PhysicsAndMathTest.java`

### 5.3 Invalidation Conditions
The investigation findings and proposed math model would be invalidated if:
- The mechanical hood gear ratio or physical stop limits change, altering $\theta_{\min}$ or $\theta_{\max}$.
- The shooter flywheel diameter ($3.0\text{ in}$) or compression ratio ($0.8$) constants are modified in `Constants.java`.
