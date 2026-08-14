# Handoff Report — Explorer 3 (Domain Validation & Safety Handling)

## 1. Observation

### 1.1 Source Context & Existing Math Codebase
An inspection of the workspace specifications (`ORIGINAL_REQUEST.md`, `PROJECT.md`, `SCOPE.md`) and existing Java source files in `src/main/java/frc/robot/` and `src/test/java/frc/robot/` revealed the following key facts:

1. **`ORIGINAL_REQUEST.md` (Lines 18–22)**:
   > "R2. Ballistic Trajectory & Domain Validation
   > 1. Ensure the launch angle $\theta$ and horizontal distance $d$ to target height $h$ satisfy ballistic trajectory constraints (e.g., $d \tan(\theta) > h$ so the projectile ascends high enough to reach the target height, avoiding non-positive radicands in the velocity formula).
   > 2. Given a fixed/current flywheel velocity $v$, solve for the required launch angle $\theta = \arctan\left(\frac{v^2 \pm \sqrt{v^4 - g(g d^2 + 2 h v^2)}}{g d}\right)$ (or numerical inversion) and set the hood angle accordingly.
   > 3. Protect against `NaN`, infinite, or out-of-bounds mechanical values with fallback defaults."

2. **`PROJECT.md` & `SCOPE.md` Interface Contract for `TrajectorySolver`**:
   - `TrajectorySolver` must provide:
     - `solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters)` -> `OptionalDouble`
     - `calculateMinimumVelocity(double distanceMeters, double targetHeightMeters)` -> `double`
     - `calculateTrajectory(double currentFlywheelRPM, double distanceMeters)` -> `TrajectoryResult`
   - `TrajectoryResult` record structure:
     ```java
     public record TrajectoryResult(
         double desiredHoodAngleRad,
         double targetFlywheelRPM,
         boolean isValid,
         boolean flywheelAdjusted,
         boolean isUnreachable
     ) {}
     ```

3. **`Constants.java` Physical Parameters**:
   - Line 34: `public static final double ShooterDiameter = 3.0;` (inches $\approx 0.0762\text{ m}$)
   - Line 47: `public static final double kSHOOTER_COMPRESSION_RATIO = 0.8;`
   - Line 49: `public static final double kGRAVITATIONAL_CONSTANT = 9.80665;` ($\text{m/s}^2$)
   - Line 136: `public static final double ScoreHeight = 55;` (inches $\approx 1.397\text{ m}$)
   - Line 31: `public static final double kSHOOTER_FLYWHEEL_RPM = 1000;`

4. **`SUB_Hood.java` (Lines 45–48)**:
   ```java
   public static double findoptimalangle(final double distance) {
       double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
       return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
   }
   ```
   *Observation*: This calculates $\theta_{\text{opt}} = \frac{\pi}{4} + \frac{1}{2}\arctan(h/d)$ which minimizes required velocity, but does **not** solve for launch angle given a fixed/current flywheel velocity $v_0$.

5. **`SUB_Shooter.java` (Lines 111–133)**:
   ```java
   public static double findoptimalRPM(final double distance, final double angleRadians) {
       double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
       double denom = 2.0 * (distance * Math.tan(angleRadians) - height);
       if (denom <= 0.001 || distance <= 0.1) {
           return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
       }
       ...
       double exitVelocity = (1.0 / cosAngle) * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance) / denom);
       ...
       if (Double.isNaN(exitRPM) || Double.isInfinite(exitRPM)) {
           return Constants.Shooter.kSHOOTER_FLYWHEEL_RPM;
       }
       return exitRPM;
   }
   ```
   *Observation*: Handles `denom <= 0.001` and `Double.isNaN` / `Double.isInfinite` by returning `1000 RPM`, but this method assumes angle is known and solves for RPM, which is the inverse of the hood-first requirement (where RPM is fixed and angle is solved).

6. **`PhysicsAndMathTest.java` (Lines 19–30)**:
   *Observation*: Existing tests only loop distance from 1.0m to 8.0m and assert that `findoptimalangle` and `findoptimalRPM` do not return `NaN` or `Infinite`. There are no tests for fixed-velocity angle solving, complex roots ($\Delta < 0$), $d \tan(\theta) > h$ guard, boundary distances ($1.5\text{m} \le d \le 7.0\text{m}$), mechanical soft limits, or flywheel RPM stability.

---

## 2. Logic Chain

### 2.1 Derivation & Analysis of Discriminant Guard ($\Delta < 0$)
The trajectory equation relating horizontal distance $d$, target height differential $h$, launch angle $\theta$, exit velocity $v_0$, and gravity $g = 9.80665\text{ m/s}^2$ is:
$$h = d \tan(\theta) - \frac{g d^2}{2 v_0^2 \cos^2(\theta)} = d \tan(\theta) - \frac{g d^2}{2 v_0^2} (1 + \tan^2(\theta))$$

Rearranging into standard quadratic form for $T = \tan(\theta)$:
$$\left(\frac{g d^2}{2 v_0^2}\right) T^2 - d T + \left(h + \frac{g d^2}{2 v_0^2}\right) = 0$$

Multiplying by $2 v_0^2$:
$$(g d^2) T^2 - (2 d v_0^2) T + (2 h v_0^2 + g d^2) = 0$$

The discriminant of this quadratic is:
$$\Delta_T = (-2 d v_0^2)^2 - 4(g d^2)(2 h v_0^2 + g d^2) = 4 d^2 \left( v_0^4 - g(g d^2 + 2 h v_0^2) \right)$$

Defining the simplified discriminant term:
$$\Delta = v_0^4 - g(g d^2 + 2 h v_0^2)$$

- **Step 1 (Real Solutions)**: When $\Delta > 0$, two distinct real angles exist:
  $$\theta_1 = \arctan\left(\frac{v_0^2 - \sqrt{\Delta}}{g d}\right) \quad \text{(Low trajectory)}, \qquad \theta_2 = \arctan\left(\frac{v_0^2 + \sqrt{\Delta}}{g d}\right) \quad \text{(High trajectory)}$$
- **Step 2 (Critical Boundary / Minimum Speed)**: When $\Delta = 0$, exactly one solution exists. Solving $\Delta = 0$ for $v_0$ yields the absolute minimum exit velocity $v_{\min}$ required to reach target $(d, h)$:
  $$v_0^4 - 2 g h v_0^2 - g^2 d^2 = 0 \implies v_{\min} = \sqrt{g(h + \sqrt{d^2 + h^2})}$$
- **Step 3 (Complex Roots / Unreachable Target)**: When $\Delta < 0$ (i.e. $v_0 < v_{\min}$), no real launch angle exists. The square root $\sqrt{\Delta}$ yields imaginary numbers. Passing $\Delta < 0$ to `Math.sqrt()` returns `Double.NaN`.
- **Conclusion for Discriminant Guard**:
  `TrajectorySolver.solveLaunchAngle` MUST check `if (Delta < 0.0)` BEFORE calling `Math.sqrt(Delta)`.
  When $\Delta < 0$:
  - `solveLaunchAngle` MUST return `OptionalDouble.empty()`.
  - `calculateTrajectory` MUST set `isUnreachable = true`, `isValid = false`, `flywheelAdjusted = true`, and set `targetFlywheelRPM` to the zone default RPM or $v_{\min}$ converted to RPM.

---

### 2.2 Derivation & Analysis of Apex Guard Condition ($d \tan(\theta) > h$)
Requirement R2.1 specifies $d \tan(\theta) > h$.

- **Step 1 (Geometric & Physical Meaning)**:
  The straight line along the launch angle vector has height $y_{\text{vector}}(d) = d \tan(\theta)$ at horizontal distance $d$. Gravity pulls the projectile down by $\frac{g d^2}{2 v_0^2 \cos^2(\theta)}$ during flight.
  Therefore:
  $$h = y(d) = d \tan(\theta) - \frac{g d^2}{2 v_0^2 \cos^2(\theta)}$$
  Rearranging gives:
  $$d \tan(\theta) - h = \frac{g d^2}{2 v_0^2 \cos^2(\theta)}$$
  Since $g > 0$, $d > 0$, $v_0 > 0$, and $\cos^2(\theta) > 0$, the right-hand side is strictly positive:
  $$d \tan(\theta) - h > 0 \iff d \tan(\theta) > h \iff \theta > \arctan(h/d)$$
- **Step 2 (Physical Guard Role)**:
  The condition $d \tan(\theta) > h$ guarantees that the launch angle points higher than the direct line-of-sight angle to the target ($\theta_{\text{LOS}} = \arctan(h/d)$).
  If a candidate angle $\theta$ satisfies $\tan(\theta) \le h/d$, the projectile can never reach height $h$ at distance $d$.
- **Conclusion for Apex Guard**:
  In `TrajectorySolver`, after computing $\theta = \arctan\left(\frac{v_0^2 - \sqrt{\Delta}}{g d}\right)$, verify `if (d * Math.tan(theta) <= h)`. If so, reject the angle (`isValid = false`).

---

### 2.3 Distance Bounds & Operational Envelope ($1.5\text{ m} \le d \le 7.0\text{ m}$)
- **Step 1 (Distance Limits)**:
  - $d < 1.5\text{ m}$: Robot is too close to hub. Shot requires extreme launch angles ($\theta > \theta_{\max}$) or low flywheel speed where motor control is erratic.
  - $d > 7.0\text{ m}$: Outside designated scoring zone boundary. Requires $v_0 > 9.15\text{ m/s}$ (RPM > 2866), increasing vision and drag errors.
- **Step 2 (Validation Strategy)**:
  `TrajectorySolver.calculateTrajectory(currentRPM, distance)` MUST evaluate:
  ```java
  if (Double.isNaN(distance) || Double.isInfinite(distance) || distance < 1.5 || distance > 7.0) {
      return new TrajectoryResult(STOWED_HOOD_ANGLE_RAD, DEFAULT_RPM, false, false, false);
  }
  ```

---

### 2.4 Mechanical Soft Limits $[\theta_{\min}, \theta_{\max}]$ & Flywheel Stability
The hood physical motor range is bounded by mechanical soft limits $[\theta_{\min}, \theta_{\max}]$.

- **Case 1: $\theta_1 \in [\theta_{\min}, \theta_{\max}]$**
  - Trajectory is achievable using the current flywheel velocity $v_0$.
  - Result: `desiredHoodAngleRad = theta_1`, `targetFlywheelRPM = currentFlywheelRPM`, `isValid = true`, `flywheelAdjusted = false`, `isUnreachable = false`.
  - **Inertia Preservation**: Flywheel RPM stays strictly constant during minor distance fluctuations ($d \pm \Delta d$) as long as $\theta_1$ stays within $[\theta_{\min}, \theta_{\max}]$.

- **Case 2: $\theta_1 < \theta_{\min}$ (Calculated angle is flatter than hood's minimum limit)**
  - Current flywheel speed $v_0$ is **too fast** for distance $d$ (shot would overshoot even at flat hood setting).
  - Result: `flywheelAdjusted = true`. The system transitions to the zone target RPM appropriate for distance $d$ (or reduces RPM until $\theta(v_0, d, h) \ge \theta_{\min}$).
  - Hood angle is set to $\theta_{\min}$ or re-solved at the new target RPM.

- **Case 3: $\theta_1 > \theta_{\max}$ (Calculated angle is steeper than hood's maximum limit)**
  - Current flywheel speed $v_0$ is **too slow** for distance $d$ (shot falls short even at maximum hood setting).
  - Result: `flywheelAdjusted = true`. System increases target RPM to zone target RPM or minimum required RPM.

---

### 2.5 `NaN` & Infinity Safety Guard Architecture
To prevent passing `NaN` or `Infinity` values to TalonFX motor position/velocity requests (which can cause driver errors or motor runaways):

1. **Input Pre-Validation**:
   - Reject `Double.isNaN(distance)`, `Double.isInfinite(distance)`, `distance <= 0`.
   - Reject `Double.isNaN(currentFlywheelRPM)`, `Double.isInfinite(currentFlywheelRPM)`, `currentFlywheelRPM <= 0`.
2. **Discriminant Pre-Check**:
   - Calculate $\Delta = v_0^4 - g(g d^2 + 2 h v_0^2)$.
   - If $\Delta < 0$, do **not** call `Math.sqrt(Delta)`. Immediately return safe fallback.
3. **Output Post-Validation**:
   - Check `Double.isNaN(calculatedAngle)` or `Double.isInfinite(calculatedAngle)`.
   - Check `Double.isNaN(calculatedRPM)` or `Double.isInfinite(calculatedRPM)`.
4. **Safe Fallbacks**:
   - Default Hood Angle: Stowed position $\theta_{\text{stowed}}$ (e.g. 20.0° / 0.349 rad) or optimal static angle $\frac{\pi}{4} + 0.5 \arctan(h/d)$.
   - Default Target RPM: 1000.0 RPM (`Constants.Shooter.kSHOOTER_FLYWHEEL_RPM`) or current RPM.
   - Status Flags: `isValid = false`, `isUnreachable = (Delta < 0 || v0 < vMin)`.

---

## 3. Caveats

1. **Read-Only Investigation**: All code snippets, test designs, and domain validation structures are proposed for implementation in Milestone 1 and Milestone 4. No project source files (`src/main/java` or `src/test/java`) were modified during this investigation.
2. **Mechanical Soft Limit Values**: Exact values for $[\theta_{\min}, \theta_{\max}]$ must be confirmed with hardware specs in `Constants.java` or `SUB_Hood.java` during implementation (e.g. $\theta_{\min} = 15.0^\circ \approx 0.2618\text{ rad}$, $\theta_{\max} = 70.0^\circ \approx 1.2217\text{ rad}$).
3. **Wheel Conversion Parameters**: Speed-to-RPM conversion relies on $D_{\text{wheel}} = 3.0\text{ in}$ ($0.0762\text{ m}$) and $C_{\text{comp}} = 0.8$. If physical wheel size or compression changes, conversion constants in `TrajectorySolver` must be updated.

---

## 4. Conclusion

### 4.1 Required Safety & Validation Rules in `TrajectorySolver`
To satisfy Requirements R2.1, R2.4, and R2.5, `TrajectorySolver.java` MUST incorporate the following validation sequence in `calculateTrajectory`:

```
Input (currentRPM, distance)
   │
   ├─► Pre-check: Is distance in [1.5m, 7.0m] and numbers finite/positive?
   │     NO  ──► Return TrajectoryResult(stowedAngle, defaultRPM, isValid=false, flywheelAdjusted=false, isUnreachable=false)
   │     YES ──► Continue
   │
   ├─► Convert currentRPM -> v0
   │
   ├─► Compute Discriminant Delta = v0^4 - g*(g*d^2 + 2*h*v0^2)
   │     Delta < 0 ──► Return TrajectoryResult(stowedAngle, zoneRPM, isValid=false, flywheelAdjusted=true, isUnreachable=true)
   │     Delta >= 0 ──► Compute theta1 = atan((v0^2 - sqrt(Delta)) / (g*d))
   │
   ├─► Check Apex Guard: Is d * tan(theta1) > h?
   │     NO  ──► Return TrajectoryResult(stowedAngle, zoneRPM, isValid=false, flywheelAdjusted=true, isUnreachable=true)
   │     YES ──► Continue
   │
   ├─► Check Hood Soft Limits: Is theta1 in [theta_min, theta_max]?
   │     YES ──► Return TrajectoryResult(theta1, currentRPM, isValid=true, flywheelAdjusted=false, isUnreachable=false)  [INERTIA HELD!]
   │     NO  ──► Return TrajectoryResult(theta1_clamped, zoneRPM, isValid=true, flywheelAdjusted=true, isUnreachable=false)  [ZONE ADJUSTMENT]
```

---

### 4.2 Comprehensive Unit Test Matrix (`PhysicsAndMathTest.java`)
The unit test suite MUST be expanded in `src/test/java/frc/robot/PhysicsAndMathTest.java` with the following test cases:

| Test Case Name | Input Conditions | Expected Outcome & Verification Assertions |
| :--- | :--- | :--- |
| `testSolveLaunchAngleAccuracy` | $d \in [1.5, 7.0]\text{ m}$, $v_0 = 8.0\text{ m/s}$ | `solveLaunchAngle` returns `OptionalDouble.of(theta)`. Assert $y(d) = d\tan\theta - \frac{gd^2}{2v_0^2\cos^2\theta} \approx h$ within $10^{-4}\text{ m}$. |
| `testUnreachableTargetComplexRoots` | $d = 6.0\text{ m}$, $v_0 = 3.0\text{ m/s}$ ($v_0 < v_{\min}$) | $\Delta < 0$. `solveLaunchAngle` returns `OptionalDouble.empty()`. `calculateTrajectory` returns `isValid = false`, `isUnreachable = true`, no `NaN` exception. |
| `testMinimumVelocityCalculation` | $d = 3.0\text{ m}$, $h = 1.397\text{ m}$ | `calculateMinimumVelocity` returns $\approx 6.7936\text{ m/s}$. Speed $v_{\min}-0.01$ returns `empty()`; $v_{\min}+0.01$ returns valid angle. |
| `testApexGuardCondition` | Solved angles for valid distances | Assert $d \tan(\theta) > h$ for all valid returned setpoints. |
| `testDistanceLimits` | $d = 1.4\text{ m}$, $d = 1.5\text{ m}$, $d = 7.0\text{ m}$, $d = 7.1\text{ m}$ | $d = 1.4\text{ m}$ and $d = 7.1\text{ m}$ yield `isValid = false`. $d = 1.5\text{ m}$ and $d = 7.0\text{ m}$ yield `isValid = true`. |
| `testNaNAndInfinitySafety` | $d \in \{\text{NaN}, \infty, -1.0, 0.0\}$, $v_0 \in \{\text{NaN}, 0.0, -5.0\}$ | Graceful fallback output; `isValid = false`; no `NaN` or `Infinity` in return record; zero unhandled exceptions. |
| `testFlywheelInertiaPreservation` | Initial RPM 2500, distance shifts $2.5\text{ m} \to 2.8\text{ m} \to 3.1\text{ m}$ | `targetFlywheelRPM` remains **exactly 2500 RPM** (`flywheelAdjusted = false`). `desiredHoodAngleRad` updates continuously. |
| `testFlywheelAdjustmentOutsideEnvelope` | Distance $6.5\text{ m}$ with initial low RPM 1200 | `flywheelAdjusted = true`. `targetFlywheelRPM` increases to zone setpoint. |

---

## 5. Verification Method

### 5.1 Automated Build and Test Verification
To verify domain math implementation and test suite:

1. **Compile Java Check**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat compileJava
   ```
   *Expectation*: `BUILD SUCCESSFUL` with 0 warnings/errors.

2. **Unit Test Suite Execution**:
   ```powershell
   $env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
   .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
   *Expectation*: `BUILD SUCCESSFUL`, all tests in `PhysicsAndMathTest.java` pass cleanly.

### 5.2 Files to Inspect
- `src/main/java/frc/robot/utils/TrajectorySolver.java` (to be created in M1)
- `src/test/java/frc/robot/PhysicsAndMathTest.java` (to be updated in M1/M4)
- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/subsystems/SUB_Hood.java`
- `src/main/java/frc/robot/subsystems/SUB_Shooter.java`

### 5.3 Invalidation Conditions
This analysis is invalidated if:
- Score target height $h$ ($55\text{ inches}$) or flywheel physical specs ($D = 3.0\text{ in}$, $C = 0.8$) are modified in `Constants.java`.
- Gravitational constant $g$ is altered.
