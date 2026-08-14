# Handoff Report: Subsystems Focus (SUB_Shooter & SUB_Hood)

**Author:** Explorer Survey 1 (Subsystems Focus)  
**Date:** 2026-08-13  
**Working Directory:** `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1`  
**Objective:** Complete subsystems survey of `SUB_Shooter` and `SUB_Hood` to support Requirements R1 (Inertia-Preserving / Hood-First Trajectory Controller) and R5 (Telemetry).

---

## 1. Observation

### 1.1 Source Files Examined
The investigation examined the following project files:
- `src/main/java/frc/robot/subsystems/SUB_Shooter.java`
- `src/main/java/frc/robot/subsystems/SUB_Hood.java`
- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/commands/CMD_AimBot.java`
- `src/main/java/frc/robot/commands/CMD_AimBotAuto.java`
- `src/main/java/frc/robot/commands/CMD_Shuttle.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/utils/CommandUtil.java`
- `src/main/java/frc/robot/utils/Hub.java`
- `src/main/java/frc/robot/utils/RobotTelemetry.java`
- `src/test/java/frc/robot/PhysicsAndMathTest.java`

---

### 1.2 Current Subsystem Architecture: `SUB_Shooter.java`

#### Hardware Configuration & Actuators
- **Flywheel Motors:** Dual CTRE TalonFX motors:
  - `MotorOne` (CAN ID 43, `Constants.Shooter.kSHOOTER_MotorOne_MOTOR_CANID`)
  - `MotorTwo` (CAN ID 44, `Constants.Shooter.kSHOOTER_MotorTwo_MOTOR_CANID`)
  - `MotorTwo` is configured as a `Follower` of `MotorOne` aligned in direction (`MotorAlignmentValue.Aligned`).
- **Control Modes:**
  - Closed-loop velocity control via CTRE Phoenix 6 `VelocityVoltage` (`m_request`).
  - Open-loop voltage control via `VoltageOut` (`voltageRequest`).
- **Dual Current Limit Profiles:**
  - `shooterConfig`: Applied during active shooting (`isShooting == true`). Stator limit = 100A; Supply limit = 60A (Lower limit = 40A after 1.0s). Neutral mode = `Coast`.
  - `shooterLowConfig`: Applied during idle state (`isShooting == false`). Stator limit = 100A; Supply limit = 10A (Lower limit = 5A after 1.0s). Neutral mode = `Coast`.
  - Configuration switching occurs in `periodic()` on state transitions of the static flag `SUB_Shooter.isShooting`.

#### Velocity & Physics Control Methods
- **Lookup Table (`distanceToRPM`):** `InterpolatingDoubleTreeMap` mapping distance (meters) to target RPM:
  ```java
  distanceToRPM.put(1.6346195276, 1050.0);
  distanceToRPM.put(2.49493587092, 1250.0);
  distanceToRPM.put(3.03308176613, 1390.0);
  distanceToRPM.put(4.10526503, 1600.0);
  distanceToRPM.put(5.34766117, 1790.0);
  distanceToRPM.put(10.5, 2400.0);
  ```
- **`shootMeters(double meters)`:** Calls `distanceToRPM.get(meters)` and sets `setRPM(targetRPM)`.
- **`findoptimalRPM(double distance, double angleRadians)`:** Static trajectory calculation:
  ```java
  double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);
  double denom = 2.0 * (distance * Math.tan(angleRadians) - height);
  // Vacuum projectile model: exitVelocity = (1 / cos(theta)) * sqrt(g * d^2 / denom)
  double exitVelocity = (1.0 / cosAngle) * Math.sqrt((Constants.Shooter.kGRAVITATIONAL_CONSTANT * distance * distance) / denom);
  double surfaceSpeed = exitVelocity / Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
  double rps = surfaceSpeed / (Math.PI * wheelDiameterMeters);
  double exitRPM = rps * 60.0;
  ```
  Returns `Constants.Shooter.kSHOOTER_FLYWHEEL_RPM` (1000 RPM default) if `denom <= 0.001`, `distance <= 0.1`, `cosAngle < 0.001`, or if result is `NaN`/`Infinite`.
- **`atDesiredRPM()`:** Checks `Math.abs(flywheelRPM() - desiredSpeed) < 75`.

---

### 1.3 Current Subsystem Architecture: `SUB_Hood.java`

#### Hardware Configuration & Actuators
- **Hood Motor:** Single CTRE TalonFX motor (`hood`, CAN ID 47, `Constants.Hood.kHOOD_CAN_ID`).
- **Current Limits:** Stator limit = 25A; Supply limit = 7A (Lower limit = 5A after 0.5s).

#### Angle Control & Positioning Methods
- **`setToPosition(double angle)`:**
  ```java
  hood.set(Constants.Hood.kHOOD_PID_CONTROLLER.calculate(hood.getPosition().getValueAsDouble(), angle));
  ```
  *Key Observation:* `hood.getPosition().getValueAsDouble()` returns TalonFX motor/gearbox rotations, whereas `angle` passed from commands (`SUB_Hood.findoptimalangle(distance)`) is in radians. There is currently **no gear ratio conversion** or unit transformation between physical hood angle (radians/degrees) and motor rotations.
- **`findoptimalangle(double distance)`:** Empirical launcher angle formula:
  ```java
  double height = Units.inchesToMeters(Constants.Hood.ScoreHeight); // 55 inches = 1.397 m
  return (Math.PI / 4.0) + 0.5 * Math.atan2(height, distance);
  ```
- **Physical & Software Limits:** No software soft limits (`SoftwareLimitSwitchConfigs`) or position clamps are currently implemented in `SUB_Hood.java`.

---

### 1.4 Command Cross-Subsystem Usage Map

| File | Flywheel Control | Hood Control | Issues Identified |
|---|---|---|---|
| `RobotContainer.java` (Default Commands) | Default command runs `shooter.setRPM(findoptimalRPM(dist, findoptimalangle(dist)))` | Default command runs `hood.resetSafe()` (sets position to 0) | Shooter default continuously modulates RPM while hood default resets to zero. |
| `CMD_AimBot.java` | Does NOT call `shooter.setRPM()` or `shootMeters()` in `execute()` | Calls `hood.setToPosition(SUB_Hood.findoptimalangle(distance))` | Hood target set in radians directly into raw rotation PID controller. |
| `CMD_AimBotAuto.java` | Calls `shooter.shootMeters(distance)` every tick | Does NOT reference `SUB_Hood` | Hood stays in default zero position during auto aiming. |
| `CMD_Shuttle.java` | Calls `shooter.shootMeters(distance)` every tick | Does NOT reference `SUB_Hood` | Hood ignored during shuttle maneuvers. |
| `CommandUtil.java` (`ShootDistance`) | Calls `shooter.shootMeters(distance)` | Does NOT reference `SUB_Hood` | Named auto command ignores hood adjustment. |

---

### 1.5 Existing Telemetry Map

#### `SUB_Shooter.java` Periodic Telemetry
Publishes 19 entries under `SmartDashboard`:
- `Shooter/Desired RPM`
- `Shooter/Motor One Stator Current`, `Shooter/Motor Two Stator Current`
- `Shooter/Motor One Supply Current`, `Shooter/Motor Two Supply Current`
- `Shooter/Motor One Supply Voltage`, `Shooter/Motor Two Supply Voltage`
- `Shooter/Motor One Voltage`, `Shooter/Motor Two Voltage`
- `Shooter/Motor One Encoder Pos`, `Shooter/Motor Two Encoder Pos`
- `Shooter/Motor One Torque Current`, `Shooter/Motor Two Torque Current`
- `Shooter/Motor One Device Temp`, `Shooter/Motor Two Device Temp`
- `Shooter/Motor One Processor Temp`, `Shooter/Motor Two Processor Temp`
- `Shooter/FlywheelRPM (One)`, `Shooter/FlywheelRPM (Two)`, `Shooter/FlywheelRPM (Average)`

#### `SUB_Hood.java` Periodic Telemetry
Publishes 9 entries under `SmartDashboard`:
- `Hood/Position`
- `Hood/Stator Current`, `Hood/Supply Current`, `Hood/Supply Voltage`, `Hood/Motor Voltage`
- `Hood/Torque Current`, `Hood/Device Temp`, `Hood/Processor Temp`, `Hood/Velocity`

---

## 2. Logic Chain

### 2.1 The Flywheel Velocity Oscillation Problem
1. Currently, `shootMeters(distance)` evaluates `distanceToRPM.get(distance)` every control tick (20 ms).
2. As the robot moves, vision noise or drivetrain movement causes `distance` to fluctuate by $\pm 0.05\text{ m}$ to $\pm 0.30\text{ m}$.
3. Because the lookup table interpolates linearly, target flywheel RPM changes continuously (e.g., oscillating between 1400 RPM and 1550 RPM).
4. Accelerating dual Kraken 6000 flywheels requires high instantaneous torque current (up to 100A stator limit), drawing significant energy from the main battery and causing system voltage dips.
5. Furthermore, `atDesiredRPM()` returns `false` while the flywheels ramp up/down, delaying indexer feed and slowing overall shot rate.

### 2.2 Why Hood-First Trajectory Control Solves The Problem
1. The hood mechanism has a low moment of inertia ($I_{\text{hood}} \ll I_{\text{flywheel}}$) and can adjust angle rapidly (in tens of milliseconds) with low current draw (stator limit 25A, supply limit 7A).
2. By keeping the flywheel speed $v$ fixed at a stable target RPM for a broad distance zone, distance fluctuations can be absorbed entirely by adjusting launch angle $\theta$.
3. Ballistic physics allows solving for launch angle $\theta$ given distance $d$, target height $h$, and fixed exit velocity $v$:
   $$h = d \tan\theta - \frac{g d^2}{2 v^2 \cos^2\theta}$$
   Using $\frac{1}{\cos^2\theta} = 1 + \tan^2\theta$, this yields the quadratic equation in $\tan\theta$:
   $$\left(\frac{g d^2}{2 v^2}\right) \tan^2\theta - d \tan\theta + \left(h + \frac{g d^2}{2 v^2}\right) = 0$$
   The discriminant is:
   $$\Delta = d^2 - 4 \left(\frac{g d^2}{2 v^2}\right)\left(h + \frac{g d^2}{2 v^2}\right) = \frac{d^2}{v^4} \left( v^4 - g(g d^2 + 2 h v^2) \right)$$
   When $\Delta \ge 0$, valid launch angles exist:
   $$\theta = \arctan\left(\frac{v^2 \pm \sqrt{v^4 - g(g d^2 + 2 h v^2)}}{g d}\right)$$
4. Flywheel speed needs to change ONLY when:
   - $\Delta < 0$ (flywheel speed too slow to physically reach target height $h$ at distance $d$).
   - The required launch angle $\theta$ falls outside the mechanical limits $[\theta_{\min}, \theta_{\max}]$ of the hood.
   - The robot crosses a major distance boundary between designated zones (e.g., Subwoofer/Close, Mid-range, Long-range).

---

## 3. Caveats

1. **Read-Only Investigation:** No source files under `src/` were edited during this survey.
2. **Hood Gear Ratio Measurement Needed:** Existing code calls `hood.getPosition().getValueAsDouble()` directly without unit scaling. Implementation will require measuring the exact physical gear reduction from the TalonFX motor shaft to the hood pivot (rotations per radian or rotations per degree).
3. **Aerodynamic Drag & Compression Model:** The theoretical trajectory formula assumes vacuum ballistics and constant compression ratio ($0.8$). Empirical tuning or a drag compensation scalar $k_{\text{drag}}$ may be needed during physical robot testing.

---

## 4. Conclusion & Recommendations

### 4.1 Recommended Interface Changes for `SUB_Hood.java`

1. **Unit Conversion & Angle Scaling:**
   - Define conversion constants:
     - `kHOOD_GEAR_RATIO`: Motor rotations per radian of physical launch angle.
     - `kHOOD_MIN_ANGLE_RAD` & `kHOOD_MAX_ANGLE_RAD`: Physical mechanical limits (e.g., $20^\circ \approx 0.349\text{ rad}$ to $70^\circ \approx 1.222\text{ rad}$).
   - `getHoodAngleRadians()`: `return getPosition() / kHOOD_GEAR_RATIO;`
   - `setToAngle(double angleRadians)`: Clamps target angle to $[\theta_{\min}, \theta_{\max}]$, converts to motor rotations, and executes closed-loop positioning.

2. **Ballistic Angle Calculator:**
   - `double calculateLaunchAngle(double distanceMeters, double flywheelRPM)`: Given fixed flywheel RPM, computes exit velocity $v$, checks discriminant $\Delta = v^4 - g(g d^2 + 2 h v^2)$, and returns lower-trajectory solution $\theta = \arctan\left(\frac{v^2 - \sqrt{\Delta}}{g d}\right)$.
   - `boolean isTrajectoryValid(double distanceMeters, double flywheelRPM)`: Returns `true` if $\Delta \ge 0$ AND calculated angle $\theta \in [\theta_{\min}, \theta_{\max}]$.

---

### 4.2 Recommended Interface Changes for `SUB_Shooter.java`

1. **Inertia-Preserving Zone-Based Target Manager:**
   - Define discrete flywheel setpoint zones with hysteresis:
     - **Zone 1 (Close / Subwoofer, 1.0m – 2.8m):** Target RPM = 1200 RPM.
     - **Zone 2 (Mid-Range, 2.5m – 4.8m):** Target RPM = 1650 RPM.
     - **Zone 3 (Long-Range / Shuttle, 4.5m – 8.5m):** Target RPM = 2250 RPM.
   - `double selectZoneTargetRPM(double distanceMeters, double currentTargetRPM)`: Implements a hysteresis envelope ($\pm 0.3\text{ m}$) so minor robot movements near zone boundaries do not cause RPM oscillation.
   - `rpmToExitVelocity(double rpm)` & `exitVelocityToRPM(double v)`: Standard static utility methods for projectile velocity mapping.

2. **Coordinated Trajectory Query Method:**
   - `double getInertiaPreservingTargetRPM(double distanceMeters)`: Checks if current flywheel speed can achieve valid trajectory via `SUB_Hood.isTrajectoryValid(distanceMeters, currentTargetRPM)`. If valid, returns `currentTargetRPM`; if invalid or boundary crossed, returns new zone setpoint.

---

### 4.3 Summary of Required Constants (`Constants.java`)

```java
public static final class Shooter {
    // Distance Zone Setpoints (RPM)
    public static final double kZONE1_RPM = 1200.0;
    public static final double kZONE2_RPM = 1650.0;
    public static final double kZONE3_RPM = 2250.0;
    public static final double kZONE_HYSTERESIS_METERS = 0.30;
    public static final double kSHOOTER_TOLERANCE_RPM = 50.0;
}

public static class Hood {
    public static final double kHOOD_GEAR_RATIO = 25.0; // Example: 25:1 reduction
    public static final double kHOOD_MIN_ANGLE_RAD = Units.degreesToRadians(20.0);
    public static final double kHOOD_MAX_ANGLE_RAD = Units.degreesToRadians(70.0);
    public static final double kHOOD_DEFAULT_ANGLE_RAD = Units.degreesToRadians(45.0);
}
```

---

### 4.4 Telemetry Requirements (R5 Compliance)

To satisfy R5 Telemetry and SmartDashboard reporting requirements, the following keys must be published during subsystem `periodic()` calls:

| Dashboard Key | Type | Description |
|---|---|---|
| `Shooter/Target RPM` | double | Currently requested flywheel target RPM |
| `Shooter/Current RPM Average` | double | Measured average RPM from dual TalonFX encoders |
| `Shooter/RPM Stable` | boolean | `true` when `Math.abs(flywheelRPM - targetRPM) <= kSHOOTER_TOLERANCE_RPM` |
| `Shooter/Active Zone` | String | Active distance zone (e.g. `"ZONE_1"`, `"ZONE_2"`, `"ZONE_3"`) |
| `Hood/Target Angle (Deg)` | double | Desired hood launch angle in degrees |
| `Hood/Current Angle (Deg)` | double | Measured hood launch angle in degrees |
| `Hood/At Target Angle` | boolean | `true` when hood is within angle tolerance |
| `Hood/Angle Limit Reached` | boolean | `true` if target angle is clamped to physical bounds |
| `Trajectory/Distance (m)` | double | Current measured distance to target |
| `Trajectory/Solvable` | boolean | `true` if $\Delta \ge 0$ for current speed & distance |
| `Trajectory/Convergence Status` | boolean | Master ready-to-shoot flag: heading aligned && RPM stable && hood ready |

---

## 5. Verification Method

To independently verify the implementation following refactor:

1. **Compilation Check:**
   Execute build command:
   ```cmd
   ./gradlew.bat compileJava
   ```
   *Expected Output:* BUILD SUCCESSFUL with 0 errors and 0 warnings.

2. **Automated Unit Tests:**
   Execute unit test command:
   ```cmd
   ./gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
   *Verification Criteria:*
   - Verify `PhysicsAndMathTest.java` passes across distance test range $1.5\text{ m}$ to $7.0\text{ m}$.
   - Test hood launch angle equation solver for fixed RPM setpoints without encountering `NaN` or infinite values.
   - Verify flywheel setpoint stability when distance perturbations of $\pm 0.2\text{ m}$ occur within hood adjustment limits.

3. **Telemetry & Dashboard Verification:**
   Inspect SmartDashboard / Elastic layout for expected keys: `Shooter/Target RPM`, `Hood/Target Angle (Deg)`, `Trajectory/Solvable`, and `Trajectory/Convergence Status`.
