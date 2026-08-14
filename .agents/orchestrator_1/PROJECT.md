# Project: Robot Shooter and Aiming Refactor

## Architecture
The robot shooting system comprises a dual Kraken TalonFX flywheel (`SUB_Shooter`), a single TalonFX adjustable hood (`SUB_Hood`), indexer and metering subsystems, and vision-assisted aiming commands (`CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`). 

The refactored architecture introduces:
1. **Centralized Trajectory Solver (`TrajectorySolver.java`)**: A pure, testable physics utility that derives exact launch angle $\theta$ for a given fixed flywheel velocity $v_0$, distance $d$, and height differential $h$. Handles domain boundaries, complex roots, and `NaN`/`Infinity` fallback.
2. **Inertia-Preserving Subsystem Controllers**:
   - `SUB_Hood`: Converts degrees/radians to TalonFX motor position via gear ratio `kHOOD_GEAR_RATIO`, enforces physical soft limits $[\theta_{\min}, \theta_{\max}]$, and dynamically adjusts hood angle for distance perturbations.
   - `SUB_Shooter`: Manages stable flywheel setpoints using discrete distance zones with hysteresis ($\pm 0.3\text{m}$). Keeps RPM fixed while target distance moves within hood adjustment range, eliminating current spikes and flywheel velocity oscillation.
3. **Unified Subsystem Command Alignment**:
   - All aim commands (`CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`) and named auto routines (`CommandUtil`) declare requirements over all 5 shooter group subsystems (`drivetrain`, `shooter`, `hood`, `index`, `metering`).
   - Standardized Hub target translation calculation (`HubTargetUtil.java`) eliminates duplicated Tag 10/26 distance math.
   - Default commands in `RobotContainer` set `shooter` to idle (0 RPM) and `hood` to stowed safe position.
4. **Comprehensive Telemetry & Testing**:
   - Dashboard telemetry tracks desired hood angle, target RPM, flywheel stability, and ready-to-fire convergence status.
   - Unit test suite (`PhysicsAndMathTest.java`) validates solver accuracy across 1.5m to 7.0m, boundary handling, and flywheel stability.

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | TrajectorySolver Physics Utility | Closed-form fixed-velocity launch angle solver $\theta(v_0, d, h)$, RPM-to-$v_0$ conversions, $v_{\min}$ calculation | M1 | R2.2, R2.3 |
| 2 | Trajectory Domain Validation & Safety | Handle complex roots $\Delta < 0$, $d\tan\theta > h$ guard, $1.5\text{m}\le d\le 7.0\text{m}$ bounds, mechanical limits, `NaN`/`Infinity` fallback | M1 | R2.1, R2.4, R2.5 |
| 3 | Inertia-Preserving Hood Controller (`SUB_Hood`) | Hood gear ratio scaling, `getHoodAngleRadians()`, `setToAngle()`, soft limits $[\theta_{\min}, \theta_{\max}]$, telemetry | M2 | R1.1, R1.3, R5.1 |
| 4 | Inertia-Preserving Flywheel Controller (`SUB_Shooter`) | Zone-based RPM setpoints with hysteresis ($\pm 0.3\text{m}$), stable target holding, speed changes only on envelope violation | M2 | R1.2, R1.4, R5.2 |
| 5 | Centralized Hub & Distance Utility (`HubTargetUtil`) | Unified Tag 10/26 alliance offset and distance calculation | M3 | R3.4 |
| 6 | Command Subsystem Alignment & Firing Gate | Declare 5-subsystem requirements on `CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`; unified readiness gate (heading && RPM && hood) | M3 | R3.1, R3.2, R3.5 |
| 7 | Default Commands & Auto Registrations | Idle shooter default (0 RPM), stowed hood default in `RobotContainer`; update `CommandUtil` named auto commands | M3 | R3.3 |
| 8 | Dashboard Telemetry (R5) | SmartDashboard output for target hood angle, target RPM, flywheel stability, trajectory solvability, convergence status | M2, M3 | R5.1, R5.2, R5.3 |
| 9 | Comprehensive Unit Test Suite (R4) | JUnit 5 tests in `PhysicsAndMathTest.java` covering 1.5m–7.0m solver accuracy, complex roots, fallback, flywheel stability, build verification | M4 | R4.1, R4.2, R4.3, R4.4, R4.5 |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | TrajectorySolver & Domain Math | Create `frc.robot.utils.TrajectorySolver` with fixed-$v_0$ quadratic angle solver, domain validation, complex root & NaN handling | None | PLANNED |
| M2 | Subsystems Refactor (Shooter & Hood) | Refactor `SUB_Hood` with gear ratio & soft limits; refactor `SUB_Shooter` with zone setpoints & hysteresis; integrate telemetry | M1 | PLANNED |
| M3 | Commands & Alignment Refactor | Create `HubTargetUtil`; update `CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle` requirements & readiness gate; update `RobotContainer` & `CommandUtil` | M1, M2 | PLANNED |
| M4 | E2E & Unit Testing Track | Comprehensive JUnit 5 test suite expansion in `PhysicsAndMathTest.java`; verify `./gradlew.bat compileJava` and `./gradlew.bat test -x extractReleaseNative -x extractDebugNative` | M1, M2, M3 | PLANNED |

## Interface Contracts

### `TrajectorySolver` Interface (`frc.robot.utils.TrajectorySolver`)
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

### `SUB_Hood` Interface (`frc.robot.subsystems.SUB_Hood`)
```java
public class SUB_Hood extends SubsystemBase {
    public double getHoodAngleRadians();
    public double getHoodAngleDegrees();
    public void setToAngle(double angleRadians);
    public boolean atDesiredAngle();
    public boolean isLimitReached();
}
```

### `SUB_Shooter` Interface (`frc.robot.subsystems.SUB_Shooter`)
```java
public class SUB_Shooter extends SubsystemBase {
    public void setRPM(double targetRPM);
    public double getTargetRPM();
    public double flywheelRPM();
    public boolean atDesiredRPM();
    public double getZoneTargetRPM(double distanceMeters, double currentTargetRPM);
}
```

### `HubTargetUtil` Interface (`frc.robot.utils.HubTargetUtil`)
```java
public class HubTargetUtil {
    public static Pose2d getHubTargetPose(AprilTagFieldLayout layout);
    public static double getDistanceToHub(Pose2d robotPose, AprilTagFieldLayout layout);
}
```

## Code Layout
```
src/main/java/frc/robot/
├── Constants.java                              # Updated constants (Shooter zones, Hood gear ratio & limits)
├── subsystems/
│   ├── SUB_Shooter.java                        # Zone setpoints, hysteresis, stable RPM holding
│   └── SUB_Hood.java                           # Gear ratio scaling, physical angle PID, soft limits
├── commands/
│   ├── CMD_AimBot.java                         # 5-subsystem requirements, inertia-preserving control, readiness gate
│   ├── CMD_AimBotAuto.java                     # 5-subsystem requirements, hood-first controller
│   └── CMD_Shuttle.java                        # 5-subsystem requirements, hood-first shuttle control
├── utils/
│   ├── TrajectorySolver.java                   # Physics equations, quadratic angle solver, domain validation
│   ├── HubTargetUtil.java                      # Centralized Hub target pose & distance calculation
│   └── CommandUtil.java                        # Named auto commands updated with refactored shoot sequences
└── RobotContainer.java                         # Idle shooter default, stowed hood default, updated trigger bindings

src/test/java/frc/robot/
└── PhysicsAndMathTest.java                     # Comprehensive JUnit 5 test suite (1.5m to 7.0m, bounds, stability)
```
