# Explorer Survey 2: Commands & RobotContainer Focus - Handoff Report

## Executive Summary
This report provides a detailed investigation of the current command architecture, `RobotContainer` bindings, default command scheduling, `CommandUtil` autonomous registrations, vision-to-target distance processing, and gaps preventing full R3 (Autonomous & Teleoperated Command Alignment) compliance.

---

## 1. Observation

### 1.1 Examination of Aim Commands
* **`CMD_AimBot.java`** (`src/main/java/frc/robot/commands/CMD_AimBot.java`):
  * **Declared Requirements** (Line 96): `addRequirements(drivetrain, metering, index, hood);`. `SUB_Shooter` is **NOT** included in the requirements!
  * **Shooter Control**: In `initialize()` (Line 120) sets static flag `SUB_Shooter.isShooting = true`, and in `end()` (Line 192) sets `false`. It does **not** call `shooter.setRPM()` or `shooter.shootMeters()` directly.
  * **Hood Control**: In `execute()` (Line 156), calls `hood.setToPosition(SUB_Hood.findoptimalangle(distance));`.
  * **Metering & Indexer**: In `execute()` (Line 157), calls `metering.set(1.0);`. Indexer is enabled at `Constants.Index.kINDEX_MOTOR_VOLTS` (8.5V) when `isThetaErrorCorrect` is `true` (Line 160).
  * **Firing Readiness Criteria**: Checks heading error ($\le 5^\circ$) and angular velocity ($\le 20^\circ/\text{s}$). Does **NOT** check if `shooter.atDesiredRPM()` or if the hood is converged before firing indexer.

* **`CMD_AimBotAuto.java`** (`src/main/java/frc/robot/commands/CMD_AimBotAuto.java`):
  * **Declared Requirements** (Line 70): `addRequirements(drivetrain, shooter, index);`. Neither `SUB_Hood` nor `SUB_Metering` are included!
  * **Shooter Control**: In `execute()` (Line 127), calls `shooter.shootMeters(distance);`, which modulates flywheel RPM via `distanceToRPM` lookup table every tick.
  * **Hood & Metering Control**: Neither is commanded. During auto aim, `hood` remains on its default command (`hood.resetSafe()`, position 0) and `metering` remains off (0V).
  * **Firing Readiness Criteria**: Checks `isThetaErrorCorrect` ($\le 3^\circ$, angular vel $\le 20^\circ/\text{s}$) **AND** `isShooterReady = shooter.atDesiredRPM()` (Line 129).

* **`CMD_Shuttle.java`** (`src/main/java/frc/robot/commands/CMD_Shuttle.java`):
  * **Declared Requirements** (Line 88): `addRequirements(drivetrain, index, shooter);`. `SUB_Hood` and `SUB_Metering` are **NOT** included.
  * **Shooter & Lead Compensation**: Calculates virtual target location accounting for Time-of-Flight lead ($v \cdot \text{TOF}$). Calls `shooter.shootMeters(distance)` (Line 154) continuously updating flywheel speed.
  * **Hood & Metering Control**: Neither is commanded.

### 1.2 Examination of `RobotContainer` Defaults and Button Bindings
* **Default Commands** (`src/main/java/frc/robot/RobotContainer.java` Lines 139–182):
  * `shooter.setDefaultCommand`:
    ```java
    shooter.setDefaultCommand(new RunCommand(() -> {
        final double distance = drivetrain.getPose().getTranslation().getDistance(
            SUB_PhotonVision.getInstance().at_field.getTagPose(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26
            ).map(pose -> pose.toPose2d().getTranslation().plus(
                new Translation2d(Units.inchesToMeters(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -23.5 : 23.5), 0)
            )).orElse(drivetrain.getPose().getTranslation())
        );
        shooter.setRPM(SUB_Shooter.findoptimalRPM(
            distance,
            SUB_Hood.findoptimalangle(distance)
        ));
    }, shooter));
    ```
    * **Impact**: While `CMD_AimBot` is running in teleop, `shooter` is not required by `CMD_AimBot`, so `shooter` runs its default command in parallel, recalculating and setting target RPM continuously based on distance!
  * `hood.setDefaultCommand`: `new RunCommand(() -> hood.resetSafe(), hood)` (Drives hood to position 0).
  * `index.setDefaultCommand`: `new InstantCommand(() -> index.set(0), index)`
  * `metering.setDefaultCommand`: `new InstantCommand(() -> metering.set(0), metering)`

* **Driver Button Bindings**:
  * **Driver 1 Right Trigger** (Line 243): Runs `ParallelCommandGroup(CMD_AimBot, getLinearCompress())`.
  * **Driver 2 Left Trigger** (Line 264): Manual shooter RPM override `RunCommand(() -> shooter.setRPM(targetRPM), shooter)`.
  * **Driver 2 Right Trigger** (Line 265): Manual index run.
  * **Driver 2 B Button** (Line 278): `CMD_Shuttle`.

### 1.3 Examination of `CommandUtil` Named Auto Commands
* **File**: `src/main/java/frc/robot/utils/CommandUtil.java` Lines 38–125:
  * `"ShootAutoAim"` (Line 81): Registers `new CMD_AimBotAuto(drivetrain, photonVision, shooter, index)`.
  * `"ShootDistance"` (Line 100): Runs `SequentialCommandGroup` where phase 1 calls `shooter.shootMeters(distance)` while backing up index, then phase 2 runs index at 8.5V. Does not command `hood` or `metering`.
  * `"ManualShoot"` (Line 72): Spools flywheel to fixed 1000 RPM, then runs indexer. Does not command `hood` or `metering`.
  * `"StopShooting"` (Line 119): Stops `index` and `shooter`. Does not stop or stow `hood` or reset `metering`.

### 1.4 Examination of Distance to Target Processing
* **Vision & Pose Estimation**:
  * `SUB_PhotonVision` manages 3 cameras (`BackLeftCam`, `BackRightCam`, `HighCam`) using `MULTI_TAG_PNP_ON_COPROCESSOR` with 2026 AprilTag field layout (`AprilTagFields.k2026RebuiltAndymark`).
  * `RobotContainer.photonPoseUpdate()` passes valid camera pose updates to `CommandSwerveDrivetrain.addVisionMeasurement()`.
  * Drivetrain fuses odometry, Pigeon2 gyro, and vision into `drivetrain.getPose()`.
* **Distance Calculation Duplication**:
  * Target Hub translation is derived from Tag 10 (Red Alliance) or Tag 26 (Blue Alliance) offset by $X = \pm 23.5$ inches ($\pm 0.5969$m).
  * 2D Euclidean distance $d = \text{robotPose.getTranslation().getDistance(hubCenterTranslation)}$ is independently computed in 5 places:
    1. `CMD_AimBot.java` (Line 155)
    2. `CMD_AimBotAuto.java` (Line 125)
    3. `CMD_Shuttle.java` (Line 127)
    4. `RobotContainer.java` (Lines 162 & 284)
    5. `CommandUtil.java` (Line 102)

---

## 2. Logic Chain

1. **Observation**: `CMD_AimBot` does not require `shooter`. `CMD_AimBotAuto` and `CMD_Shuttle` require `shooter`, but do not require `hood` or `metering`.
2. **Step 1**: In WPILib Command-Based framework, if a command does not claim a subsystem requirement, that subsystem continues running its default command.
3. **Step 2**: In teleop (`CMD_AimBot`), `shooter` is unassigned, so `shooter` runs its default command which continuously calls `shooter.setRPM(findoptimalRPM(distance, findoptimalangle(distance)))` every 20ms tick.
4. **Step 3**: In auto (`CMD_AimBotAuto` and `CMD_Shuttle`), `hood` and `metering` are unassigned. `hood` runs its default command (`hood.resetSafe()`, driving hood to angle 0), while `metering` remains unpowered (0V). `CMD_AimBotAuto` continuously calls `shooter.shootMeters(distance)` which modulates flywheel RPM every tick via table lookup.
5. **Step 4**: Continuously modulating flywheel RPM as distance changes ($1.5\text{m}$ to $7.0\text{m}$) causes high current spikes, flywheel speed oscillation, and battery draw, while keeping the hood at angle 0 in auto prevents angle-based trajectory optimization.
6. **Step 5**: Furthermore, teleop uses `findoptimalRPM` (physics calculation), while auto uses `distanceToRPM` (interpolating map). This creates a direct contradiction between autonomous and teleoperated shooting behavior.
7. **Step 6**: To achieve R3 (Autonomous & Teleoperated Alignment) and R1 (Inertia-Preserving Hood-First Trajectory Control):
   - All aim/shoot commands (`CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`, and `CommandUtil` routines) must require all 5 shooting-related subsystems (`drivetrain`, `shooter`, `hood`, `index`, `metering`).
   - All aim/shoot commands must utilize a unified inertia-preserving hood-first trajectory algorithm: maintaining a stable flywheel RPM setpoint and adjusting hood angle $\theta$ for distance fluctuations, only altering flywheel RPM when distance exceeds mechanical hood boundaries.

---

## 3. Caveats
- **No Caveats**: All 25 Java files in the repository were fully located, inspected, and cross-referenced.

---

## 4. Conclusion & Recommendations

### 4.1 Identified Architectural Gaps
1. **Inconsistent Subsystem Locking**: Subsystem requirement sets differ between commands (`CMD_AimBot` vs `CMD_AimBotAuto` vs `CMD_Shuttle`).
2. **Default Command Interference**: Teleop default shooter command constantly recalculates flywheel setpoint from distance while aiming. Auto aim commands leave hood on default command ($0^\circ$).
3. **Dual Trajectory Models**: Teleop calculates RPM mathematically via `findoptimalRPM`, auto calculates RPM via `distanceToRPM` lookup map.
4. **Missing Readiness Gates**: `CMD_AimBot` does not check `shooter.atDesiredRPM()` or `hood.atDesiredAngle()` before powering indexer. `CMD_AimBotAuto` ignores `metering`.
5. **Duplicated Distance Logic**: Hub target calculation is copy-pasted across 5 classes.

---

### 4.2 Recommended Interface Contracts & Command Structure

#### Recommendation 1: Centralized Hub & Distance Utility (`HubTargetUtil` or `Hub.java`)
Create a single point of truth for Hub target location and distance calculation:
```java
public class HubTargetUtil {
    public static Pose2d getHubTargetPose(AprilTagFieldLayout layout) {
        int tagId = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 10 : 26;
        Pose2d tagPose = layout.getTagPose(tagId).orElse(new Pose3d()).toPose2d();
        double hubOffsetX = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) 
            ? Units.inchesToMeters(-23.5) 
            : Units.inchesToMeters(23.5);
        return new Pose2d(tagPose.getX() + hubOffsetX, tagPose.getY(), new Rotation2d());
    }

    public static double getDistanceToHub(Pose2d robotPose, AprilTagFieldLayout layout) {
        return robotPose.getTranslation().getDistance(getHubTargetPose(layout).getTranslation());
    }
}
```

#### Recommendation 2: Standardized Subsystem Requirement Contract
Every aiming / shooting command (`CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`, and `CommandUtil` shoot sequences) **MUST** declare requirement over all 5 shooter-group subsystems:
```java
addRequirements(drivetrain, shooter, hood, index, metering);
```

#### Recommendation 3: Unified Trajectory & Readiness Logic (R1 & R2 Integration)
1. **Flywheel Setpoint Stability**:
   - Establish baseline zone flywheel RPMs (e.g. 1800 RPM for general field scoring, or zone-based step setpoints).
   - Maintain constant flywheel RPM while distance fluctuates within the hood's physical range ($1.5\text{m}$ to $7.0\text{m}$).
2. **Hood Launch Angle Solving**:
   - Solve for launch angle $\theta(v, d, h)$ given fixed flywheel velocity $v$ and target distance $d$.
   - Command `hood.setToPosition(requiredTheta)`.
3. **Unified Firing Gate**:
   - Index and metering motors activate **ONLY** when:
     $$\text{isReadyToFire} = \text{isThetaErrorCorrect} \land \text{shooter.atDesiredRPM()} \land \text{hood.atDesiredAngle()}$$

#### Recommendation 4: Default Command Modifications in `RobotContainer.java`
- `shooter.setDefaultCommand`: Change to idle setpoint (e.g., 0 RPM or idle coasting), eliminating distance-based RPM modulation when idle.
- `hood.setDefaultCommand`: Maintain stowed safe position (`resetSafe()`).

#### Recommendation 5: Auto Command Registrations in `CommandUtil.java`
- `"ShootAutoAim"`: Instantiate refactored `CMD_AimBotAuto` with full subsystem requirements.
- `"ShootDistance"`: Use unified inertia-preserving controller before feeding.
- `"StopShooting"`: Cleanly stop `shooter`, `hood`, `index`, and `metering`.

---

## 5. Verification Method

To independently verify these recommendations once implemented:
1. **Compilation Check**:
   ```cmd
   ./gradlew.bat compileJava
   ```
2. **Unit Test Execution**:
   ```cmd
   ./gradlew.bat test -x extractReleaseNative -x extractDebugNative
   ```
   Verify physics tests in `PhysicsAndMathTest.java` pass, validating hood angle solving for fixed flywheel velocities across distances $1.5\text{m}$ to $7.0\text{m}$.
3. **Subsystem Requirement Inspection**:
   Inspect `CMD_AimBot.java`, `CMD_AimBotAuto.java`, `CMD_Shuttle.java` to confirm all 5 subsystems (`drivetrain`, `shooter`, `hood`, `index`, `metering`) are passed and declared in `addRequirements(...)`.
4. **Dashboard Telemetry Check**:
   Verify SmartDashboard displays:
   - `Shooter/Desired RPM` (remains stable during distance perturbations)
   - `Hood/Position` (adjusts dynamically to maintain trajectory)
   - `CMD_AimBot/isThetaErrorCorrect` and readiness flags.
