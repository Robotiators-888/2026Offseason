# Handoff Report — Worker 1 (Milestone 1: TrajectorySolver & Domain Math)

## 1. Observation

### 1.1 Source Files Created & Updated
1. **`src/main/java/frc/robot/utils/TrajectorySolver.java`**:
   - Created with package `frc.robot.utils`.
   - Record contract: `public record TrajectoryResult(double desiredHoodAngleRad, double targetFlywheelRPM, boolean isValid, boolean flywheelAdjusted, boolean isUnreachable)`.
   - Method implementations:
     - `public static double exitVelocityToRPM(double v0)`
     - `public static double rpmToExitVelocity(double rpm)`
     - `public static double calculateMinimumVelocity(double distanceMeters, double targetHeightMeters)`
     - `public static OptionalDouble solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters)`
     - `public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters)`

2. **`src/test/java/frc/robot/PhysicsAndMathTest.java`**:
   - Retained existing tests (`testOptimalAngleAndRPMCalculations`, `testAllianceFlipUtil`, `testAlertSystem`).
   - Added 6 new JUnit 5 unit tests:
     - `testVelocityConversions()`
     - `testMinimumVelocityCalculation()`
     - `testClosedFormLaunchAngleSolverAcrossDistances()`
     - `testComplexRootAndOutOfBoundsDomainValidation()`
     - `testFlywheelRPMStabilityWithinAdjustmentEnvelope()`
     - `testFlywheelAdjustmentWhenOutsideEnvelope()`

### 1.2 Verification Commands & Captured Tool Output

Command 1: Compile check
```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava
```
Output:
```
> Task :compileJava UP-TO-DATE
BUILD SUCCESSFUL in 2s
1 actionable task: 1 up-to-date
```

Command 2: Unit test execution
```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
```
Output:
```
> Task :compileJava UP-TO-DATE
> Task :processResources NO-SOURCE
> Task :classes UP-TO-DATE
> Task :compileTestJava
> Task :processTestResources NO-SOURCE
> Task :testClasses
HAL Extensions: No extensions found

> Task :test
PhysicsAndMathTest > testAllianceFlipUtil() PASSED
PhysicsAndMathTest > testFlywheelAdjustmentWhenOutsideEnvelope() PASSED
PhysicsAndMathTest > testAlertSystem() PASSED
PhysicsAndMathTest > testFlywheelRPMStabilityWithinAdjustmentEnvelope() PASSED
PhysicsAndMathTest > testMinimumVelocityCalculation() PASSED
PhysicsAndMathTest > testOptimalAngleAndRPMCalculations() PASSED
PhysicsAndMathTest > testClosedFormLaunchAngleSolverAcrossDistances() PASSED
PhysicsAndMathTest > testVelocityConversions() PASSED
PhysicsAndMathTest > testComplexRootAndOutOfBoundsDomainValidation() PASSED

BUILD SUCCESSFUL in 4s
3 actionable tasks: 2 executed, 1 up-to-date
```

---

## 2. Logic Chain

1. **Velocity Conversions ($v_0 \leftrightarrow \text{RPM}$)**:
   - $v_0 = \frac{\text{RPM} \cdot \pi \cdot D \cdot C_{\text{comp}}}{60}$ using wheel diameter $D = 3.0\text{ in} = 0.0762\text{ m}$ (`Constants.Shooter.ShooterDiameter`) and compression ratio $C_{\text{comp}} = 0.8$ (`Constants.Shooter.kSHOOTER_COMPRESSION_RATIO`).
   - Inverse: $\text{RPM} = \frac{60 \cdot v_0}{\pi \cdot D \cdot C_{\text{comp}}}$.
   - Guards return $0.0$ for non-positive, NaN, or infinite inputs.

2. **Minimum Launch Velocity Calculation**:
   - $v_{\min} = \sqrt{g(h + \sqrt{d^2 + h^2})}$ with $g = 9.80665\text{ m/s}^2$ (`Constants.Shooter.kGRAVITATIONAL_CONSTANT`) and $h = \text{targetHeightMeters}$.
   - Evaluates minimum physical speed to reach target height $h$ at distance $d$.

3. **Closed-Form Fixed-Velocity Launch Angle Solver (`solveLaunchAngle`)**:
   - Quadratic trajectory formula: $\theta = \arctan\left(\frac{v_0^2 - \sqrt{v_0^4 - g(gd^2 + 2hv_0^2)}}{gd}\right)$.
   - Discriminant check: $\Delta = v_0^4 - g(gd^2 + 2hv_0^2)$.
   - Returns `OptionalDouble.empty()` if $\Delta < 0$ (complex roots), $d\tan\theta \le h$ (apex guard), $v_0 \le 0$, $d \le 0$, or if result is NaN/Infinity.

4. **Trajectory Traversal & Inertia Preservation (`calculateTrajectory`)**:
   - Checks distance bounds: $1.5\text{ m} \le d \le 7.0\text{ m}$. Returns unreachable fallback record if out of bounds or NaN.
   - Attempts fixed-velocity launch angle solve using `currentFlywheelRPM`. If solved angle $\theta \in [\theta_{\min}, \theta_{\max}]$ ($[15^\circ, 75^\circ]$), returns `TrajectoryResult` with `targetFlywheelRPM = currentFlywheelRPM`, `flywheelAdjusted = false`, preserving flywheel inertia.
   - If current speed cannot hit target or falls outside hood limits, calculates required velocity $v_{\text{required}}$ at optimal angle $\theta_{\text{opt}} = \frac{\pi}{4} + 0.5 \arctan(h/d)$, adjusts flywheel speed (`flywheelAdjusted = true`), and re-solves launch angle.

---

## 3. Caveats

1. **Vacuum Kinematic Model**: Model assumes parabolic 2D kinematics. The effective compression ratio $C_{\text{comp}} = 0.8$ accounts for energy loss and surface interaction in practice.
2. **Hood Mechanical Limits**: Soft limits are set to $[15^\circ, 75^\circ]$ ($[0.2618\text{ rad}, 1.3090\text{ rad}]$). These match physical launcher capabilities and align with M2 subsystem requirements.

---

## 4. Conclusion

- `src/main/java/frc/robot/utils/TrajectorySolver.java` is fully implemented and tested according to specification.
- `src/test/java/frc/robot/PhysicsAndMathTest.java` contains 9 unit tests verifying all velocity conversions, minimum velocity calculations, closed-form solver accuracy across 1.5m–7.0m, complex root/out-of-bounds guards, and flywheel RPM stability.
- All verification commands pass with 0 errors and 0 warnings (`BUILD SUCCESSFUL`).

---

## 5. Verification Method

### 5.1 Verification Commands
Run in PowerShell:
```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative
```
Expected Output: `BUILD SUCCESSFUL` for both commands, with all 9 unit tests passing.

### 5.2 Files to Inspect
- `src/main/java/frc/robot/utils/TrajectorySolver.java`
- `src/test/java/frc/robot/PhysicsAndMathTest.java`

### 5.3 Invalidation Conditions
- Changes to `Constants.Shooter.ShooterDiameter`, `kSHOOTER_COMPRESSION_RATIO`, `kGRAVITATIONAL_CONSTANT`, or `Constants.Hood.ScoreHeight`.
