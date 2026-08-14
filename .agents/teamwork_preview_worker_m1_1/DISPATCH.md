## 2026-08-13T21:16:57Z
You are Worker 1 for Milestone 1 (TrajectorySolver & Domain Math).
Working Directory: C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_worker_m1_1

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

TASK:
Implement `src/main/java/frc/robot/utils/TrajectorySolver.java` and `src/test/java/frc/robot/PhysicsAndMathTest.java` according to specification.

INPUT ARTIFACTS TO READ:
1. `ORIGINAL_REQUEST.md` at C:\Users\Robotiators\Documents\GitHub\2026Offseason\ORIGINAL_REQUEST.md
2. `PROJECT.md` at C:\Users\Robotiators\Documents\GitHub\2026Offseason\PROJECT.md
3. `SCOPE.md` at C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\sub_orch_m1\SCOPE.md
4. Explorer reports:
   - `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_m1_1\handoff.md`
   - `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_m1_2\handoff.md`
   - `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_m1_3\handoff.md`

REQUIREMENTS:
1. Create `src/main/java/frc/robot/utils/TrajectorySolver.java` with package `frc.robot.utils` adhering to the precise interface contract:
   - `public record TrajectoryResult(double desiredHoodAngleRad, double targetFlywheelRPM, boolean isValid, boolean flywheelAdjusted, boolean isUnreachable)`
   - `public static double exitVelocityToRPM(double v0)`
   - `public static double rpmToExitVelocity(double rpm)`
   - `public static double calculateMinimumVelocity(double distanceMeters, double targetHeightMeters)`
   - `public static OptionalDouble solveLaunchAngle(double exitVelocityMetersPerSec, double distanceMeters, double targetHeightMeters)`
   - `public static TrajectoryResult calculateTrajectory(double currentFlywheelRPM, double distanceMeters)`
2. Ensure physics equations are implemented faithfully:
   - $v_0 = (\text{RPM} \times \pi \times D \times C_{\text{comp}}) / 60$ (where $D = 3.0\text{ in} = 0.0762\text{ m}$, $C_{\text{comp}} = 0.8$).
   - $\text{RPM} = (60 \times v_0) / (\pi \times D \times C_{\text{comp}})$.
   - $v_{\min} = \sqrt{g(h + \sqrt{d^2 + h^2})}$ with $g = 9.80665\text{ m/s}^2$ (from `Constants.Shooter.kGRAVITATIONAL_CONSTANT`) and $h = \text{Units.inchesToMeters}(55.0)$.
   - Quadratic angle solver: $\theta = \arctan\left(\frac{v_0^2 - \sqrt{v_0^4 - g(gd^2 + 2hv_0^2)}}{gd}\right)$.
   - Guard against complex roots ($\Delta = v_0^4 - g(gd^2 + 2hv_0^2) < 0$), $d\tan\theta \le h$, non-positive or out-of-bounds distance ($1.5\text{m} \le d \le 7.0\text{m}$), mechanical limits $[\theta_{\min}, \theta_{\max}]$, and return graceful fallbacks without throwing or producing NaN/Infinity.
3. Create/update `src/test/java/frc/robot/PhysicsAndMathTest.java` with JUnit 5 unit tests verifying:
   - Velocity conversions ($v_0 \leftrightarrow \text{RPM}$).
   - Minimum velocity calculation.
   - Closed-form launch angle solver across distances 1.5m to 7.0m.
   - Complex root & out-of-bounds domain validation (returning `OptionalDouble.empty()` and `isValid = false`).
   - Flywheel RPM stability when distance moves within hood angle adjustment envelope.

VERIFICATION COMMANDS (YOU MUST RUN THESE AND CAPTURE OUTPUT):
- `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava`
- `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative`

OUTPUT:
Write your completion report to `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_worker_m1_1\handoff.md`.
Then send a message back to parent with your summary and link to handoff.md.
