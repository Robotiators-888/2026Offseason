# Scope: Milestone 1 — TrajectorySolver & Domain Math

## Objective
Implement `frc.robot.utils.TrajectorySolver` with closed-form fixed-velocity launch angle solver, domain validation, complex root handling, and graceful NaN/Infinity fallback.

## Assigned Features (from PROJECT.md)
1. **Feature 1**: TrajectorySolver Physics Utility (R2.2, R2.3)
   - Closed-form solver: $\theta = \arctan\left(\frac{v_0^2 - \sqrt{v_0^4 - g(gd^2 + 2hv_0^2)}}{gd}\right)$
   - Velocity conversions: $v_0 \leftrightarrow \text{RPM}$ using wheel diameter 3.0 in, compression ratio 0.8.
   - Minimum velocity calculation: $v_{\min} = \sqrt{g(h + \sqrt{d^2+h^2})}$.
2. **Feature 2**: Trajectory Domain Validation & Safety (R2.1, R2.4, R2.5)
   - Discriminant check $\Delta = v_0^4 - g(gd^2 + 2hv_0^2) < 0 \implies$ invalid target / insufficient speed.
   - Guard condition $d\tan\theta > h$.
   - Bounds check $1.5\text{m} \le d \le 7.0\text{m}$ and hood angle limits $[\theta_{\min}, \theta_{\max}]$.
   - Fallback protection against `NaN`/`Infinity` returning home position / safe default and `isValid = false`.

## Interface Contract
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

## Relevant References
- `PROJECT.md`
- `ORIGINAL_REQUEST.md`
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_3\handoff.md`
