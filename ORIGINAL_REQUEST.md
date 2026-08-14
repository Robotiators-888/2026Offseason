# Original User Request

## 2026-08-13T21:13:39Z

Refactor the robot shooter and aiming logic across subsystems and commands to prioritize hood angle adjustment over flywheel velocity modulation, minimizing flywheel speed changes due to flywheel inertia and battery energy draw, while ensuring robust mathematical trajectory validation.

Working directory: C:\Users\Robotiators\Documents\GitHub\2026Offseason
Integrity mode: development

## Requirements

### R1. Inertia-Preserving / Hood-First Trajectory Controller
Refactor the shooter and hood coordination logic (`SUB_Shooter`, `SUB_Hood`, `CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`, `RobotContainer`). When aiming/shooting:
1. Determine a stable flywheel target RPM for the scoring zone.
2. As the robot moves or distance fluctuates, maintain the current flywheel speed and adjust the adjustable hood angle to achieve the required launch angle for the new distance.
3. Only adjust the flywheel speed when the required trajectory cannot be achieved within the hood's physical angle limits or when transitioning across major distance boundaries.

### R2. Ballistic Trajectory & Domain Validation
Formulate and verify the projectile physics model for combined hood angle and flywheel speed:
1. Ensure the launch angle $\theta$ and horizontal distance $d$ to target height $h$ satisfy ballistic trajectory constraints (e.g., $d \tan(\theta) > h$ so the projectile ascends high enough to reach the target height, avoiding non-positive radicands in the velocity formula).
2. Given a fixed/current flywheel velocity $v$, solve for the required launch angle $\theta = \arctan\left(\frac{v^2 \pm \sqrt{v^4 - g(g d^2 + 2 h v^2)}}{g d}\right)$ (or numerical inversion) and set the hood angle accordingly.
3. Protect against `NaN`, infinite, or out-of-bounds mechanical values with fallback defaults.

### R3. Autonomous & Teleoperated Command Alignment
Ensure default commands (`shooter.setDefaultCommand`), aim commands (`CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`), and named autonomous commands (`CommandUtil`) adhere to this hood-dominant, speed-holding logic without unintended flywheel deceleration between shots or during minor robot repositioning.

## Acceptance Criteria

### Automated Build & Unit Tests
- [ ] `./gradlew.bat compileJava` succeeds with 0 errors and 0 warnings.
- [ ] `./gradlew.bat test -x extractReleaseNative -x extractDebugNative` passes comprehensive unit tests verifying:
  - Hood angle calculation accurately solves for launch angle at fixed flywheel velocities across distances from 1.5m to 7.0m.
  - Mathematical equations handle boundary conditions gracefully without `NaN` or negative square roots.
  - Flywheel setpoints remain stable when distance perturbations occur within the hood's adjustment envelope.

### Telemetry & Driver Control
- [ ] SmartDashboard accurately publishes desired hood angle, target flywheel RPM, and trajectory convergence status.
- [ ] Driver controls and autonomous routines seamlessly execute aiming without continuous flywheel speed oscillation.
