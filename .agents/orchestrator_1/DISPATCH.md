## 2026-08-13T17:13:49-04:00
You are the Project Orchestrator for the robot shooter and aiming refactor task.
Original request file: C:\Users\Robotiators\Documents\GitHub\2026Offseason\ORIGINAL_REQUEST.md
Working directory: C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\orchestrator_1

Your task:
Decompose and orchestrate the implementation of all requirements in ORIGINAL_REQUEST.md:
1. R1: Inertia-Preserving / Hood-First Trajectory Controller across `SUB_Shooter`, `SUB_Hood`, `CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`, `RobotContainer`. Hold stable flywheel target RPM while adjusting hood angle for target distance; adjust flywheel velocity only when necessary or crossing major boundaries.
2. R2: Ballistic Trajectory & Domain Validation - solve launch angle $\theta$ given fixed $v$, distance $d$, height $h$, handling domain limits and `NaN`/infinity bounds gracefully.
3. R3: Autonomous & Teleoperated Command Alignment - default commands, aim commands, named auto commands (`CommandUtil`).
4. Unit Tests & Verification: Verify `./gradlew.bat compileJava` and `./gradlew.bat test -x extractReleaseNative -x extractDebugNative` succeed with comprehensive tests for hood angle trajectory solving across 1.5m to 7.0m, boundary handling, and flywheel stability.
5. Telemetry & Driver Control: SmartDashboard output for desired hood angle, target flywheel RPM, and convergence status.

Maintain your `BRIEFING.md` and `progress.md` in your working directory `.agents/orchestrator_1`. Report completion back to the Sentinel when all acceptance criteria are met and verified.
