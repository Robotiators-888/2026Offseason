# BRIEFING — 2026-08-13T17:14:47-04:00

## Mission
Investigate SUB_Shooter and SUB_Hood in the 2026Offseason codebase to support R1 (Inertia-Preserving / Hood-First Trajectory Controller) and R5 (Telemetry).

## 🔒 My Identity
- Archetype: Explorer Survey 1
- Roles: Subsystems Focus Explorer
- Working directory: C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1
- Original parent: fe351549-52b3-42d5-9cf3-53ee61ae9ee2
- Milestone: Explorer Survey

## 🔒 Key Constraints
- Read-only investigation — do NOT edit source code files
- Deliver findings in handoff.md following 5-component handoff report structure
- Update progress.md upon completion

## Current Parent
- Conversation ID: fe351549-52b3-42d5-9cf3-53ee61ae9ee2
- Updated: 2026-08-13T17:14:47-04:00

## Investigation State
- **Explored paths**:
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
- **Key findings**:
  - `SUB_Shooter` currently uses `shootMeters(distance)` via lookup table `distanceToRPM`, modulating RPM continuously every tick on small distance changes.
  - `SUB_Hood` position controller receives target in radians but reads raw TalonFX motor rotations without gear ratio conversion or physical limits.
  - Commands (`CMD_AimBot`, `CMD_AimBotAuto`, `CMD_Shuttle`, `CommandUtil`) do not coordinate shooter speed and hood angle together.
  - R1 & R5 requirements mapped to specific interface additions in `SUB_Shooter` and `SUB_Hood`, zone setpoints with hysteresis, quadratic launch angle solver given fixed RPM, and telemetry keys.
- **Unexplored areas**: None (Subsystems investigation completed).

## Key Decisions Made
- Completed thorough codebase investigation and delivered 5-component handoff report to `handoff.md`.

## Artifact Index
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1\DISPATCH.md` — Dispatch prompt record
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1\BRIEFING.md` — Persistent briefing state
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1\progress.md` — Progress log
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1\handoff.md` — Handoff report with findings and recommendations
