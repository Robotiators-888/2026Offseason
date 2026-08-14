# BRIEFING — 2026-08-13T21:14:48Z

## Mission
Investigate existing codebase for commands and command integration (CMD_AimBot, CMD_AimBotAuto, CMD_Shuttle, RobotContainer, CommandUtil), distance calculation/vision/pose estimation, auto command registration, gaps for R3 alignment, and recommendations for exact command structure and binding changes.

## 🔒 My Identity
- Archetype: Explorer / Investigator
- Roles: Commands & RobotContainer Focus Explorer
- Working directory: C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2
- Original parent: fe351549-52b3-42d5-9cf3-53ee61ae9ee2
- Milestone: Explorer Survey 2

## 🔒 Key Constraints
- Read-only investigation — do NOT implement code outside .agents directory
- Produce structured handoff report in C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2\handoff.md
- Send message back to parent when done

## Current Parent
- Conversation ID: fe351549-52b3-42d5-9cf3-53ee61ae9ee2
- Updated: 2026-08-13T21:14:48Z

## Investigation State
- **Explored paths**:
  - `src/main/java/frc/robot/commands/CMD_AimBot.java`
  - `src/main/java/frc/robot/commands/CMD_AimBotAuto.java`
  - `src/main/java/frc/robot/commands/CMD_Shuttle.java`
  - `src/main/java/frc/robot/RobotContainer.java`
  - `src/main/java/frc/robot/utils/CommandUtil.java`
  - `src/main/java/frc/robot/subsystems/SUB_Shooter.java`
  - `src/main/java/frc/robot/subsystems/SUB_Hood.java`
  - `src/main/java/frc/robot/subsystems/SUB_PhotonVision.java`
  - `src/main/java/frc/robot/utils/Hub.java`
  - `src/test/java/frc/robot/PhysicsAndMathTest.java`
- **Key findings**:
  - `CMD_AimBot` is missing `shooter` requirement; `CMD_AimBotAuto` & `CMD_Shuttle` are missing `hood` and `metering` requirements.
  - Shooter default command in `RobotContainer` continuously recalculates flywheel setpoint from distance while `CMD_AimBot` runs in teleop.
  - `CMD_AimBotAuto` does not command `hood`, leaving hood at angle 0 while changing flywheel RPM via `distanceToRPM` lookup map.
  - Teleop uses mathematical `findoptimalRPM`, while auto uses `distanceToRPM` lookup map.
  - Distance calculation to AprilTag 10/26 Hub center is duplicated across 5 files.
- **Unexplored areas**: None

## Key Decisions Made
- Completed comprehensive investigation and handoff report in `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2\handoff.md`.

## Artifact Index
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2\DISPATCH.md` — Initial dispatch prompt
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2\BRIEFING.md` — Working state briefing
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2\progress.md` — Liveness progress log
- `C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_2\handoff.md` — Final structured handoff report
