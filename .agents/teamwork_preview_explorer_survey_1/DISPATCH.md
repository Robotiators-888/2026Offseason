## 2026-08-13T17:14:03-04:00
You are Explorer Survey 1 (Subsystems Focus).
Working Directory: C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1
Original Request: C:\Users\Robotiators\Documents\GitHub\2026Offseason\ORIGINAL_REQUEST.md

OBJECTIVE:
Investigate existing codebase for `SUB_Shooter` and `SUB_Hood` to support R1 (Inertia-Preserving / Hood-First Trajectory Controller) and R5 (Telemetry).
Specifically:
1. Examine `SUB_Shooter.java` and `SUB_Hood.java` (and related hardware/subsystem classes).
2. Document current state: methods, target RPM control, hood angle control, sensor inputs, limits, telemetry outputs.
3. Identify how target flywheel RPM vs target hood angle is currently computed or specified.
4. Recommend exact interface changes and methods needed for holding stable flywheel target RPM while adjusting hood angle for distance, only adjusting flywheel velocity when necessary or crossing major boundaries.
5. Identify all parameters, constants, and telemetry requirements (SmartDashboard).

DO NOT edit any code files.
Write your complete findings and handoff report to:
`C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_survey_1\handoff.md`.
Update `progress.md` in your working directory when done.
