## 2026-08-13T21:16:05Z
Investigate existing codebase constants (`Constants.java` or related constant files), existing shooter/hood configurations, target height differential parameters, flywheel dimensions, compression ratios, and hood angular soft limits.

INSTRUCTIONS:
1. Read ORIGINAL_REQUEST.md at C:\Users\Robotiators\Documents\GitHub\2026Offseason\ORIGINAL_REQUEST.md
2. Read PROJECT.md at C:\Users\Robotiators\Documents\GitHub\2026Offseason\PROJECT.md
3. Read SCOPE.md at C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\sub_orch_m1\SCOPE.md
4. Inspect src/main/java/frc/robot/Constants.java (and any other files in src/main/java/frc/robot/) to find:
   - Target Hub height differential h (or how height differential is defined/calculated).
   - Standard gravity constant g (e.g. 9.81 m/s^2).
   - Flywheel wheel diameter (3.0 in = 0.0762 m) and compression ratio (0.8).
   - Hood soft limits [theta_min, theta_max].
   - Distance limits (1.5 m to 7.0 m).

OUTPUT:
Write your investigation report and evidence to C:\Users\Robotiators\Documents\GitHub\2026Offseason\.agents\teamwork_preview_explorer_m1_1\handoff.md.
Then send a message back to parent with your summary and link to handoff.md.
