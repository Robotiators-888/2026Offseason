# E2E & Unit Test Infra: Robot Shooter and Aiming Refactor

## Test Philosophy
- Requirement-driven, multi-tier testing for trajectory solving, domain boundaries, flywheel stability, and full build/test verification.
- Test commands:
  - Compilation: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat compileJava`
  - Test Suite: `$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"; .\gradlew.bat test -x extractReleaseNative -x extractDebugNative`

## Feature Inventory & Test Matrix
| # | Feature | Requirement | Tier 1 (Coverage) | Tier 2 (Boundary) | Tier 3 (Interactions) | Tier 4 (Workloads) |
|---|---------|-------------|:-----------------:|:-----------------:|:---------------------:|:------------------:|
| 1 | Fixed-$v_0$ Launch Angle Solver | R2.2, R2.3 | 5 test cases | 5 test cases | ✓ | ✓ |
| 2 | Trajectory Domain Validation & Fallback | R2.1, R2.4, R2.5 | 5 test cases | 5 test cases | ✓ | ✓ |
| 3 | Inertia-Preserving Hood & Shooter Controllers | R1.1, R1.2, R1.3, R1.4 | 5 test cases | 5 test cases | ✓ | ✓ |
| 4 | Telemetry & Readiness Gate | R5.1, R5.2, R5.3 | 5 test cases | 5 test cases | ✓ | ✓ |

## Test Suite Location
- `src/test/java/frc/robot/PhysicsAndMathTest.java`
