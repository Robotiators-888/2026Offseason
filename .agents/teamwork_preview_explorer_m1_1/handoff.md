# Handoff Report — Explorer 1 (Milestone 1: TrajectorySolver & Domain Math Constants & Configurations)

## 1. Observation

Direct investigation of the project source code and specification documentation revealed the following exact constants, parameter locations, formulas, and baseline configurations:

### 1.1 Standard Gravity Constant `g`
- **File**: `src/main/java/frc/robot/Constants.java`
- **Line 49**:
  ```java
  public static final double kGRAVITATIONAL_CONSTANT = 9.80665;
  ```
- **Usage**: Referenced in `src/main/java/frc/robot/subsystems/SUB_Shooter.java` line 123 for projectile physics velocity calculation.

### 1.2 Target Hub Height Differential `h`
- **File**: `src/main/java/frc/robot/Constants.java`
- **Line 136**:
  ```java
  public static final double ScoreHeight = 55; // Inches
  ```
- **In Meters**: `Units.inchesToMeters(Constants.Hood.ScoreHeight)` = `1.397` meters ($55.0 \times 0.0254 = 1.397\text{ m}$).
- **Usage**:
  - `src/main/java/frc/robot/subsystems/SUB_Hood.java` (Line 46): `double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);`
  - `src/main/java/frc/robot/subsystems/SUB_Shooter.java` (Line 112): `double height = Units.inchesToMeters(Constants.Hood.ScoreHeight);`

### 1.3 Flywheel Wheel Diameter and Compression Ratio
- **File**: `src/main/java/frc/robot/Constants.java`
- **Lines 34–35 & 47**:
  ```java
  public static final double ShooterDiameterInches = 3.0;
  public static final double ShooterDiameter = 3.0;
  public static final double kSHOOTER_COMPRESSION_RATIO = 0.8;
  ```
- **In Meters**: Wheel diameter $D_{\text{wheel}} = 3.0\text{ inches} = 0.0762\text{ meters}$.
- **Compression Ratio**: $C_{\text{comp}} = 0.8$.
- **Physics Formula & Implementation**: `src/main/java/frc/robot/subsystems/SUB_Shooter.java` (Lines 124–127):
  ```java
  double wheelDiameterMeters = Units.inchesToMeters(Constants.Shooter.ShooterDiameter);
  double surfaceSpeed = exitVelocity / Constants.Shooter.kSHOOTER_COMPRESSION_RATIO;
  double rps = surfaceSpeed / (Math.PI * wheelDiameterMeters);
  double exitRPM = rps * 60.0;
  ```
- **Exact Conversion Equations**:
  $$v_0 = \text{RPM} \times \left(\frac{\pi \cdot D_{\text{wheel}} \cdot C_{\text{comp}}}{60}\right) = \text{RPM} \times \left(\frac{\pi \cdot 0.0762 \cdot 0.8}{60}\right) \approx \text{RPM} \times 0.003191858136\text{ m/s}$$
  $$\text{RPM} = v_0 \times \left(\frac{60}{\pi \cdot D_{\text{wheel}} \cdot C_{\text{comp}}}\right) = v_0 \times \left(\frac{60}{\pi \cdot 0.0762 \cdot 0.8}\right) \approx v_0 \times 313.29712438\text{ RPM per (m/s)}$$

### 1.4 Hood Angular Soft Limits and Gear Ratio Configuration
- **File**: `src/main/java/frc/robot/Constants.java` (Lines 133–137) & `src/main/java/frc/robot/subsystems/SUB_Hood.java`
- **Current Baseline Finding**: `Constants.Hood` currently only contains `kHOOD_CAN_ID = 47`, `kHOOD_PID_CONTROLLER`, and `ScoreHeight = 55`. Gear ratio and explicit angular soft limits $[\theta_{\min}, \theta_{\max}]$ are **not yet present in `Constants.java`**.
- **Specification Alignment**: `PROJECT.md` (Lines 9, 24, 94) specifies that `Constants.java` and `SUB_Hood` will incorporate `kHOOD_GEAR_RATIO` and mechanical soft limits $[\theta_{\min}, \theta_{\max}]$. For `TrajectorySolver` domain validation, the working envelope is bounded by typical physical limits (e.g., $\theta_{\min} \approx 0.35\text{ rad} / 20^\circ$ and $\theta_{\max} \approx 1.22\text{ rad} / 70^\circ$).

### 1.5 Target Distance Operational Limits
- **Specification Documents**: `ORIGINAL_REQUEST.md` (Line 32), `PROJECT.md` (Lines 17, 23, 30), `SCOPE.md` (Line 14).
- **Target Distance Envelope**: Validated operational zone is **$1.5\text{ m}$ to $7.0\text{ m}$**.
- **Existing Lookup Table in Code**: `src/main/java/frc/robot/subsystems/SUB_Shooter.java` (Lines 52–57):
  - $1.63\text{ m} \to 1050\text{ RPM}$
  - $2.49\text{ m} \to 1250\text{ RPM}$
  - $3.03\text{ m} \to 1390\text{ RPM}$
  - $4.11\text{ m} \to 1600\text{ RPM}$
  - $5.35\text{ m} \to 1790\text{ RPM}$
  - $10.5\text{ m} \to 2400\text{ RPM}$

---

## 2. Logic Chain

1. **Height Differential Parameter ($h = 1.397\text{ m}$)**:
   - `Constants.Hood.ScoreHeight` is $55.0$ inches. `Units.inchesToMeters(55.0)` converts this value to $1.397\text{ m}$.
   - Both `SUB_Hood.findoptimalangle` and `SUB_Shooter.findoptimalRPM` use this height differential directly as the target $y$-coordinate ($h$) relative to launcher release point in kinematics equations.

2. **Gravitational Acceleration ($g = 9.80665\text{ m/s}^2$)**:
   - `Constants.Shooter.kGRAVITATIONAL_CONSTANT` defines $g = 9.80665\text{ m/s}^2$. This exact value ensures floating-point precision alignment across all trajectory calculations and tests.

3. **Flywheel Kinematics ($v_0 \leftrightarrow \text{RPM}$)**:
   - $v_{\text{surface}} = \frac{\text{RPM}}{60} \cdot \pi \cdot D_{\text{wheel}}$.
   - $v_0 = v_{\text{surface}} \cdot C_{\text{comp}}$.
   - Substituting $D_{\text{wheel}} = 0.0762\text{ m}$ and $C_{\text{comp}} = 0.8$:
     - Multiplier for RPM $\to v_0$: $k_{v} = \frac{\pi \times 0.0762 \times 0.8}{60} \approx 0.003191858136$
     - Multiplier for $v_0 \to \text{RPM}$: $k_{\text{rpm}} = \frac{1}{k_v} \approx 313.29712438$
   - `TrajectorySolver.exitVelocityToRPM(v0)` and `TrajectorySolver.rpmToExitVelocity(rpm)` in M1 must use these exact constant ratios.

4. **Domain Boundaries & Mechanical Soft Limits**:
   - Distance limits $1.5\text{ m} \le d \le 7.0\text{ m}$ guard against out-of-range targeting where quadratic solutions become steep/unstable or exceed motor speed safety ceilings.
   - Discriminant check $\Delta = v_0^4 - g(g d^2 + 2 h v_0^2) < 0$ determines whether a given flywheel RPM can physically reach target height $h$ at distance $d$.

---

## 3. Caveats

1. **Read-Only Investigation**: In strict compliance with guidelines, no project code files were modified.
2. **Missing `Constants` Fields**: `kHOOD_GEAR_RATIO` and soft limit constants $[\theta_{\min}, \theta_{\max}]$ are not yet in `Constants.java` (scheduled to be added in M2). `TrajectorySolver` in M1 should accept hood angle limits as parameters or define sensible default constants in `TrajectorySolver` until `Constants.Hood` is updated.
3. **Height Offset Assumptions**: `ScoreHeight` ($55\text{ inches} / 1.397\text{ m}$) represents total target height relative to launcher height 0. If field measurements adjust for effective launcher exit height on the physical robot frame, $h$ will be calculated as $h_{\text{target}} - h_{\text{launcher}}$.

---

## 4. Conclusion

- All required physical and mathematical constants are fully identified in `Constants.java`:
  - $g = 9.80665\text{ m/s}^2$ (`Constants.Shooter.kGRAVITATIONAL_CONSTANT`)
  - $h = 1.397\text{ m}$ (`Units.inchesToMeters(Constants.Hood.ScoreHeight)`)
  - $D_{\text{wheel}} = 3.0\text{ in} = 0.0762\text{ m}$ (`Constants.Shooter.ShooterDiameter`)
  - $C_{\text{comp}} = 0.8$ (`Constants.Shooter.kSHOOTER_COMPRESSION_RATIO`)
- Target distance domain limits are established as **$1.5\text{ m}$ to $7.0\text{ m}$**.
- Conversions between exit velocity $v_0$ and flywheel RPM are mathematically exact and ready for implementation in `TrajectorySolver.java`.

---

## 5. Verification Method

### 5.1 Independent Codebase Verification Commands
Run the standard WPILib compilation check in PowerShell:
```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
.\gradlew.bat compileJava
```
*Expected Result*: `BUILD SUCCESSFUL`, 0 errors, 0 warnings.

Run unit test execution:
```powershell
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
.\gradlew.bat test -x extractReleaseNative -x extractDebugNative
```
*Expected Result*: All existing unit tests in `PhysicsAndMathTest.java` pass cleanly.

### 5.2 Files to Inspect
- `src/main/java/frc/robot/Constants.java` (Lines 34–35, 47, 49, 136)
- `src/main/java/frc/robot/subsystems/SUB_Shooter.java` (Lines 111–133)
- `src/main/java/frc/robot/subsystems/SUB_Hood.java` (Lines 45–48)
- `src/test/java/frc/robot/PhysicsAndMathTest.java`

### 5.3 Invalidation Conditions
- Modifications to `Constants.java` for `kGRAVITATIONAL_CONSTANT`, `ShooterDiameter`, `kSHOOTER_COMPRESSION_RATIO`, or `ScoreHeight`.
- Physical changes to the shooter wheel size or mechanical hood geometry.
