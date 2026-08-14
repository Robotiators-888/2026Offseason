// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        /** Controller port mapping and driver preferences */
        public static class Operator {
                public static final int kDriver1ControllerPort = 0;
                public static final int kDriver2ControllerPort = 1;
                public static final double kDriveDeadband = 0.1;
        }

        /** Shooter motor IDs and PID/Feedforward tuning (Manually Tuned) */
        public static final class Shooter {
                public static final int kSHOOTER_FOLLOWER_MOTOR_CANID = 44;
                public static final int kSHOOTER_LEADER_MOTOR_CANID = 43;

                //Physical Specs
                public static final double ShooterDiameter = 3;                
                // Feedforward constants (Manual tuning in progress)
                public static final double kSHOOTER_FLYWHEEL_kS = 0.0;
                public static final double kSHOOTER_FLYWHEEL_kV = 0.14; 
                public static final double kSHOOTER_FLYWHEEL_kA = 0.0;
                
                // PID constants
                public static final double kSHOOTER_FLYWHEEL_kP = 0.5;
                public static final double kSHOOTER_FLYWHEEL_kI = 0;
                public static final double kSHOOTER_FLYWHEEL_kD = 0;

                public static final double kSHOOTER_COMPRESSION_RATIO = .8; // Todo find ratio

                public static final double kGRAVITATIONAL_CONSTANT = 9.80665;

                /**
                 * Standing flywheel speeds, ascending. The shooter holds the lowest band that can
                 * still reach the target and lets the hood cover everything inside that band, so a
                 * range change costs a hood move rather than a flywheel spin-up.
                 *
                 * <p>Values are RPM from the tuned look-up table: 1200 covers out to roughly 2.3 m,
                 * 1500 to roughly 3.5 m, 1800 to roughly 5.4 m. Past that the table itself is
                 * uncalibrated — its 10.5 m entry is still marked TODO.
                 *
                 * <p>A single-entry array gives literal fixed-RPM behaviour. It is deliberately not
                 * the default: at 1800 RPM and 1.6 m the only hood solutions are about 85 degrees
                 * (near-vertical lob) or 45 degrees (flat and fast), because the flywheel is
                 * massively overpowered for that range. Banding keeps the hood near the
                 * minimum-energy angle where the shot is least sensitive to error.
                 *
                 * <p>With these three bands the hood sweeps about 52 to 78 degrees across 1.5 m to
                 * 5.5 m, and the flywheel changes speed at only two distances (~2.3 m and ~3.5 m)
                 * instead of continuously. Adding bands shrinks the hood sweep and adds spin-up
                 * events; removing them does the reverse.
                 */
                public static final double[] kREADY_RPM_BANDS = {1200, 1500, 1800};

                /** How far past a band's reach the robot must go before stepping down a band. */
                public static final double kBAND_HYSTERESIS = 0.90;

                /** At-speed tolerance to arm a shot, in RPM. */
                public static final double kRPM_TOLERANCE = 75;

                /**
                 * Looser at-speed tolerance used to keep feeding once a shot has started, in RPM.
                 * Ball contact drops the flywheel well past the arming tolerance, and cutting the
                 * indexer mid-ball jams the feed.
                 */
                public static final double kRPM_SUSTAIN_TOLERANCE = 250;

                /**
                 * How long the full arm condition (aligned, at speed, hood arrived, in range) must
                 * hold continuously before the feed opens. A single 20 ms blip is jitter, not
                 * readiness.
                 */
                public static final double kFEED_ARM_DEBOUNCE_SECONDS = 0.10;

                /**
                 * Effective half-width of the hub opening for aiming, in meters: half the 41.7 in
                 * hexagonal opening minus a 5.91 in FUEL radius. The heading tolerance is the angle
                 * this subtends at the current range.
                 */
                public static final double kAIM_HALF_WIDTH_METERS =
                                Units.inchesToMeters((41.7 - 5.91) / 2.0);

                /** Fraction of the geometric tolerance teleop aiming actually accepts. */
                public static final double kAIM_SAFETY_FACTOR_TELEOP = 0.8;

                /** Tighter than teleop, since auto has no driver to judge the shot. */
                public static final double kAIM_SAFETY_FACTOR_AUTO = 0.6;

                /** Heading-controller gains shared by the aim and shuttle commands. */
                public static final double kAIM_HEADING_kP = 5.0;
                public static final double kAIM_HEADING_kD = 0.2;

                /** Inside this heading error the rotation output is zeroed so the robot settles. */
                public static final double kAIM_HEADING_TOLERANCE_RADS = Units.degreesToRadians(0.75);

                /** Constant term that breaks drivetrain stiction, applied outside the tolerance band. */
                public static final double kAIM_STICTION_RADS_PER_SEC = Units.degreesToRadians(9);

                /**
                 * The stiction term ramps in over this width past the heading tolerance instead of
                 * switching on — a step there is a limit cycle: the robot settles, the term
                 * vanishes, it drifts out, and the term slams back in.
                 */
                public static final double kAIM_STICTION_RAMP_WIDTH_RADS = Units.degreesToRadians(2);

                /**
                 * Hood angle held while shuttling, in mechanism radians. A shuttle is a lob onto
                 * the floor, not a shot at the hub opening, so the analytic hood solve (which
                 * targets ScoreHeight above the shooter) does not apply — this is a fixed lob
                 * angle. BENCH TUNE.
                 */
                public static final double kSHUTTLE_HOOD_ANGLE_RADS = Units.degreesToRadians(45);

                /**
                 * The shuttle flywheel setpoint only moves when the table value drifts this far,
                 * in RPM, from the current setpoint. Must comfortably exceed the at-speed
                 * tolerance, or the setpoint chases odometry jitter and atDesiredRPM() never
                 * latches long enough to feed.
                 */
                public static final double kSHUTTLE_RPM_LATCH_DEADBAND = 150;
        }

        /** Intake roller motor configuration */
        public static final class Roller {
                public static final int kINTAKE_LEFTMOTOR_CANID = 30;

                /**
                 * NOTE: shares ID 31 with {@link Linear#kLINEAR_MOTOR_CANID}. This does not collide
                 * on the wire — FRC CAN arbitration includes the manufacturer and device type, so a
                 * CTRE 31 and a REV 31 are distinct — but it is the only duplicate in the robot and
                 * it will bite whoever next re-IDs hardware. Renumbering means physically re-IDing
                 * the motor, so it is left alone rather than changed silently here.
                 */
                public static final int kINTAKE_RIGHTMOTOR_CANID = 31;
                public static final double kROLLER_MOTOR_VOLTAGE = 10.91276304645254;
        }

        /** Intake arm motor configuration and relative setpoints (NEO 2.0 Encoder) */
        public static final class Linear {
                public static final int kLINEAR_MOTOR_CANID = 31;

                /**
                 * Motor rotations (relative to the retracted position). No position conversion
                 * factor is configured on the SparkMax, so the encoder reads MOTOR rotations —
                 * the old value of {@code 360.0*4.2} treated it as degrees, commanding the arm
                 * 360x past its stop so it only ever rode the output clamp into the hard stop.
                 * BENCH VERIFY the deploy pose actually lands where intended.
                 */
                public static final double kLINEAR_FORWARD_SETPOINT = 4.2;
                public static final double kLINEAR_BACKWARD_SETPOINT = 0;       // Motor rotations (Relative)

                // Gains, not controllers. SUB_Linear builds its own PIDController instances from
                // these — a shared static controller carries mutable loop state between commands.
                public static final double kLINEAR_FAST_kP = 4;
                public static final double kLINEAR_SLOW_kP = 1;
        }
        
        /** Indexing system: Spindexer and Metering wheel */
        public static final class Index {
                public static final int KINDEX_MOTOR_CANID = 41;      // Spindexer motor
                public static final int kMETERING_WHEEL_CANID = 42;   // High-speed feed to shooter
                
                public static final double kINDEX_MOTOR_VOLTS = 8.5;    // Spindexer feed voltage
        }

        /** Standard field measurements in meters */
        public static final class Field {
                /**
                 * The one AprilTag layout the whole robot agrees on — vision, aiming, and alliance
                 * flipping all resolve through this. Keeping a single reference is what stops the
                 * field dimensions and the tag poses from drifting apart.
                 */
                public static final AprilTagFieldLayout kTagLayout =
                                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

                /**
                 * Derived from the layout above rather than typed in. The previous literals
                 * (16.532 x 8.001) matched no real field — they were ~14 mm long and ~42 mm narrow
                 * against 2026 Rebuilt AndyMark, which skewed every red-alliance flip and clipped
                 * valid vision poses near the field edge.
                 */
                public static final double fieldLength = kTagLayout.getFieldLength();
                public static final double fieldWidth = kTagLayout.getFieldWidth();
        }

        /** Vision camera names and physical transformations on the robot */
        public static final class PhotonVision {
                public static final String kCamName1 = "BackLeftCam";

                // These three were declared but never referenced; the filter in
                // RobotContainer.processCameraPose() hardcoded its own values instead. They are set
                // here to the values that were actually running (0.2 ambiguity, 4.0 m) rather than
                // the untested numbers that used to sit here, so wiring them up changes nothing
                // about behaviour. kMaxZError is newly enforced — there was no Z check at all.

                /** Max height above the floor, in meters, before a pose solution is discarded. */
                public static final double kMaxZError = 0.2;

                /** Max per-tag pose ambiguity; any worse tag in the solve rejects the whole frame. */
                public static final double kMaxAmbiguity = 0.2;

                /**
                 * Max distance to the nearest tag used, in meters. Raised from 4.0: shooting
                 * range extends past 5.4 m, and cutting vision off at 4 m left the pose to drift
                 * on wheel odometry exactly where the aim solution is most distance-sensitive.
                 * The quadratic std-dev scaling already de-weights far solves.
                 */
                public static final double kMaxDistance = 6.0;

                /** Divisor in xyStddev = dist^2 / divisor for single-tag fallback solves. */
                public static final double kSingleTagStddevDivisor = 16.0;

                /** Same, for multi-tag PnP solves, which are far more trustworthy. FIELD TUNE. */
                public static final double kMultiTagStddevDivisor = 40.0;

                public static final Rotation3d cameraRotation = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(-25),
                                Units.degreesToRadians(172-90));
                public static final Transform3d kRobotToCamera1 = new Transform3d(
                                Units.inchesToMeters(-11.55), Units.inchesToMeters(10.5),
                                Units.inchesToMeters(7.8), cameraRotation);

                public static final String kCam2Name = "BackRightCam"; 
                public static final Rotation3d cameraRotation2 = new Rotation3d(
                                Units.degreesToRadians(0), Units.degreesToRadians(-25),
                                Units.degreesToRadians(-172+90));
                public static final Transform3d kRobotToCamera2 = new Transform3d(
                                Units.inchesToMeters(-11.55), Units.inchesToMeters(-10.5),
                                Units.inchesToMeters(7.8), cameraRotation2);


                public static final String kCam3Name = "HighCam";
                public static final Rotation3d cameraRotation3 = new Rotation3d(Units.degreesToRadians(3),
                                 Units.degreesToRadians(-1.5), Units.degreesToRadians(-4));
                public static final Transform3d kRobotToCamera3 = new Transform3d(
                                 Units.inchesToMeters(-4), Units.inchesToMeters(0),
                                 Units.inchesToMeters(20.5), cameraRotation3);
        }

        public static class Hood {
                public static final int kHOOD_CAN_ID = 47;

                /** Height of the scoring target above the shooter exit, in inches. */
                public static final double ScoreHeight = 55;

                /**
                 * Motor rotations per rotation of the hood mechanism. Feeding this to the TalonFX as
                 * SensorToMechanismRatio makes getPosition() report mechanism rotations, so every
                 * angle in code can be plain radians.
                 *
                 * <p>MEASURE THIS. 1.0 means "direct drive", which is almost certainly wrong; until
                 * it is measured the hood will move by the wrong amount even though the units now
                 * line up. Rotate the hood a known amount by hand and read Hood/Angle (Deg).
                 */
                public static final double kHOOD_GEAR_RATIO = 1.0;

                /**
                 * Mechanism angle, in radians, when the encoder reads zero — i.e. the hood sitting
                 * against its retracted hard stop. MEASURE THIS TOO.
                 */
                public static final double kHOOD_ANGLE_AT_ZERO_RADS = 0.0;

                /**
                 * Travel limits of the hood, as mechanism angles in radians. MEASURE THESE.
                 *
                 * <p>The banded-RPM scheme in {@link Constants.Shooter#kREADY_RPM_BANDS} needs the
                 * hood to cover roughly <b>52 to 78 degrees</b> to shoot from 1.5 m to 5.5 m. If the
                 * real mechanism has less travel than that, widen the bands (more, closer-spaced
                 * entries shrink the hood sweep) rather than clipping the range.
                 */
                public static final double kHOOD_MIN_ANGLE_RADS = 0.0;
                public static final double kHOOD_MAX_ANGLE_RADS = Math.PI / 2.0;

                /**
                 * Slot0 gains for the on-motor position loop, in volts per mechanism rotation of
                 * error. These are NOT the old duty-cycle gains — the previous kP of 4 was in a
                 * different unit system and does not carry over. Tune on the bench.
                 */
                public static final double kHOOD_kP = 8.0;
                public static final double kHOOD_kI = 0.0;
                public static final double kHOOD_kD = 0.0;

                /** Stator current above which the homing routine decides the hood has hit its stop. */
                public static final double kHOOD_STALL_AMPS = 20.0;

                /** Open-loop output used while homing toward the retracted stop. */
                public static final double kHOOD_HOMING_OUTPUT = -0.08;

                /** Give up homing after this long so a failed sensor cannot stall the motor forever. */
                public static final double kHOOD_HOMING_TIMEOUT_SECONDS = 2.0;
        }
        public static class Metering {
                public static final int kMETERING_MOTOR_CAN_ID = 45;
                public static final int kMETERING_MOTOR_FOLLOWER_CAN_ID = 46;
        }
}
