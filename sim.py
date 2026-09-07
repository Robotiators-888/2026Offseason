"""FRC 2026 2D Multi-Robot Autonomous & Physics Simulation.

This module provides a full-featured 2D physics simulation of the 2026 FRC game,
including:
    - Accurate field layout with neutral zone, depots, and alliance hubs.
    - Ball-ball spatial hash grid collisions (zero clipping).
    - Multi-robot swerve kinematics with bumper-to-bumper collision dynamics.
    - High-throughput progressive roller intake (1,000 RPM, 2" diameter, 50-ball hopper limit).
    - High-rate 10 balls/second shooter mechanism with continuous auto-feeding.
    - AimBot autonomous targeting with zoned RPM hysteresis and 3D projectile arc physics.
    - Native PathPlanner trajectory parsing and time-parameterized velocity profiling.
    - Interactive Tkinter desktop GUI with live telemetry and autonomous routine controls.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
import time
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Optional, Tuple

# ==============================================================================
# 1. FIELD DIMENSIONS & ROBOT PHYSICAL CONSTANTS
# ==============================================================================

# Field Dimensions (Meters)
FIELD_LENGTH_METERS: float = 16.541
FIELD_WIDTH_METERS: float = 8.211

# Robot Dimensions & Dynamics
ROBOT_WIDTH_METERS: float = 0.88
ROBOT_LENGTH_METERS: float = 0.88
ROBOT_COLLISION_RADIUS: float = math.hypot(ROBOT_WIDTH_METERS / 2.0, ROBOT_LENGTH_METERS / 2.0)
MAX_LINEAR_VELOCITY: float = 4.5       # Meters per second
MAX_LINEAR_ACCELERATION: float = 3.5   # Meters per second squared
MAX_ANGULAR_VELOCITY: float = 6.28     # Radians per second
MAX_ANGULAR_ACCELERATION: float = 9.42 # Radians per second squared

# Physics Constants
SURFACE_FRICTION_COEFFICIENT: float = 0.94
GRAVITATIONAL_ACCELERATION: float = 9.81  # Meters per second squared

# Intake Geometry (Relative to Robot Center, in Meters)
INTAKE_OFFSET_X: float = 0.35
INTAKE_OFFSET_Y: float = 0.0
INTAKE_WIDTH: float = 0.65
INTAKE_LENGTH: float = 0.28

# Intake Roller Dynamics
INTAKE_ROLLER_RPM: float = 1000.0
INTAKE_ROLLER_DIAMETER_INCHES: float = 2.0
INTAKE_ROLLER_DIAMETER_METERS: float = INTAKE_ROLLER_DIAMETER_INCHES * 0.0254
INTAKE_SURFACE_SPEED_MPS: float = (
    INTAKE_ROLLER_RPM * math.pi * INTAKE_ROLLER_DIAMETER_METERS
) / 60.0  # Approx 2.66 m/s

# Shooter Geometry (Relative to Robot Center, in Meters)
SHOOTER_OFFSET_X: float = -0.0762
SHOOTER_OFFSET_Y: float = 0.1651

# Alliance Hub Target Coordinates (Meters)
BLUE_HUB_COORDINATES: Tuple[float, float] = (4.025, 4.105)
RED_HUB_COORDINATES: Tuple[float, float] = (12.516, 4.105)

# Game Piece (Ball) Specifications
BALL_DIAMETER_METERS: float = 0.150114  # 5.91 inches
BALL_RADIUS_METERS: float = BALL_DIAMETER_METERS / 2.0
MAX_HOPPER_CAPACITY: int = 50

# Neutral Zone Boundaries (Centered 30 ft x 12 ft)
NEUTRAL_ZONE_CENTER_X: float = FIELD_LENGTH_METERS / 2.0
NEUTRAL_ZONE_CENTER_Y: float = FIELD_WIDTH_METERS / 2.0
NEUTRAL_ZONE_MIN_X: float = NEUTRAL_ZONE_CENTER_X - (15.0 * 0.3048)  # 30 ft span
NEUTRAL_ZONE_MAX_X: float = NEUTRAL_ZONE_CENTER_X + (15.0 * 0.3048)
NEUTRAL_ZONE_MIN_Y: float = NEUTRAL_ZONE_CENTER_Y - (6.0 * 0.3048)   # 12 ft span
NEUTRAL_ZONE_MAX_Y: float = NEUTRAL_ZONE_CENTER_Y + (6.0 * 0.3048)

# Depot Boundaries (Red & Blue Alliances)
DEPOT_DELTA_X: float = 2.0 * 0.3048
DEPOT_HALF_WIDTH_Y: float = 2.0 * 0.3048

BLUE_DEPOT_MIN_X: float = 0.0
BLUE_DEPOT_MAX_X: float = DEPOT_DELTA_X
BLUE_DEPOT_MIN_Y: float = (FIELD_WIDTH_METERS / 2.0) - DEPOT_HALF_WIDTH_Y
BLUE_DEPOT_MAX_Y: float = (FIELD_WIDTH_METERS / 2.0) + DEPOT_HALF_WIDTH_Y

RED_DEPOT_MIN_X: float = FIELD_LENGTH_METERS - DEPOT_DELTA_X
RED_DEPOT_MAX_X: float = FIELD_LENGTH_METERS
RED_DEPOT_MIN_Y: float = (FIELD_WIDTH_METERS / 2.0) - DEPOT_HALF_WIDTH_Y
RED_DEPOT_MAX_Y: float = (FIELD_WIDTH_METERS / 2.0) + DEPOT_HALF_WIDTH_Y

# AimBot Hysteresis & Transitions
AIMBOT_ZONE1_TO_2_THRESHOLD: float = 3.65
AIMBOT_ZONE2_TO_1_THRESHOLD: float = 3.55
AIMBOT_ZONE2_TARGET_RPM: float = 3000.0
AIMBOT_ZONE3_TARGET_RPM: float = 3400.0


# ==============================================================================
# 2. MATHEMATICAL & GEOMETRICAL HELPERS
# ==============================================================================

def normalize_angle(angle_radians: float) -> float:
    """Normalize an angle in radians to the range [-pi, pi]."""
    while angle_radians > math.pi:
        angle_radians -= 2.0 * math.pi
    while angle_radians < -math.pi:
        angle_radians += 2.0 * math.pi
    return angle_radians


def angle_difference(target_angle_rad: float, current_angle_rad: float) -> float:
    """Compute the shortest signed angular difference (target - current)."""
    return normalize_angle(target_angle_rad - current_angle_rad)


def compute_cubic_bezier(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    t: float,
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    """Evaluate a cubic Bezier curve point, 1st derivative, and 2nd derivative at parameter t."""
    u: float = 1.0 - t
    u2: float = u * u
    u3: float = u2 * u
    t2: float = t * t
    t3: float = t2 * t

    # Position
    pos_x = u3 * p0[0] + 3.0 * u2 * t * p1[0] + 3.0 * u * t2 * p2[0] + t3 * p3[0]
    pos_y = u3 * p0[1] + 3.0 * u2 * t * p1[1] + 3.0 * u * t2 * p2[1] + t3 * p3[1]

    # 1st Derivative (Velocity vector along curve)
    vel_x = 3.0 * u2 * (p1[0] - p0[0]) + 6.0 * u * t * (p2[0] - p1[0]) + 3.0 * t2 * (p3[0] - p2[0])
    vel_y = 3.0 * u2 * (p1[1] - p0[1]) + 6.0 * u * t * (p2[1] - p1[1]) + 3.0 * t2 * (p3[1] - p2[1])

    # 2nd Derivative (Acceleration vector)
    acc_x = 6.0 * u * (p2[0] - 2.0 * p1[0] + p0[0]) + 6.0 * t * (p3[0] - 2.0 * p2[0] + p1[0])
    acc_y = 6.0 * u * (p2[1] - 2.0 * p1[1] + p0[1]) + 6.0 * t * (p3[1] - 2.0 * p2[1] + p1[1])

    return (pos_x, pos_y), (vel_x, vel_y), (acc_x, acc_y)


def calculate_aimbot_rpm(distance_meters: float, current_rpm: float) -> float:
    """Calculate flywheel RPM setpoint using zoned hysteresis matching Java CMD_AimBotAuto."""
    if distance_meters < AIMBOT_ZONE2_TO_1_THRESHOLD:
        return 112.5 * distance_meters + 2337.5

    if distance_meters <= AIMBOT_ZONE1_TO_2_THRESHOLD:
        if current_rpm >= AIMBOT_ZONE2_TARGET_RPM - 150.0:
            return AIMBOT_ZONE2_TARGET_RPM
        return 112.5 * distance_meters + 2337.5

    if distance_meters < 4.25:
        return AIMBOT_ZONE2_TARGET_RPM

    return AIMBOT_ZONE3_TARGET_RPM


def calculate_shooter_pitch(distance_meters: float) -> float:
    """Calculate shooter hood pitch in degrees based on distance to alliance hub."""
    if distance_meters < 2.5:
        return 52.0
    if distance_meters < 3.8:
        return 55.0
    if distance_meters < 4.8:
        return 58.0
    return 61.0


# ==============================================================================
# 3. BALL & PROJECTILE PHYSICS
# ==============================================================================

class Ball:
    """Represents a physical game piece on the field carpet."""

    def __init__(self, x: float, y: float) -> None:
        self.x: float = x
        self.y: float = y
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.radius: float = BALL_RADIUS_METERS
        self.is_collected: bool = False
        self.intake_progress: float = 0.0

    def update(self, dt: float) -> None:
        """Update ball position and apply carpet rolling resistance."""
        if self.is_collected:
            return

        self.x += self.vx * dt
        self.y += self.vy * dt

        # Apply friction
        self.vx *= SURFACE_FRICTION_COEFFICIENT
        self.vy *= SURFACE_FRICTION_COEFFICIENT

        # Stop micro-drifting
        if math.hypot(self.vx, self.vy) < 0.01:
            self.vx = 0.0
            self.vy = 0.0

        # Field boundary collision
        if self.x < self.radius:
            self.x = self.radius
            self.vx = -self.vx * 0.5
        elif self.x > FIELD_LENGTH_METERS - self.radius:
            self.x = FIELD_LENGTH_METERS - self.radius
            self.vx = -self.vx * 0.5

        if self.y < self.radius:
            self.y = self.radius
            self.vy = -self.vy * 0.5
        elif self.y > FIELD_WIDTH_METERS - self.radius:
            self.y = FIELD_WIDTH_METERS - self.radius
            self.vy = -self.vy * 0.5


class Projectile:
    """Represents a ball launched towards an alliance hub through 3D ballistic arc."""

    def __init__(
        self,
        start_pos: Tuple[float, float],
        target_pos: Tuple[float, float],
        rpm: float,
        pitch_deg: float,
    ) -> None:
        self.x: float = start_pos[0]
        self.y: float = start_pos[1]
        self.z: float = 0.45  # Shooter exit height in meters

        self.target_x: float = target_pos[0]
        self.target_y: float = target_pos[1]

        # Muzzle velocity scaling from flywheel surface speed
        wheel_radius_meters = 0.0508
        muzzle_velocity = (rpm * 2.0 * math.pi / 60.0) * wheel_radius_meters * 0.72

        pitch_rad = math.radians(pitch_deg)
        horizontal_speed = muzzle_velocity * math.cos(pitch_rad)
        self.vz: float = muzzle_velocity * math.sin(pitch_rad)

        yaw = math.atan2(target_pos[1] - self.y, target_pos[0] - self.x)
        self.vx: float = horizontal_speed * math.cos(yaw)
        self.vy: float = horizontal_speed * math.sin(yaw)

        self.is_active: bool = True
        self.scored: bool = False

    def update(self, dt: float) -> None:
        """Advance ballistic projectile position under gravity."""
        if not self.is_active:
            return

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        self.vz -= GRAVITATIONAL_ACCELERATION * dt

        # Target hub proximity detection
        dist_to_hub = math.hypot(self.x - self.target_x, self.y - self.target_y)
        if dist_to_hub < 0.65 and 1.2 <= self.z <= 2.8:
            self.scored = True
            self.is_active = False

        if self.z <= 0.0:
            self.is_active = False


# ==============================================================================
# 4. ROBOT SUBSYSTEMS & STATE
# ==============================================================================

class Robot:
    """Models an autonomous FRC robot with swerve drive, intake, and aimbot subsystems."""

    def __init__(
        self,
        name: str,
        alliance: str,
        initial_x: float,
        initial_y: float,
        initial_heading: float,
    ) -> None:
        self.name: str = name
        self.alliance: str = alliance.lower()
        self.x: float = initial_x
        self.y: float = initial_y
        self.heading: float = initial_heading

        # Velocity states
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.omega: float = 0.0

        # Hardware states
        self.hopper_balls: int = 0
        self.intake_deployed: bool = True
        self.is_shooting: bool = False
        self.flywheel_rpm: float = 0.0
        self.shots_fired: int = 0
        self.score_count: int = 0

        # Subsystem timers & rates
        self.shot_cooldown: float = 0.0
        self.shooting_rate_balls_per_sec: float = 10.0

        # AimBot state
        self.aimbot_locked: bool = False
        self.distance_to_hub: float = 0.0
        self.target_hub_pos: Tuple[float, float] = (
            BLUE_HUB_COORDINATES if self.alliance == "blue" else RED_HUB_COORDINATES
        )

        # Autonomous mode assignment
        self.auto_mode: str = "auto"

    def get_shooter_field_position(self) -> Tuple[float, float]:
        """Compute the world coordinates of the robot's shooter exit."""
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        shooter_x = self.x + (SHOOTER_OFFSET_X * cos_h - SHOOTER_OFFSET_Y * sin_h)
        shooter_y = self.y + (SHOOTER_OFFSET_X * sin_h + SHOOTER_OFFSET_Y * cos_h)
        return shooter_x, shooter_y

    def get_bumper_polygon(self) -> List[Tuple[float, float]]:
        """Compute the four corner vertices of the robot's perimeter bumper."""
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        half_w = ROBOT_WIDTH_METERS / 2.0
        half_l = ROBOT_LENGTH_METERS / 2.0

        corners = []
        for dx, dy in [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]:
            wx = self.x + (dx * cos_h - dy * sin_h)
            wy = self.y + (dx * sin_h + dy * cos_h)
            corners.append((wx, wy))
        return corners

    def update(self, dt: float) -> None:
        """Update robot kinematics, AimBot calculations, and shooter cooldowns."""
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.heading = normalize_angle(self.heading + self.omega * dt)

        # Clamp position within field walls
        half_diag = ROBOT_WIDTH_METERS / 2.0
        self.x = max(half_diag, min(FIELD_LENGTH_METERS - half_diag, self.x))
        self.y = max(half_diag, min(FIELD_WIDTH_METERS - half_diag, self.y))

        # AimBot calculations
        shooter_x, shooter_y = self.get_shooter_field_position()
        delta_x = self.target_hub_pos[0] - shooter_x
        delta_y = self.target_hub_pos[1] - shooter_y
        self.distance_to_hub = math.hypot(delta_x, delta_y)

        # Angular alignment to hub
        target_heading = math.atan2(delta_y, delta_x)
        alignment_error = abs(angle_difference(target_heading, self.heading))
        self.aimbot_locked = alignment_error < math.radians(6.5)

        # Flywheel speed ramp
        target_rpm = calculate_aimbot_rpm(self.distance_to_hub, self.flywheel_rpm)
        rpm_rate = 2500.0 * dt
        if self.flywheel_rpm < target_rpm:
            self.flywheel_rpm = min(target_rpm, self.flywheel_rpm + rpm_rate)
        else:
            self.flywheel_rpm = max(target_rpm, self.flywheel_rpm - rpm_rate)

        # Shooter rate limiter
        if self.shot_cooldown > 0.0:
            self.shot_cooldown = max(0.0, self.shot_cooldown - dt)

    def can_shoot(self) -> bool:
        """Return True if robot is ready to launch a ball towards its hub."""
        return (
            self.is_shooting
            and self.hopper_balls > 0
            and self.shot_cooldown <= 0.0
            and self.flywheel_rpm >= 2000.0
        )

    def shoot(self) -> Optional[Projectile]:
        """Fire one ball from the hopper, returning a Projectile instance."""
        if not self.can_shoot():
            return None

        self.hopper_balls -= 1
        self.shots_fired += 1
        self.shot_cooldown = 1.0 / self.shooting_rate_balls_per_sec

        shooter_pos = self.get_shooter_field_position()
        pitch = calculate_shooter_pitch(self.distance_to_hub)
        return Projectile(shooter_pos, self.target_hub_pos, self.flywheel_rpm, pitch)


# ==============================================================================
# 5. MULTI-ROBOT COLLISION RESOLUTION
# ==============================================================================

def resolve_robot_collisions(robots: List[Robot]) -> None:
    """Resolve physical bumper collisions between pairs of robots."""
    bumper_clearance_limit = ROBOT_WIDTH_METERS * 0.98

    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            rob_a = robots[i]
            rob_b = robots[j]

            dx = rob_b.x - rob_a.x
            dy = rob_b.y - rob_a.y
            dist = math.hypot(dx, dy)

            if 0.0001 < dist < bumper_clearance_limit:
                overlap = bumper_clearance_limit - dist
                normal_x = dx / dist
                normal_y = dy / dist

                # Positional separation
                rob_a.x -= normal_x * (overlap * 0.5)
                rob_a.y -= normal_y * (overlap * 0.5)
                rob_b.x += normal_x * (overlap * 0.5)
                rob_b.y += normal_y * (overlap * 0.5)

                # Velocity dampening along normal
                relative_vx = rob_b.vx - rob_a.vx
                relative_vy = rob_b.vy - rob_a.vy
                velocity_along_normal = relative_vx * normal_x + relative_vy * normal_y

                if velocity_along_normal < 0:
                    impulse = -1.15 * velocity_along_normal
                    rob_a.vx -= normal_x * (impulse * 0.5)
                    rob_a.vy -= normal_y * (impulse * 0.5)
                    rob_b.vx += normal_x * (impulse * 0.5)
                    rob_b.vy += normal_y * (impulse * 0.5)


# ==============================================================================
# 6. PATHPLANNER TRAJECTORY ENGINE
# ==============================================================================

@dataclass
class TrajectoryPoint:
    """A sampled trajectory state at time t."""
    time: float
    x: float
    y: float
    velocity: float
    heading: float


class PathTrajectory:
    """Parses .path files and provides time-parameterized trajectory sampling."""

    def __init__(self, path_file: Path) -> None:
        self.path_file: Path = path_file
        self.points: List[TrajectoryPoint] = []
        self.total_duration: float = 0.0
        self.initial_heading: float = 0.0
        self._generate_trajectory()

    def _generate_trajectory(self) -> None:
        """Parse PathPlanner JSON and compute time-parameterized velocity profile."""
        with open(self.path_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        waypoints = data.get("waypoints", [])
        if len(waypoints) < 2:
            return

        # Parse global constraints
        global_constraints = data.get("globalConstraints", {})
        max_vel = global_constraints.get("maxVelocity", MAX_LINEAR_VELOCITY)
        max_acc = global_constraints.get("maxAcceleration", MAX_LINEAR_ACCELERATION)

        # Parse target rotations
        rot_targets = data.get("rotationTargets", [])
        ideal_start_rot = data.get("idealStartingState", {}).get("rotation", 0.0)
        self.initial_heading = math.radians(ideal_start_rot)

        # Generate dense geometric curve samples
        dense_positions: List[Tuple[float, float]] = []
        dense_headings: List[float] = []

        num_segments = len(waypoints) - 1
        samples_per_seg = 40

        for seg_idx in range(num_segments):
            w0 = waypoints[seg_idx]
            w1 = waypoints[seg_idx + 1]

            p0 = (w0["anchor"]["x"], w0["anchor"]["y"])
            p1 = (w0["nextControl"]["x"], w0["nextControl"]["y"]) if w0.get("nextControl") else p0
            p3 = (w1["anchor"]["x"], w1["anchor"]["y"])
            p2 = (w1["prevControl"]["x"], w1["prevControl"]["y"]) if w1.get("prevControl") else p3

            for s in range(samples_per_seg if seg_idx < num_segments - 1 else samples_per_seg + 1):
                t = s / float(samples_per_seg)
                pos, vel, _ = compute_cubic_bezier(p0, p1, p2, p3, t)
                dense_positions.append(pos)

                # Segment progress
                seg_progress = seg_idx + t
                target_head = self.initial_heading

                # Check rotation targets
                for rt in rot_targets:
                    waypoint_rel_pos = rt.get("waypointRelativePos", 0.0)
                    if seg_progress >= waypoint_rel_pos:
                        target_head = math.radians(rt.get("rotationDegrees", 0.0))

                dense_headings.append(target_head)

        if not dense_positions:
            return

        # Compute cumulative distance along curve
        distances: List[float] = [0.0]
        for idx in range(1, len(dense_positions)):
            dx = dense_positions[idx][0] - dense_positions[idx - 1][0]
            dy = dense_positions[idx][1] - dense_positions[idx - 1][1]
            distances.append(distances[-1] + math.hypot(dx, dy))

        total_length = distances[-1]

        # Forward-backward acceleration profile passes
        speeds = [max_vel] * len(dense_positions)
        speeds[0] = 0.0
        speeds[-1] = 0.0

        # Forward pass (acceleration limit)
        for i in range(len(dense_positions) - 1):
            ds = distances[i + 1] - distances[i]
            if ds > 0:
                speeds[i + 1] = min(speeds[i + 1], math.sqrt(speeds[i] ** 2 + 2.0 * max_acc * ds))

        # Backward pass (deceleration limit)
        for i in range(len(dense_positions) - 1, 0, -1):
            ds = distances[i] - distances[i - 1]
            if ds > 0:
                speeds[i - 1] = min(speeds[i - 1], math.sqrt(speeds[i] ** 2 + 2.0 * max_acc * ds))

        # Integrate time profile
        time_elapsed = 0.0
        self.points.append(TrajectoryPoint(0.0, dense_positions[0][0], dense_positions[0][1], 0.0, dense_headings[0]))

        for i in range(len(dense_positions) - 1):
            ds = distances[i + 1] - distances[i]
            avg_v = max(0.1, (speeds[i] + speeds[i + 1]) / 2.0)
            dt_step = ds / avg_v
            time_elapsed += dt_step
            self.points.append(
                TrajectoryPoint(
                    time_elapsed,
                    dense_positions[i + 1][0],
                    dense_positions[i + 1][1],
                    speeds[i + 1],
                    dense_headings[i + 1],
                )
            )

        self.total_duration = time_elapsed

    def sample(self, current_time: float) -> TrajectoryPoint:
        """Sample the trajectory at an elapsed time in seconds."""
        if not self.points:
            return TrajectoryPoint(0.0, 0.0, 0.0, 0.0, 0.0)

        if current_time <= 0.0:
            return self.points[0]

        if current_time >= self.total_duration:
            return self.points[-1]

        times = [pt.time for pt in self.points]
        idx = bisect.bisect_left(times, current_time)

        pt_a = self.points[idx - 1]
        pt_b = self.points[idx]
        span = pt_b.time - pt_a.time

        if span <= 0.0001:
            return pt_a

        alpha = (current_time - pt_a.time) / span
        interp_x = pt_a.x + alpha * (pt_b.x - pt_a.x)
        interp_y = pt_a.y + alpha * (pt_b.y - pt_a.y)
        interp_v = pt_a.velocity + alpha * (pt_b.velocity - pt_a.velocity)

        diff_h = angle_difference(pt_b.heading, pt_a.heading)
        interp_h = normalize_angle(pt_a.heading + alpha * diff_h)

        return TrajectoryPoint(current_time, interp_x, interp_y, interp_v, interp_h)


# ==============================================================================
# 7. AUTONOMOUS ROUTINE ENGINE
# ==============================================================================

class AutonomousEngine:
    """Executes PathPlanner autonomous sequences using Holonomic Drive PID."""

    def __init__(self, robot: Robot, paths_directory: Path) -> None:
        self.robot: Robot = robot
        self.paths_dir: Path = paths_directory
        self.trajectories: List[Tuple[str, PathTrajectory]] = []
        self.current_traj_idx: int = 0
        self.elapsed_time: float = 0.0
        self.is_finished: bool = False
        self.is_active: bool = False

        # Holonomic Tracking PID Gains
        self.kp_translation: float = 4.8
        self.kp_rotation: float = 5.2

    def load_auto_file(self, auto_path: Path) -> bool:
        """Load an autonomous routine file and all referenced .path files."""
        self.trajectories.clear()
        self.current_traj_idx = 0
        self.elapsed_time = 0.0
        self.is_finished = False

        if not auto_path.exists():
            return False

        with open(auto_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        command_tree = data.get("command", {})

        def extract_path_names(cmd_node: dict) -> List[str]:
            names = []
            if cmd_node.get("type") == "path":
                p_name = cmd_node.get("data", {}).get("pathName")
                if p_name:
                    names.append(p_name)
            for child in cmd_node.get("data", {}).get("commands", []):
                names.extend(extract_path_names(child))
            return names

        path_names = extract_path_names(command_tree)
        for name in path_names:
            p_file = self.paths_dir / f"{name}.path"
            if p_file.exists():
                self.trajectories.append((name, PathTrajectory(p_file)))

        if self.trajectories:
            first_traj = self.trajectories[0][1]
            if first_traj.points:
                self.robot.x = first_traj.points[0].x
                self.robot.y = first_traj.points[0].y
                self.robot.heading = first_traj.points[0].heading

        return len(self.trajectories) > 0

    def start(self) -> None:
        """Start or restart the loaded autonomous sequence."""
        self.current_traj_idx = 0
        self.elapsed_time = 0.0
        self.is_finished = False
        self.is_active = True

    def update(self, dt: float) -> None:
        """Step the autonomous trajectory execution and apply motor drive speeds."""
        if not self.is_active or self.is_finished or not self.trajectories:
            self.robot.vx = 0.0
            self.robot.vy = 0.0
            self.robot.omega = 0.0
            return

        name, traj = self.trajectories[self.current_traj_idx]
        self.elapsed_time += dt

        target_pt = traj.sample(self.elapsed_time)

        # Translation error (Field coordinates)
        err_x = target_pt.x - self.robot.x
        err_y = target_pt.y - self.robot.y
        err_heading = angle_difference(target_pt.heading, self.robot.heading)

        # Holonomic PID speed generation
        calc_vx = self.kp_translation * err_x
        calc_vy = self.kp_translation * err_y
        calc_omega = self.kp_rotation * err_heading

        # Clamp to max dynamics
        lin_speed = math.hypot(calc_vx, calc_vy)
        if lin_speed > MAX_LINEAR_VELOCITY:
            calc_vx = (calc_vx / lin_speed) * MAX_LINEAR_VELOCITY
            calc_vy = (calc_vy / lin_speed) * MAX_LINEAR_VELOCITY

        self.robot.vx = calc_vx
        self.robot.vy = calc_vy
        self.robot.omega = max(-MAX_ANGULAR_VELOCITY, min(MAX_ANGULAR_VELOCITY, calc_omega))

        # Check trajectory completion
        if self.elapsed_time >= traj.total_duration:
            self.current_traj_idx += 1
            self.elapsed_time = 0.0
            if self.current_traj_idx >= len(self.trajectories):
                self.is_finished = True
                self.is_active = False
                self.robot.vx = 0.0
                self.robot.vy = 0.0
                self.robot.omega = 0.0


# ==============================================================================
# 8. FIELD & ENVIRONMENT STATE
# ==============================================================================

class Field:
    """Maintains field objects: balls, spatial hash grid, and collision logic."""

    def __init__(self) -> None:
        self.balls: List[Ball] = []
        self.projectiles: List[Projectile] = []
        self.spatial_grid_cell_size: float = BALL_DIAMETER_METERS
        self.reset()

    def reset(self) -> None:
        """Reset balls on the field to standard competition starting positions."""
        self.balls.clear()
        self.projectiles.clear()

        # 1. Neutral Zone: 30 ft x 12 ft centered, 18 rows x 20 cols = 360 balls
        rows = 18
        cols = 20
        span_x = NEUTRAL_ZONE_MAX_X - NEUTRAL_ZONE_MIN_X
        span_y = NEUTRAL_ZONE_MAX_Y - NEUTRAL_ZONE_MIN_Y

        for r in range(rows):
            y = NEUTRAL_ZONE_MIN_Y + (r + 0.5) * (span_y / rows)
            stagger = (BALL_RADIUS_METERS * 0.5) if (r % 2 == 1) else 0.0
            for c in range(cols):
                x = NEUTRAL_ZONE_MIN_X + (c + 0.5) * (span_x / cols) + stagger
                if NEUTRAL_ZONE_MIN_X <= x <= NEUTRAL_ZONE_MAX_X and NEUTRAL_ZONE_MIN_Y <= y <= NEUTRAL_ZONE_MAX_Y:
                    self.balls.append(Ball(x, y))

        # 2. Blue Alliance Depot: 4 rows x 6 cols = 24 balls
        depot_span_x = BLUE_DEPOT_MAX_X - BLUE_DEPOT_MIN_X
        depot_span_y = BLUE_DEPOT_MAX_Y - BLUE_DEPOT_MIN_Y
        for r in range(4):
            y = BLUE_DEPOT_MIN_Y + (r + 0.5) * (depot_span_y / 4.0)
            for c in range(6):
                x = BLUE_DEPOT_MIN_X + (c + 0.5) * (depot_span_x / 6.0)
                self.balls.append(Ball(x, y))

        # 3. Red Alliance Depot: 4 rows x 6 cols = 24 balls
        red_span_x = RED_DEPOT_MAX_X - RED_DEPOT_MIN_X
        red_span_y = RED_DEPOT_MAX_Y - RED_DEPOT_MIN_Y
        for r in range(4):
            y = RED_DEPOT_MIN_Y + (r + 0.5) * (red_span_y / 4.0)
            for c in range(6):
                x = RED_DEPOT_MIN_X + (c + 0.5) * (red_span_x / 6.0)
                self.balls.append(Ball(x, y))

    def update(self, dt: float, robots: List[Robot]) -> None:
        """Update ball physics, collisions, intake processing, and projectiles."""
        # 1. Update loose balls
        for b in self.balls:
            b.update(dt)

        # 2. Ball-Ball Collision Detection via Spatial Hash Grid
        grid: Dict[Tuple[int, int], List[Ball]] = {}
        for b in self.balls:
            if b.is_collected:
                continue
            cx = int(b.x / self.spatial_grid_cell_size)
            cy = int(b.y / self.spatial_grid_cell_size)
            grid.setdefault((cx, cy), []).append(b)

        min_ball_distance = BALL_DIAMETER_METERS
        for (cx, cy), cell_balls in grid.items():
            neighbor_cells = [
                (cx, cy),
                (cx + 1, cy),
                (cx - 1, cy + 1),
                (cx, cy + 1),
                (cx + 1, cy + 1),
            ]
            for ncx, ncy in neighbor_cells:
                if (ncx, ncy) not in grid:
                    continue
                other_balls = grid[(ncx, ncy)]
                is_same_cell = (ncx == cx and ncy == cy)

                for i, b1 in enumerate(cell_balls):
                    start_j = (i + 1) if is_same_cell else 0
                    for j in range(start_j, len(other_balls)):
                        b2 = other_balls[j]
                        dx = b2.x - b1.x
                        dy = b2.y - b1.y
                        dist = math.hypot(dx, dy)

                        if 0.0001 < dist < min_ball_distance:
                            overlap = min_ball_distance - dist
                            nx = dx / dist
                            ny = dy / dist

                            # Zero-clipping separation
                            b1.x -= nx * (overlap * 0.5)
                            b1.y -= ny * (overlap * 0.5)
                            b2.x += nx * (overlap * 0.5)
                            b2.y += ny * (overlap * 0.5)

                            # Normal impulse exchange
                            rvx = b2.vx - b1.vx
                            rvy = b2.vy - b1.vy
                            vel_norm = rvx * nx + rvy * ny
                            if vel_norm < 0:
                                impulse = -0.75 * vel_norm
                                b1.vx -= nx * (impulse * 0.5)
                                b1.vy -= ny * (impulse * 0.5)
                                b2.vx += nx * (impulse * 0.5)
                                b2.vy += ny * (impulse * 0.5)

        # 3. Robot-Ball Interaction (Intake Zone & Bumper Deflection)
        for rob in robots:
            cos_h = math.cos(rob.heading)
            sin_h = math.sin(rob.heading)

            # World position of intake zone
            intake_center_wx = rob.x + (INTAKE_OFFSET_X * cos_h - INTAKE_OFFSET_Y * sin_h)
            intake_center_wy = rob.y + (INTAKE_OFFSET_X * sin_h + INTAKE_OFFSET_Y * cos_h)

            for b in self.balls:
                if b.is_collected:
                    continue

                # Transform to robot local coordinates
                rel_x = b.x - rob.x
                rel_y = b.y - rob.y
                local_x = rel_x * cos_h + rel_y * sin_h
                local_y = -rel_x * sin_h + rel_y * cos_h

                # Check Intake Zone
                in_intake_x = (INTAKE_OFFSET_X - INTAKE_LENGTH / 2.0) <= local_x <= (INTAKE_OFFSET_X + INTAKE_LENGTH / 2.0)
                in_intake_y = -INTAKE_WIDTH / 2.0 <= local_y <= INTAKE_WIDTH / 2.0

                if in_intake_x and in_intake_y and rob.intake_deployed:
                    if rob.hopper_balls < MAX_HOPPER_CAPACITY:
                        # Pull ball inward with progressive roller speed
                        b.intake_progress += (INTAKE_SURFACE_SPEED_MPS / INTAKE_LENGTH) * dt
                        pull_speed = INTAKE_SURFACE_SPEED_MPS * 0.85
                        b.vx = -cos_h * pull_speed
                        b.vy = -sin_h * pull_speed

                        if b.intake_progress >= 1.0:
                            b.is_collected = True
                            rob.hopper_balls += 1
                        continue
                    else:
                        # Hopper is full: push ball out of the way
                        push_mag = 1.8
                        b.vx = cos_h * push_mag
                        b.vy = sin_h * push_mag

                # Check Bumper Collision Deflection
                half_w = ROBOT_WIDTH_METERS / 2.0
                half_l = ROBOT_LENGTH_METERS / 2.0
                buffer = BALL_RADIUS_METERS

                if (-half_l - buffer) <= local_x <= (half_l + buffer) and (-half_w - buffer) <= local_y <= (half_w + buffer):
                    # Clamp point on robot bounding box
                    closest_lx = max(-half_l, min(half_l, local_x))
                    closest_ly = max(-half_w, min(half_w, local_y))

                    dlx = local_x - closest_lx
                    dly = local_y - closest_ly
                    dist_to_box = math.hypot(dlx, dly)

                    if dist_to_box < buffer:
                        push_dist = buffer - max(0.001, dist_to_box)
                        norm_lx = (dlx / dist_to_box) if dist_to_box > 0.0001 else 1.0
                        norm_ly = (dly / dist_to_box) if dist_to_box > 0.0001 else 0.0

                        # Transform normal back to world coordinates
                        norm_wx = norm_lx * cos_h - norm_ly * sin_h
                        norm_wy = norm_lx * sin_h + norm_ly * cos_h

                        b.x += norm_wx * push_dist
                        b.y += norm_wy * push_dist

                        # Transfer robot momentum
                        rob_speed = math.hypot(rob.vx, rob.vy)
                        impact_speed = max(1.2, rob_speed * 1.25)
                        b.vx = norm_wx * impact_speed
                        b.vy = norm_wy * impact_speed

        # 4. Projectile Trajectory Updates
        active_projectiles = []
        for p in self.projectiles:
            p.update(dt)
            if p.scored:
                for rob in robots:
                    if math.hypot(p.target_x - rob.target_hub_pos[0], p.target_y - rob.target_hub_pos[1]) < 0.1:
                        rob.score_count += 1
            if p.is_active:
                active_projectiles.append(p)
        self.projectiles = active_projectiles


# ==============================================================================
# 9. INTERACTIVE TKINTER SIMULATION APPLICATION
# ==============================================================================

class SimulationApp:
    """Tkinter graphical desktop interface for multi-robot simulation and visualization."""

    def __init__(self, root: tk.Tk, paths_dir: Path, autos_dir: Path) -> None:
        self.root: tk.Tk = root
        self.root.title("FRC 2026 2D Multi-Robot Simulation")
        self.paths_dir: Path = paths_dir
        self.autos_dir: Path = autos_dir

        self.field: Field = Field()
        self.robots: List[Robot] = []
        self.engines: List[AutonomousEngine] = []

        # Display parameters
        self.pixels_per_meter: float = 64.0
        self.canvas_width: int = int(FIELD_LENGTH_METERS * self.pixels_per_meter)
        self.canvas_height: int = int(FIELD_WIDTH_METERS * self.pixels_per_meter)

        self.is_paused: bool = False
        self.simulation_speed: float = 1.0
        self.last_tick_time: float = time.time()

        self._initialize_robots()
        self._build_user_interface()
        self._schedule_tick()

    def _initialize_robots(self) -> None:
        """Create the four competition robots (2 Blue, 2 Red) and their auto engines."""
        configs = [
            ("Blue Left", "blue", 1.85, 6.25, 0.0),
            ("Blue Right", "blue", 1.85, 2.15, 0.0),
            ("Red Left", "red", 14.65, 2.15, math.pi),
            ("Red Right", "red", 14.65, 6.25, math.pi),
        ]
        self.robots.clear()
        self.engines.clear()

        for name, alliance, ix, iy, ih in configs:
            robot = Robot(name, alliance, ix, iy, ih)
            engine = AutonomousEngine(robot, self.paths_dir)
            self.robots.append(robot)
            self.engines.append(engine)

    def _build_user_interface(self) -> None:
        """Build the GUI layout: canvas on the left, control sidebar on the right."""
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Simulation Canvas
        self.canvas = tk.Canvas(
            main_frame,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="#18181b",
            highlightthickness=1,
            highlightbackground="#3f3f46",
        )
        self.canvas.pack(side=tk.LEFT, padx=5, pady=5)

        # Control Panel
        sidebar = ttk.Frame(main_frame, width=320)
        sidebar.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Master Controls
        btn_frame = ttk.LabelFrame(sidebar, text="Simulation Controls")
        btn_frame.pack(fill=tk.X, pady=4)

        self.btn_run_all = ttk.Button(btn_frame, text="Run All Autos", command=self.run_all_autos)
        self.btn_run_all.pack(fill=tk.X, padx=4, pady=2)

        self.btn_pause = ttk.Button(btn_frame, text="Pause", command=self.toggle_pause)
        self.btn_pause.pack(fill=tk.X, padx=4, pady=2)

        self.btn_reset = ttk.Button(btn_frame, text="Reset Sim", command=self.reset_simulation)
        self.btn_reset.pack(fill=tk.X, padx=4, pady=2)

        # Autonomous Routine Selectors
        autos_frame = ttk.LabelFrame(sidebar, text="Robot Routine Selection")
        autos_frame.pack(fill=tk.X, pady=6)

        available_autos = ["Stay Still"]
        if self.autos_dir.exists():
            for f in sorted(self.autos_dir.glob("*.auto")):
                available_autos.append(f.stem)

        self.auto_vars: List[tk.StringVar] = []
        for idx, rob in enumerate(self.robots):
            row = ttk.Frame(autos_frame)
            row.pack(fill=tk.X, padx=4, pady=2)

            lbl = ttk.Label(row, text=f"{rob.name}:", width=12)
            lbl.pack(side=tk.LEFT)

            var = tk.StringVar(value=available_autos[1] if len(available_autos) > 1 else "Stay Still")
            self.auto_vars.append(var)

            combo = ttk.Combobox(row, textvariable=var, values=available_autos, state="readonly", width=18)
            combo.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Telemetry Display
        telemetry_frame = ttk.LabelFrame(sidebar, text="Robot Telemetry")
        telemetry_frame.pack(fill=tk.BOTH, expand=True, pady=6)

        self.telemetry_labels: List[ttk.Label] = []
        for rob in self.robots:
            card = ttk.Frame(telemetry_frame, relief=tk.GROOVE, padding=4)
            card.pack(fill=tk.X, padx=4, pady=3)

            name_lbl = ttk.Label(
                card,
                text=rob.name,
                font=("Segoe UI", 9, "bold"),
                foreground="#38bdf8" if rob.alliance == "blue" else "#f87171",
            )
            name_lbl.pack(anchor=tk.W)

            stats_lbl = ttk.Label(card, text="", font=("Consolas", 8))
            stats_lbl.pack(anchor=tk.W)
            self.telemetry_labels.append(stats_lbl)

    def run_all_autos(self) -> None:
        """Start auto routines for all robots based on combo box selections."""
        for idx, (rob, eng) in enumerate(zip(self.robots, self.engines)):
            selected_mode = self.auto_vars[idx].get()
            if selected_mode == "Stay Still":
                rob.auto_mode = "still"
                eng.is_active = False
            else:
                rob.auto_mode = "auto"
                auto_file = self.autos_dir / f"{selected_mode}.auto"
                if auto_file.exists():
                    eng.load_auto_file(auto_file)
                    eng.start()

    def toggle_pause(self) -> None:
        """Toggle paused state."""
        self.is_paused = not self.is_paused
        self.btn_pause.config(text="Resume" if self.is_paused else "Pause")

    def reset_simulation(self) -> None:
        """Reset field, robots, and engines to initial state."""
        self.field.reset()
        self._initialize_robots()
        for idx, rob in enumerate(self.robots):
            rob.auto_mode = "still"
            self.auto_vars[idx].set("Stay Still")

    def _schedule_tick(self) -> None:
        """Schedule next animation step (approx 60 Hz)."""
        self._tick()
        self.root.after(16, self._schedule_tick)

    def _tick(self) -> None:
        """Simulation physics step and visual rendering."""
        current_time = time.time()
        dt = min(0.05, current_time - self.last_tick_time) * self.simulation_speed
        self.last_tick_time = current_time

        if not self.is_paused:
            # Step auto engines
            for eng in self.engines:
                eng.update(dt)

            # Step robot dynamics
            for rob in self.robots:
                rob.update(dt)

            # Multi-robot bumper collision handling
            resolve_robot_collisions(self.robots)

            # Auto-fire when AimBot has lock
            for rob in self.robots:
                if rob.aimbot_locked and rob.hopper_balls > 0:
                    rob.is_shooting = True
                    proj = rob.shoot()
                    if proj:
                        self.field.projectiles.append(proj)
                else:
                    rob.is_shooting = False

            # Step field physics (balls, collisions, intake)
            self.field.update(dt, self.robots)

        # Draw visuals & update dashboard
        self._render()
        self._update_telemetry()

    def _to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        """Convert field coordinates (meters) to canvas pixel coordinates."""
        cx = x * self.pixels_per_meter
        cy = self.canvas_height - (y * self.pixels_per_meter)
        return cx, cy

    def _render(self) -> None:
        """Render field zones, balls, projectiles, and robots onto the Tkinter canvas."""
        self.canvas.delete("all")

        # 1. Field Zones
        # Neutral Zone
        nz_x0, nz_y1 = self._to_canvas(NEUTRAL_ZONE_MIN_X, NEUTRAL_ZONE_MAX_Y)
        nz_x1, nz_y0 = self._to_canvas(NEUTRAL_ZONE_MAX_X, NEUTRAL_ZONE_MIN_Y)
        self.canvas.create_rectangle(nz_x0, nz_y1, nz_x1, nz_y0, fill="#27272a", outline="#3f3f46", dash=(4, 4))
        self.canvas.create_text(
            (nz_x0 + nz_x1) / 2.0,
            nz_y1 + 12,
            text="NEUTRAL ZONE (30ft x 12ft)",
            fill="#71717a",
            font=("Segoe UI", 8),
        )

        # Alliance Hubs
        for hub_pos, color in [(BLUE_HUB_COORDINATES, "#3b82f6"), (RED_HUB_COORDINATES, "#ef4444")]:
            hx, hy = self._to_canvas(hub_pos[0], hub_pos[1])
            hr = 0.65 * self.pixels_per_meter
            self.canvas.create_oval(hx - hr, hy - hr, hx + hr, hy + hr, outline=color, width=2)
            self.canvas.create_text(hx, hy, text="HUB", fill=color, font=("Segoe UI", 8, "bold"))

        # Depots
        b_dp_x0, b_dp_y1 = self._to_canvas(BLUE_DEPOT_MIN_X, BLUE_DEPOT_MAX_Y)
        b_dp_x1, b_dp_y0 = self._to_canvas(BLUE_DEPOT_MAX_X, BLUE_DEPOT_MIN_Y)
        self.canvas.create_rectangle(b_dp_x0, b_dp_y1, b_dp_x1, b_dp_y0, outline="#1d4ed8", width=1)

        r_dp_x0, r_dp_y1 = self._to_canvas(RED_DEPOT_MIN_X, RED_DEPOT_MAX_Y)
        r_dp_x1, r_dp_y0 = self._to_canvas(RED_DEPOT_MAX_X, RED_DEPOT_MIN_Y)
        self.canvas.create_rectangle(r_dp_x0, r_dp_y1, r_dp_x1, r_dp_y0, outline="#b91c1c", width=1)

        # 2. Balls
        ball_px_rad = max(2.5, BALL_RADIUS_METERS * self.pixels_per_meter)
        for b in self.field.balls:
            if not b.is_collected:
                bx, by = self._to_canvas(b.x, b.y)
                self.canvas.create_oval(
                    bx - ball_px_rad,
                    by - ball_px_rad,
                    bx + ball_px_rad,
                    by + ball_px_rad,
                    fill="#facc15",
                    outline="#ca8a04",
                )

        # 3. Ballistic Projectiles
        for p in self.field.projectiles:
            px, py = self._to_canvas(p.x, p.y)
            proj_rad = max(2.0, (BALL_RADIUS_METERS + p.z * 0.04) * self.pixels_per_meter)
            self.canvas.create_oval(
                px - proj_rad,
                py - proj_rad,
                px + proj_rad,
                py + proj_rad,
                fill="#f97316",
                outline="#ea580c",
            )

        # 4. Robots
        for rob in self.robots:
            poly = rob.get_bumper_polygon()
            canvas_poly = []
            for px, py in poly:
                cx, cy = self._to_canvas(px, py)
                canvas_poly.extend([cx, cy])

            fill_color = "#1e3a8a" if rob.alliance == "blue" else "#7f1d1d"
            outline_color = "#60a5fa" if rob.alliance == "blue" else "#f87171"
            self.canvas.create_polygon(canvas_poly, fill=fill_color, outline=outline_color, width=2)

            # Robot center and heading indicator
            rx, ry = self._to_canvas(rob.x, rob.y)
            hx, hy = self._to_canvas(
                rob.x + math.cos(rob.heading) * 0.45,
                rob.y + math.sin(rob.heading) * 0.45,
            )
            self.canvas.create_line(rx, ry, hx, hy, fill="#ffffff", width=2, arrow=tk.LAST)

            # AimBot line to hub (dashed when locked)
            if rob.aimbot_locked:
                hub_cx, hub_cy = self._to_canvas(rob.target_hub_pos[0], rob.target_hub_pos[1])
                self.canvas.create_line(rx, ry, hub_cx, hub_cy, fill="#4ade80", dash=(2, 4), width=1)

            # Robot label
            self.canvas.create_text(
                rx,
                ry,
                text=f"{rob.name}\n[{rob.hopper_balls}]",
                fill="#ffffff",
                font=("Segoe UI", 7, "bold"),
                justify=tk.CENTER,
            )

    def _update_telemetry(self) -> None:
        """Update live status labels in the sidebar."""
        for idx, rob in enumerate(self.robots):
            stats_text = (
                f"Pose: ({rob.x:4.2f}, {rob.y:4.2f}, {math.degrees(rob.heading):4.0f} deg)\n"
                f"Vel: {math.hypot(rob.vx, rob.vy):4.2f} m/s  Omega: {rob.omega:4.2f} rad/s\n"
                f"Hopper: {rob.hopper_balls}/{MAX_HOPPER_CAPACITY}  Scored: {rob.score_count}\n"
                f"AimBot: {'LOCKED' if rob.aimbot_locked else 'TRACKING'} | RPM: {rob.flywheel_rpm:4.0f}"
            )
            self.telemetry_labels[idx].config(text=stats_text)


# ==============================================================================
# 10. ENTRYPOINT & ASSET DISCOVERY
# ==============================================================================

def find_pathplanner_directories() -> Tuple[Path, Path]:
    """Search for the pathplanner/paths and pathplanner/autos directories."""
    candidates = [
        Path.cwd() / "src/main/deploy/pathplanner",
        Path(__file__).resolve().parent / "src/main/deploy/pathplanner",
        Path(__file__).resolve().parent.parent / "src/main/deploy/pathplanner",
        Path(__file__).resolve().parent.parent.parent / "src/main/deploy/pathplanner",
    ]

    for base in candidates:
        paths_dir = base / "paths"
        autos_dir = base / "autos"
        if paths_dir.exists() and autos_dir.exists():
            return paths_dir, autos_dir

    # Fallback to local paths
    return Path("src/main/deploy/pathplanner/paths"), Path("src/main/deploy/pathplanner/autos")


def main() -> None:
    """Application entrypoint."""
    paths_dir, autos_dir = find_pathplanner_directories()
    root = tk.Tk()
    app = SimulationApp(root, paths_dir, autos_dir)
    root.mainloop()


if __name__ == "__main__":
    main()
