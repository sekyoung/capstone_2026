"""
config.py — tunable parameters for the ROS 2 arm package.
No communication config — all transport is handled by ROS 2 topics.
"""
from dataclasses import dataclass, field


@dataclass
class IKConfig:
    # ── Kinematics ────────────────────────────────────────────────────
    end_effector_link: str = "link_5"
    elbow_joint_name: str = "joint_3"
    safe_reach_fraction: float = 0.98
    deadzone_threshold: float = 0.001
    min_dt: float = 0.001          # seconds; prevents division by zero on first tick

    # ── QP task weights ───────────────────────────────────────────────
    position_cost: float = 1.0
    orientation_cost: float = 0.5
    elbow_position_cost: float = 0.1
    posture_cost: float = 0.01

    # ── Solver ────────────────────────────────────────────────────────
    ik_solver: str = "osqp"
    ik_damping: float = 1e-2

    # ── Joint naming (must match URDF joint names) ────────────────────
    joint_names: list = field(default_factory=lambda: [
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
    ])

    # ── Loop rates ────────────────────────────────────────────────────
    ik_rate_hz: float = 100.0      # IK solve + publish rate
    sweep_rate_hz: float = 30.0    # test sweep publish rate
