"""
config.py — central place for all tunable parameters.
Edit here; nothing else needs to change.
"""
from dataclasses import dataclass, field
from typing import Dict


@dataclass
class CommConfig:
    mode: str = "SHM"           # "SHM" | "UDP"
    ip: str = "127.0.0.1"
    ik_recv_port: int = 10001   # Unity → IK
    ik_send_port: int = 10002   # IK → Unity
    hw_recv_port: int = 9998    # Unity → HW
    hw_send_port: int = 9997    # HW → Unity

    shm_target_name: str = "UnityIKTarget"
    shm_ik_reply_name: str = "UnityIKReply"
    shm_motor_cmd_name: str = "UnityMotorCommand"
    shm_motor_status_name: str = "UnityMotorStatus"

    shm_target_size: int = 512
    shm_reply_size: int = 512
    shm_motor_size: int = 384   # 32 motors × 12 bytes


@dataclass
class HardwareConfig:
    port: str = "COM3"
    baudrate: int = 1_000_000
    operating_mode: int = 5     # current-based position control
    target_hz: float = 1000.0
    max_motors: int = 32
    data_stride: int = 12       # bytes per motor slot (pos+tau+vel floats)
    limit_torque_nm: float = 1.0
    vel_est_mode: str = "window"
    vel_window_size: int = 5
    vel_ema_alpha: float = 0.6

    # motor_id → model string
    actuators: Dict[int, str] = field(default_factory=lambda: {
        2: "XM430-W210",
        3: "XH430-W210",
        4: "XC330-T181",
        5: "XC330-T181",
        6: "XC330-T181",
    })


@dataclass
class IKConfig:
    urdf_path: str = "robot_arm/urdf/arm.urdf"
    end_effector_link: str = "link_5"
    elbow_joint_name: str = "joint_3"
    safe_reach_fraction: float = 0.98
    deadzone_threshold: float = 0.001
    min_dt: float = 0.001

    # task costs
    position_cost: float = 1.0
    orientation_cost: float = 0.5
    elbow_position_cost: float = 0.1
    posture_cost: float = 0.01

    ik_solver: str = "osqp"
    ik_damping: float = 1e-2

    # motor command packing
    motor_ids: list = field(default_factory=lambda: [2, 3, 4, 5, 6])
    default_target_torque: float = 1.0
    default_target_velocity: float = 0.0
