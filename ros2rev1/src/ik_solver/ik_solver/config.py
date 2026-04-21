from dataclasses import dataclass, field


@dataclass
class IKConfig:
    end_effector_link: str  = 'link_5'
    safe_reach_fraction: float = 0.98
    deadzone_threshold: float  = 1e-6
    ik_gain: float             = 8.0
    min_dt: float              = 0.001

    # Damped least-squares regularisation
    ik_damping: float = 1e-2

    joint_names: list = field(default_factory=lambda: [
        'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5',
    ])

    ik_rate_hz: float   = 100.0
    sweep_rate_hz: float = 30.0
