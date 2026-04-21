# robot_arm_ros2 — ROS 2 Prototype

A minimal 5-DOF robot arm prototype that runs entirely on **ROS 2 Jazzy** with no
external pip packages. Inverse kinematics is solved using **Pinocchio**'s built-in
damped Jacobian method and the result is visualised live in **RViz2**.

This repo is structured as a standard ROS 2 Python workspace ready for `colcon build`.

---

## Robot Overview

The arm is a 5-DOF serial manipulator defined in URDF:

| Joint | Type | Axis | Limits | Role |
|-------|------|------|--------|------|
| `joint_1` | revolute | Z | ±180° | Base yaw |
| `joint_2` | revolute | X | ±100° | Shoulder pitch |
| `joint_3` | revolute | Z | ±180° | Forearm rotation |
| `joint_4` | revolute | X | ±120° | Wrist pitch |
| `joint_5` | revolute | Z | ±180° | Wrist rotation |

End-effector frame: `link_5` (TCP fixed frame: `tcp`, 45 mm above `link_5`).
Max reachable radius: ~274 mm.

---

## Architecture

```
sweep_node
    │  /target_hand_pose  (geometry_msgs/PoseStamped)
    ▼
ik_node  ──────────────────────────────────────────────────────────────────┐
    │  /joint_states  (sensor_msgs/JointState)                             │
    ▼                                                                      │
robot_state_publisher  →  /tf  →  rviz2                                   │
                                                                           │
    (replace sweep_node with a real tracking source to drive the arm) ◄───┘
```

### Nodes

#### `sweep_node`
Simulates a tracking source. Publishes a smooth circular sweep in the XY plane
at a fixed height so the IK node always has a moving target to follow.

- **Publishes:** `/target_hand_pose` (`geometry_msgs/PoseStamped`)
- **Parameters:**
  | Name | Default | Description |
  |------|---------|-------------|
  | `radius` | `0.08` m | Sweep circle radius |
  | `height` | `0.18` m | Fixed Z height of the target |
  | `speed`  | `1.0`  | Angular speed multiplier |
  | `rate_hz`| `30.0` | Publish rate |

#### `ik_node`
Runs the inverse kinematics solver at a fixed rate and publishes joint angles.

Uses **damped least-squares Jacobian IK** (Pinocchio):
```
dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · err₆D
q  = integrate(model, q, dq · dt)
```
where `err₆D` is the 6-D SE3 log error between the current and desired end-effector
poses, and `λ` is the damping factor (default `0.01`).

- **Subscribes:** `/target_hand_pose` (`geometry_msgs/PoseStamped`)
- **Publishes:** `/joint_states` (`sensor_msgs/JointState`)
- **Parameters:**
  | Name | Default | Description |
  |------|---------|-------------|
  | `urdf_path`  | — | Absolute path to `arm.urdf` (set by launch file) |
  | `ik_rate_hz` | `100.0` | IK solve + publish rate |

#### `robot_state_publisher` (system node)
Reads `/joint_states` + the URDF robot description and publishes `/tf` transforms
so RViz2 can render each link in the correct pose.

#### `rviz2` (system node)
Displays the robot using the preset config `rviz/arm.rviz`.

---

## Prerequisites

| Requirement | Version | Install |
|-------------|---------|---------|
| Ubuntu | 24.04 LTS | — |
| ROS 2 | Jazzy | [docs.ros.org/en/jazzy](https://docs.ros.org/en/jazzy/Installation.html) |
| pinocchio | ships with Jazzy | `sudo apt install ros-jazzy-pinocchio` |

> **No pip packages needed.** `pinocchio` is installed as a ROS 2 system package.

Verify pinocchio is available:
```bash
source /opt/ros/jazzy/setup.bash
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

---

## Getting Started

### 1. Clone the repo

```bash
git clone <repo-url> ros2rev
cd ros2rev
```

### 2. Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

`--symlink-install` means edits to Python source files take effect immediately
without rebuilding (you only need to rebuild when adding new entry points or
changing `package.xml` / `setup.py`).

### 3. Source the workspace overlay

```bash
source install/setup.bash
```

Add both lines to `~/.bashrc` to avoid repeating them each session:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2rev/install/setup.bash" >> ~/.bashrc
```

### 4. Launch

```bash
ros2 launch robot_arm_ros2 arm.launch.py
```

RViz2 opens automatically showing the arm sweeping in a circle.

**Optional overrides:**
```bash
# Slower sweep, half speed IK
ros2 launch robot_arm_ros2 arm.launch.py sweep_radius:=0.12 sweep_speed:=0.5 ik_rate_hz:=50.0
```

---

## Repository Layout

```
ros2rev/
├── src/
│   └── robot_arm_ros2/          # ROS 2 ament_python package
│       ├── robot_arm_ros2/      # Python module
│       │   ├── __init__.py
│       │   ├── config.py        # IKConfig dataclass — tune parameters here
│       │   ├── ik_solver.py     # Pinocchio damped-Jacobian IK (no ROS)
│       │   ├── ik_node.py       # ROS 2 node wrapping IKSolver
│       │   └── sweep_node.py    # Simulated target source (replace for real use)
│       ├── launch/
│       │   └── arm.launch.py    # Launches all nodes
│       ├── urdf/
│       │   └── arm.urdf         # 5-DOF arm robot description
│       ├── rviz/
│       │   └── arm.rviz         # RViz2 preset config
│       ├── resource/
│       │   └── robot_arm_ros2   # ament index marker (do not edit)
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── .gitignore
└── README.md
```

---

## Tuning Parameters

All IK parameters live in `robot_arm_ros2/config.py` as a dataclass:

```python
@dataclass
class IKConfig:
    end_effector_link: str  = 'link_5'     # frame used as TCP
    safe_reach_fraction: float = 0.98      # clamp target to 98% of max reach
    deadzone_threshold: float  = 0.001     # skip micro-updates below this Δq
    min_dt: float              = 0.001     # minimum timestep (avoids div-by-zero)
    ik_damping: float          = 1e-2      # λ in damped least-squares
    ik_rate_hz: float          = 100.0     # IK loop rate
```

Increase `ik_damping` if joints oscillate near singularities. Decrease it for
more precise tracking far from singularities.

---

## Replacing the Sweep with a Real Tracking Source

`sweep_node` is a drop-in placeholder. Any node that publishes
`geometry_msgs/PoseStamped` on `/target_hand_pose` will drive the arm:

```bash
# Disable sweep and publish a fixed target manually
ros2 topic pub /target_hand_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.15, y: 0.0, z: 0.18}, orientation: {w: 1.0}}}"
```

Future integrations (camera tracker, gamepad, etc.) only need to publish on
this topic — no changes to the IK or launch infrastructure.

---

## Useful Commands

```bash
# Live joint angles
ros2 topic echo /joint_states

# Live target pose
ros2 topic echo /target_hand_pose

# Node graph
ros2 node list
ros2 run rqt_graph rqt_graph

# Check TF tree
ros2 run tf2_tools view_frames
```

---

## Contributing

1. Fork and create a feature branch.
2. Keep each node self-contained — no shared global state.
3. New nodes must follow the same topic contract (`/target_hand_pose` in, `/joint_states` out).
4. Run a quick smoke test before pushing:
   ```bash
   colcon build --symlink-install && source install/setup.bash
   ros2 launch robot_arm_ros2 arm.launch.py
   ```

---

## License

MIT
