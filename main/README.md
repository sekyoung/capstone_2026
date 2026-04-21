# ros2rev1 — 5-DOF Arm ROS 2 Workspace

A 5-DOF serial robot arm prototype running on **ROS 2 Jazzy**. Inverse kinematics
is solved with **Pinocchio**'s damped least-squares Jacobian method and visualised
live in **RViz2**. No pip packages required — all dependencies ship with ROS 2.

---

## Robot Overview

The arm is defined in URDF as a 5-joint serial manipulator:

| Joint | Type | Axis | Limits | Role |
|-------|------|------|--------|------|
| `joint_1` | revolute | Z | ±180° | Base yaw |
| `joint_2` | revolute | X | ±100° | Shoulder pitch |
| `joint_3` | revolute | Z | ±180° | Forearm rotation |
| `joint_4` | revolute | X | ±120° | Wrist pitch |
| `joint_5` | revolute | Z | ±180° | Wrist rotation |

End-effector frame: `link_5`. TCP fixed frame: `tcp` (45 mm above `link_5`).

---

## Workspace Layout

```
ros2rev1/
├── src/
│   ├── ik_solver/               # IK logic + ROS 2 node
│   │   ├── ik_solver/
│   │   │   ├── config.py        # IKConfig dataclass — tune parameters here
│   │   │   ├── ik_solver.py     # Pinocchio damped-Jacobian IK (no ROS)
│   │   │   └── ik_node.py       # ROS 2 node wrapping IKSolver
│   │   └── urdf/
│   │       └── arm.urdf         # 5-DOF arm robot description
│   ├── test_sweep/              # Simulated circular target source
│   │   └── test_sweep/
│   │       └── sweep_node.py
│   ├── robot_launcher/          # Launch file
│   │   └── launch/
│   │       └── arm.launch.py    # Launches all nodes
│   └── rviz_viewer/             # RViz2 preset config
│       └── rviz/
│           └── arm.rviz
├── build/                       # colcon build output (git-ignored)
├── install/                     # colcon install output (git-ignored)
└── log/                         # colcon logs (git-ignored)
```

---

## Architecture

```
sweep_node  (test_sweep)
    │  /target_hand_pose  (geometry_msgs/PoseStamped)
    ▼
ik_node  (ik_solver)
    │  /joint_states  (sensor_msgs/JointState)
    ▼
robot_state_publisher  →  /tf  →  rviz2  (rviz_viewer)
```

Replace `sweep_node` with any node that publishes `geometry_msgs/PoseStamped`
on `/target_hand_pose` to drive the arm from a real source.

### Packages

#### `ik_solver`
Core inverse kinematics package.

- **`ik_solver.py`** — Pure-Python IK solver (no ROS). Loads the URDF via
  Pinocchio, holds joint state, and steps the damped least-squares solution:
  ```
  dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · (gain × err₆D)
  q  = integrate(model, q, dq · dt)
  ```
  where `err₆D` is the SE3 log error and `λ` is the damping factor.
- **`ik_node.py`** — ROS 2 node wrapping `IKSolver`.
  - Subscribes: `/target_hand_pose` (`geometry_msgs/PoseStamped`)
  - Publishes: `/joint_states` (`sensor_msgs/JointState`)
  - Parameters: `urdf_path`, `ik_rate_hz` (default `100.0` Hz)
- **`config.py`** — `IKConfig` dataclass with all tunable parameters.

#### `test_sweep`
Simulated target source. Publishes a smooth circular trajectory in the XY plane
at a fixed height.

- Publishes: `/target_hand_pose` (`geometry_msgs/PoseStamped`)
- Parameters:

  | Name | Default | Description |
  |------|---------|-------------|
  | `radius` | `0.08` m | Circle radius |
  | `height` | `0.18` m | Fixed Z height |
  | `speed`  | `1.0`   | Angular speed multiplier |
  | `rate_hz`| `30.0`  | Publish rate |

#### `robot_launcher`
Contains only `arm.launch.py`. Starts `robot_state_publisher`, `sweep_node`,
`ik_node`, and `rviz2` together.

Launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `ik_rate_hz`    | `100.0` | IK solve + joint-state publish rate |
| `sweep_radius`  | `0.08`  | Sweep circle radius (metres) |
| `sweep_speed`   | `1.0`   | Sweep angular speed multiplier |

#### `rviz_viewer`
Holds the `arm.rviz` RViz2 config. Loaded automatically by the launch file.

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy |
| pinocchio | ships with Jazzy |

Install pinocchio:
```bash
sudo apt install ros-jazzy-pinocchio
```

Verify:
```bash
source /opt/ros/jazzy/setup.bash
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

---

## Getting Started

### 1. Clone

```bash
git clone <repo-url> ros2rev1
cd ros2rev1
```

### 2. Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

`--symlink-install` lets you edit Python source files without rebuilding.
Rebuild only when entry points, `package.xml`, or `setup.py` change.

### 3. Source the overlay

```bash
source install/setup.bash
```

To avoid repeating this each session:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2rev1/install/setup.bash" >> ~/.bashrc
```

### 4. Launch

```bash
ros2 launch robot_launcher arm.launch.py
```

RViz2 opens automatically showing the arm tracking the circular sweep.

**Optional overrides:**
```bash
ros2 launch robot_launcher arm.launch.py sweep_radius:=0.12 sweep_speed:=0.5 ik_rate_hz:=50.0
```

---

## Tuning IK Parameters

All IK parameters live in `src/ik_solver/ik_solver/config.py`:

```python
@dataclass
class IKConfig:
    end_effector_link: str   = 'link_5'   # TCP frame
    safe_reach_fraction: float = 0.98     # clamp target to 98% of max reach
    deadzone_threshold: float  = 1e-6     # skip micro-updates below this Δq
    min_dt: float              = 0.001    # minimum timestep
    ik_damping: float          = 1e-2     # λ in damped least-squares
    ik_gain: float             = 8.0      # proportional gain on SE3 error
    ik_rate_hz: float          = 100.0    # IK loop rate
```

Increase `ik_damping` if joints oscillate near singularities; decrease for
sharper tracking away from singularities.

---

## Replacing the Sweep with a Real Source

Any node that publishes `geometry_msgs/PoseStamped` on `/target_hand_pose`
drives the arm. To test manually:

```bash
ros2 topic pub /target_hand_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.15, y: 0.0, z: 0.18}, orientation: {w: 1.0}}}"
```

---

## Useful Commands

```bash
# Watch live joint angles
ros2 topic echo /joint_states

# Watch live target pose
ros2 topic echo /target_hand_pose

# Node graph
ros2 run rqt_graph rqt_graph

# TF tree
ros2 run tf2_tools view_frames
```

---

## License

MIT
