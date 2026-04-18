# robot_arm_ros2

5-DOF robot arm software stack built on ROS 2 (Jazzy).  
All inter-process communication is done via ROS 2 topics — no UDP, no sockets.

---

## Architecture

```
sweep_node
  │  /target_hand_pose   (geometry_msgs/PoseStamped)
  │  /target_elbow_point (geometry_msgs/PointStamped)
  ▼
ik_node  ──────────────────────────────────────────────────── /joint_states
                                                                     │
                                          robot_state_publisher ◄────┤  → /tf
                                          rviz2                 ◄────┘  (via /tf)
                                          meshcat_bridge        ◄────── /joint_states
                                          [hardware_node]       ◄────── /joint_states  (future)
```

### Nodes

| Node | Package | Role |
|------|---------|------|
| `sweep_node` | `robot_arm_ros2` | Simulated tracking source — publishes a circular sweep of target poses. Replace with a real tracking source when ready. |
| `ik_node` | `robot_arm_ros2` | QP inverse kinematics (Pinocchio + Pink). Subscribes to target poses, solves IK at 100 Hz, publishes `/joint_states`. |
| `meshcat_bridge` | `robot_arm_ros2` | Subscribes to `/joint_states`, renders the arm in a Meshcat web viewer. |
| `robot_state_publisher` | ROS 2 built-in | Reads the URDF + `/joint_states` → publishes `/tf` for RViz2. |
| `rviz2` | ROS 2 built-in | 3-D visualiser. Pre-configured to show Robot Model + TF, fixed frame = `world`. |

### Topics

| Topic | Type | Publisher → Subscriber |
|-------|------|------------------------|
| `/target_hand_pose` | `geometry_msgs/PoseStamped` | `sweep_node` → `ik_node` |
| `/target_elbow_point` | `geometry_msgs/PointStamped` | `sweep_node` → `ik_node` |
| `/joint_states` | `sensor_msgs/JointState` | `ik_node` → `robot_state_publisher`, `meshcat_bridge`, *(future hardware node)* |

---

## File layout

```
ROS2TEST/
├── robot_arm_ros2/
│   ├── config.py          — tunable parameters (IK weights, rates, joint names)
│   ├── ik_solver.py       — pure IK logic (no ROS), reusable
│   ├── ik_node.py         — ROS 2 node wrapping ik_solver
│   ├── sweep_node.py      — simulated sweep target publisher
│   └── meshcat_bridge.py  — /joint_states → Meshcat web viewer
├── urdf/
│   └── arm.urdf           — 5-DOF arm (box geometry, world root link)
├── launch/
│   └── arm.launch.py      — starts all nodes
├── rviz/
│   └── arm.rviz           — pre-configured RViz2 layout
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Requirements

| Dependency | How to get |
|------------|-----------|
| ROS 2 Jazzy | `/opt/ros/jazzy` — activate with `ros2init` alias |
| Python env | conda env `robot` — contains Pinocchio, Pink, Meshcat, OSQP |
| `colcon` | included with ROS 2 Jazzy |

---

## Setup (first time only)

```bash
# 1. Activate the conda env and source ROS 2
conda activate robot
ros2init

# 2. Build the package from inside ROS2TEST/
cd ~/Desktop/QP/ROS2TEST
colcon build --symlink-install

# 3. Source the install overlay
source install/setup.bash
```

> **Tip:** Add the last two lines to a shell alias or script so you don't have to repeat them every session.

---

## Running

```bash
conda activate robot && ros2init
cd ~/Desktop/QP/ROS2TEST
source install/setup.bash

ros2 launch robot_arm_ros2 arm.launch.py
```

This starts all five nodes.  
- **RViz2** opens automatically — you should see the arm sweeping in the 3-D view.  
- **Meshcat** opens at **http://127.0.0.1:7000/static/** — open that URL in any browser for the web viewer.

### Launch arguments

```bash
# Slower sweep, faster IK
ros2 launch robot_arm_ros2 arm.launch.py sweep_speed:=0.5 ik_rate_hz:=200.0

# Larger sweep circle
ros2 launch robot_arm_ros2 arm.launch.py sweep_radius:=0.12
```

| Argument | Default | Description |
|----------|---------|-------------|
| `ik_rate_hz` | `100.0` | IK solve + `/joint_states` publish rate (Hz) |
| `sweep_radius` | `0.08` | Sweep circle radius (metres) |
| `sweep_speed` | `1.0` | Angular speed multiplier |

---

## URDF notes

The URDF (`urdf/arm.urdf`) was updated from the original with two fixes:

1. **`world` root link + `world_to_base` fixed joint** — required by `robot_state_publisher` and RViz2.  Without a root fixed frame, TF cannot be published and the robot will not appear in RViz2.

2. **`tcp` tip link + `joint_tcp` fixed joint** — a named end-effector frame at the tip of `link_5`, useful for FK inspection and future tool attachment.

Kinematics are unchanged.

### Joint layout

```
world (fixed)
└── base_motor
    └── joint_1  [revolute, Z, ±180°]   base yaw
        └── link_1
            └── joint_2  [revolute, X, ±100°]   shoulder pitch
                └── link_2
                    └── joint_3  [revolute, Z, ±180°]   forearm rotation
                        └── link_3
                            └── joint_4  [revolute, X, ±120°]   wrist pitch
                                └── link_4
                                    └── joint_5  [revolute, Z, ±180°]   wrist rotation
                                        └── link_5
                                            └── tcp  (fixed tip frame)
```

---

## Adding a real tracking source

Replace `sweep_node` with any node that publishes the same topics:

```
/target_hand_pose    geometry_msgs/PoseStamped   — TCP target, frame_id="world"
/target_elbow_point  geometry_msgs/PointStamped  — elbow hint,  frame_id="world"
```

Positions must be in the robot's world frame (x-forward, y-left, z-up, REP 103).  
The `ik_node` will pick them up automatically — no other changes needed.

---

## Adding hardware control (future)

Create a new node that **subscribes to `/joint_states`**:

```python
from sensor_msgs.msg import JointState

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg: JointState):
        # msg.name     — list of joint names  e.g. ['joint_1', ..., 'joint_5']
        # msg.position — list of angles (radians)
        # send to motors here
        ...
```

Add the node to `arm.launch.py` alongside the existing nodes.  No changes to `ik_node` or `meshcat_bridge` are required.