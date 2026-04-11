# Robot arm control system

Two independent processes connected through shared memory (Windows) or UDP (macOS / Linux).

```
Unity Engine
    │  hand/elbow pose (SHM: UnityIKTarget  or  UDP:10001)
    ▼
ik_process.py          Pinocchio + Pink QP-IK solver
    │  joint angles deg (SHM: UnityMotorCommand  or  UDP:9998)
    ▼
hw_process.py          1000 Hz Dynamixel sync read/write loop
    │  RS-485
    ▼
Motors  ID 2-6  (XM430 / XH430 / XC330)
```

## File layout

```
ik_process.py          IK process entry point
hw_process.py          Hardware process entry point
mock_dynamixel_sdk.py  Drop-in stub for testing without motors
dynamixel_motors.py    Motor HAL (unchanged)
robot_arm/
  __init__.py
  config.py            All tunable parameters in one place
  comm.py              SHM + UDP comm managers
  urdf/
    5DOF_robotArm.urdf     Robot kinematic model (unchanged)
```

## Configuration

Edit `robot_arm/config.py`.  All parameters live there — no need to edit
the process files for routine changes (port, motor IDs, task costs, etc.).

## Install dependencies

```bash
pip install pinocchio pink meshcat numpy dynamixel_sdk
```

---

## Dry-run — IK only (no hardware, no Unity)

```bash
# 1. Start IK process in UDP mode
python ik_process.py --comm UDP

# → opens Meshcat at http://localhost:7000 (check terminal for URL)
# → arm sits at zero pose waiting for tracking packets

# 2. In a second terminal, inject a fake tracking packet
python - <<'EOF'
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = json.dumps({
    "type": "tracking",
    "hand_pos":  [0.1, 0.2, 0.15],
    "hand_rot":  [1.0, 0.0, 0.0, 0.0],
    "elbow_pos": [0.05, 0.1, 0.1]
})
s.sendto(msg.encode(), ("127.0.0.1", 10001))
print("sent")
EOF

# → arm should move in the Meshcat window
```

## Dry-run — hardware process without motors

```bash
# Replace the dynamixel_sdk import in hw_process.py:
#
# Find this block near the top:
#   from dynamixel_sdk import (
#       PortHandler, PacketHandler, GroupSyncWrite, GroupFastSyncRead
#   )
#
# Replace it temporarily with:
#   from mock_dynamixel_sdk import (
#       PortHandler, PacketHandler, GroupSyncWrite, GroupFastSyncRead
#   )

python hw_process.py --comm UDP

# → prints motor init logs (MOCK)
# → enters 1000 Hz loop and prints Hz counter
# → accepts command packets on UDP port 9998
```

## Running both processes together (Windows, with Unity)

```bash
# Terminal 1
python ik_process.py

# Terminal 2
python hw_process.py

# Then press Play in Unity
```

## Running on macOS / Linux (UDP mode)

```bash
python ik_process.py --comm UDP
python hw_process.py --comm UDP --port /dev/tty.usbserial-XXXX
```

---

## IK task cost tuning

All in `IKConfig` in `config.py`:

| Parameter | Default | Effect |
|---|---|---|
| `position_cost` | 1.0 | How hard IK pulls end-effector to target |
| `orientation_cost` | 0.5 | How hard IK matches hand orientation |
| `elbow_position_cost` | 0.1 | Elbow hint weight — lower = softer |
| `posture_cost` | 0.01 | Pull back toward zero pose |
| `safe_reach_fraction` | 0.98 | Clamp target to this fraction of max reach |
| `deadzone_threshold` | 0.001 | Min joint change (rad) to trigger update |

## Adding a motor

In `config.py`, add the motor ID and model string to `HardwareConfig.actuators`:

```python
actuators: Dict[int, str] = field(default_factory=lambda: {
    2: "XM430-W210",
    3: "XH430-W210",
    ...
    7: "XM430-W350",   # new motor
})
```

Then add the motor ID to `IKConfig.motor_ids` so the IK process packs commands for it.

---

## Preparing for ROS 2 (next step)

When you're ready to add ROS 2:

- `comm.py` gains a `HWROS2Comm` and `IKROS2Comm` class that publish/subscribe
  to ROS 2 topics instead of SHM/UDP.
- `config.py` gains a `ROS2Config` dataclass for topic names and QoS.
- Both process entry points accept `--comm ROS2` via their CLI.
- No other files change.
