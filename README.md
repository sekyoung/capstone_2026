# Robot arm control system

Two independent processes communicating over UDP.

```
Tracking source  ──JSON──►  port 10001  ──►  ik_process.py  ──►  Meshcat
                                                    │
                                           5× float32 (radians)
                                                    │
                 ◄──────────────────────  port 10002  ◄──────────  (joint angle feedback)

                    [when hardware is ready]

ik_process.py  ──384 bytes──►  port 9998  ──►  hw_process.py  ──►  Motors
```

**Packet formats**
- Tracking → IK: JSON `{ "type": "tracking", "hand_pos": [...], "hand_rot": [...], "elbow_pos": [...] }`
- IK → feedback: 5× `float32` joint angles in radians (little-endian)
- IK → HW:       384 bytes — 32 motor slots × 12 bytes each (`pos_deg`, `tau_nm`, `vel` as `float32`); slots indexed by motor ID

> **Note:** The tracking source was previously Unity. It is being replaced with a ROS 2 topic publisher. The IK process is agnostic to the source — anything that sends the JSON packet format above to port 10001 works.

---

## File layout

```
src/
  ik_process.py          IK solver — receives tracking, runs QP-IK, replies with joint angles
  hw_process.py          Hardware loop — 1000 Hz Dynamixel sync read/write
  mock_dynamixel_sdk.py  Drop-in stub for testing without motors
  test_ik_no_source.py   Sends fake tracking packets to test IK standalone
  robot_arm/
    config.py            All tunable parameters (ports, IK costs, motor IDs, …)
    comm.py              IKComm and HWComm UDP classes
    urdf/
      arm.urdf           Robot kinematic model
```

## Configuration

All parameters live in `robot_arm/config.py` — ports, motor IDs, IK costs, loop rate, etc.
No need to edit the process files for routine changes.

**Port defaults** (`CommConfig`):

| Port  | Direction          | Purpose                            |
|-------|--------------------|------------------------------------|
| 10001 | Tracking → IK      | Hand/elbow tracking packets (JSON) |
| 10002 | IK → feedback      | Joint angles (5× float32)          |
| 9998  | IK → HW            | Motor commands (384 bytes)         |
| 9997  | HW → IK            | Motor feedback (384 bytes, future) |

**Motor defaults** (`HardwareConfig`):

| ID | Model |
|----|-------|
| 2  | XM430-W210 |
| 3  | XH430-W210 |
| 4  | XC330-T181 |
| 5  | XC330-T181 |
| 6  | XC330-T181 |

---

## Install dependencies

```bash
conda create -n robot python=3.12
conda activate robot
conda install -c conda-forge pinocchio pink numpy
pip install meshcat dynamixel_sdk
```

---

## Dry-run — IK only (no hardware, no tracking source)

```bash
# Terminal 1 — start IK process
cd src
python ik_process.py
# → opens Meshcat in browser (URL printed in terminal)
# → arm sits at zero pose, waiting for tracking packets

# Terminal 2 — send fake tracking packets
python test_ik_no_source.py
# → sweeps hand target in a sine-wave circle
# → arm should follow in the Meshcat window
```

Or inject a single packet manually:

```bash
python - <<'EOF'
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = json.dumps({
    "type": "tracking",
    "hand_pos":  [0.1, 0.2, 0.15],
    "hand_rot":  [1.0, 0.0, 0.0, 0.0],
    "elbow_pos": [0.05, 0.1, 0.1],
})
s.sendto(msg.encode(), ("127.0.0.1", 10001))
EOF
```

## Dry-run — hardware process without motors

Swap the import at the top of `hw_process.py`:

```python
# Replace:
from dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite, GroupFastSyncRead)

# With:
from mock_dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite, GroupFastSyncRead)
```

Then:

```bash
cd src
python hw_process.py
# → prints MOCK motor init logs
# → enters 1000 Hz loop, prints live Hz counter
# → accepts command packets on port 9998
```

---

## Running with hardware (full pipeline)

```bash
# Terminal 1
cd src
python ik_process.py

# Terminal 2
cd src
python hw_process.py --port /dev/ttyUSB0   # or COM3 on Windows

# Then start your tracking source (ROS 2 publisher or test script)
```

---

## IK task cost tuning

All in `IKConfig` in `robot_arm/config.py`:

| Parameter               | Default | Effect                                         |
|-------------------------|---------|------------------------------------------------|
| `position_cost`         | 1.0     | How hard IK pulls end-effector to target       |
| `orientation_cost`      | 0.5     | How hard IK matches hand orientation           |
| `elbow_position_cost`   | 0.1     | Elbow hint weight — lower = softer             |
| `posture_cost`          | 0.01    | Pull back toward zero pose                     |
| `safe_reach_fraction`   | 0.98    | Clamp target to this fraction of max reach     |
| `deadzone_threshold`    | 0.001   | Min joint change (rad) to trigger an update    |

## Adding a motor

In `robot_arm/config.py`, add to `HardwareConfig.actuators`:

```python
actuators: Dict[int, str] = field(default_factory=lambda: {
    2: "XM430-W210",
    3: "XH430-W210",
    4: "XC330-T181",
    5: "XC330-T181",
    6: "XC330-T181",
    7: "XM430-W350",   # new motor
})
```

Also add the ID to `IKConfig.motor_ids` so the IK process packs commands for it.

---

## ROS 2 integration (next step)

The tracking source will be a ROS 2 node that publishes hand/elbow poses and bridges them into the UDP packet format on port 10001. Planned additions:

- `robot_arm/comm.py` gains `HWROS2Comm` / `IKROS2Comm` classes that publish/subscribe to ROS 2 topics instead of UDP.
- `robot_arm/config.py` gains a `ROS2Config` dataclass for topic names and QoS.
- Both process entry points accept `--comm ROS2` via their CLI.
- No other files change.
