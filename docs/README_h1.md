# H1 Robot — ROS2 Package

Self-contained ROS2 package for the **H1 wheeled humanoid** (18-DOF upper body + 4-wheel swerve base).  
All controller code lives inside this package; no external Python repositories are required at runtime.

---

## Status

| Feature | Status |
|---|---|
| MuJoCo simulation backend | ✅ Working |
| Isaac Sim backend (ZMQ) | ✅ Working (requires `pyzmq` in system Python) |
| Hardware xapi backend | ✅ Working (requires `xapi` vendor wheel) |
| ROS2 bridge node (`h1_ros2_node`) | ✅ Working |
| Upper-body jog GUI (`h1_upper_body_jog`) | ✅ Working |
| Drive example (`h1_drive_example`) | ✅ Working |
| Arm API — timed joint move (`api_id` 1001) | ✅ Working (all backends) |
| X5 dual-arm monitor GUI (`x5_monitor`) | ✅ Working (requires `xapi` vendor wheel) |

### Python environment requirements (system Python 3.10)

| Package | Required by | Install |
|---|---|---|
| `mujoco` | MuJoCo backend | `pip3 install mujoco` |
| `numpy` | all | already installed |
| `scipy ≥ 1.11` | xapi / hardware backend | `sudo pip3 install --upgrade scipy` |
| `pyzmq` | Isaac backend | `sudo pip3 install pyzmq` |
| `atomics` | xapi hardware backend | `sudo pip3 install atomics` |
| `waiting` | xapi hardware backend | `sudo pip3 install waiting` |
| `xapi` | hardware backend | vendor wheel (see below) |
| `PySide6` | jog GUI | `sudo pip3 install PySide6` |

> ROS2 Humble and the global Python are the **same interpreter** (`/usr/bin/python3 3.10.12`).  
> No virtualenv or conda environment is needed; install packages system-wide with `sudo pip3 install`.

---

## Quick Start

```bash
# 1 — Source workspaces
#   Local simulation (loopback interface, no physical network required):
source ~/topstar_ros2/setup_local.sh
#   Or, for the real robot over the physical network:
source ~/topstar_ros2/setup.sh

# 2a — Launch with MuJoCo viewer (opens simulation window + starts ROS2 bridge)
ros2 launch topstar_ros2_example h1_sim.launch.py viewer:=true

# 2b — Headless MuJoCo (ROS2 bridge only, sim runs in background thread)
ros2 launch topstar_ros2_example h1_sim.launch.py

# 2c — Isaac Sim backend
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=isaac

# 2d — Hardware (xapi) backend
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi

# 3 — Upper-body jog GUI (in a separate terminal, after step 2)
ros2 run topstar_ros2_example h1_upper_body_jog

# 4 — Drive + arm-wave example
ros2 run topstar_ros2_example h1_drive_example
```

### Build

```bash
cd ~/topstar_ros2/example
source ~/topstar_ros2/setup_local.sh   # or setup.sh
colcon build --symlink-install --packages-select topstar_ros2_example
source install/setup.bash
```

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/lowcmd` | `topstar_hg/LowCmd` | subscribed | Upper-body joint position commands (slots 0–17) |
| `/base_cmd` | `geometry_msgs/Twist` | subscribed | Base velocity (`vx`, `vy`, `omega`) |
| `/lowstate` | `topstar_hg/LowState` | published | Joint + IMU state (slots 0–17), 50 Hz default |
| `/hand/right/cmd` | `topstar_hg/GripperCmd` | subscribed | Right gripper position command |
| `/hand/left/cmd` | `topstar_hg/GripperCmd` | subscribed | Left gripper position command |
| `/hand/right/state` | `topstar_hg/GripperState` | published | Right gripper position + effort + status |
| `/hand/left/state` | `topstar_hg/GripperState` | published | Left gripper position + effort + status |
| `/api/arm/request` | `topstar_api/Request` | subscribed | Arm API requests (see [Arm API](#arm-api) below) |
| `/api/arm/response` | `topstar_api/Response` | published | Arm API responses |

The state publication rate can be changed at launch: `state_hz:=100`.

**`GripperCmd` fields:**

| Field | Type | Description |
|---|---|---|
| `position` | `float32` | Target position: `0.0` = fully open, `1.0` = fully closed |
| `mode` | `uint8` | `0` = idle, `1` = position control |

**`GripperState` fields:**

| Field | Type | Description |
|---|---|---|
| `position` | `float32` | Current position: `0.0` = open, `1.0` = closed |
| `effort` | `float32` | Motor effort estimate |
| `status` | `uint8` | `0` = OK, non-zero = error |

---

## Arm API

The arm API follows the same **`topstar_api` request/response** convention used by the H2 robot.  
Clients publish a `topstar_api/Request` to `/api/arm/request`; `h1_ros2_node` processes the request and publishes a `topstar_api/Response` to `/api/arm/response`.  
Responses are matched to requests via the `header.identity.id` field.

### Message envelope

```
Request
  header.identity.api_id   int32    — identifies the operation (see table below)
  header.identity.id       int64    — caller-assigned request ID, echoed in the response
  parameter                string   — JSON-encoded operation parameters
  binary                   uint8[]  — unused by H1 arm API (always empty)

Response
  header.identity.api_id   int32    — echoed from request
  header.identity.id       int64    — echoed from request
  header.status.code       int32    — 0 = success, non-zero = error (see table below)
  data                     string   — JSON-encoded result or error message
```

### API operations

| `api_id` | Name | Description |
|---|---|---|
| `1001` | `move_joints_timed` | Move all 18 upper-body joints to target positions over a specified duration |

### `1001` — `move_joints_timed`

Move the upper body smoothly from its current pose to a target pose over `duration` seconds.  
The response is sent immediately once the command is validated; the motion runs asynchronously.  
While the move is in progress, regular `/lowcmd` joint commands are suppressed so they cannot override the trajectory.

**Request `parameter` JSON**

```json
{
  "joints":   [<float>, ...],   // 18 values, hardware frame (rad or m), indexed by H1JointIndex
  "duration": <float>           // seconds, must be > 0
}
```

**Response `header.status.code`**

| Code | Meaning |
|---|---|
| `0` | Command accepted; motion started |
| `1001` | Invalid parameters (bad JSON, wrong joint count, non-positive duration) |
| `1002` | Internal error |

**Response `data`** is an empty JSON object (`{}`) on success, or a plain-text error description on failure.

### Python client example

```python
import json
import rclpy
from rclpy.node import Node
from topstar_api.msg import Request as ArmRequest

ARM_API_ID_MOVE_JOINTS_TIMED = 1001

class ArmMoveClient(Node):
    def __init__(self):
        super().__init__("arm_move_client")
        self._pub = self.create_publisher(ArmRequest, "/api/arm/request", 10)

    def move(self, joints: list[float], duration: float, req_id: int = 1) -> None:
        msg = ArmRequest()
        msg.header.identity.api_id = ARM_API_ID_MOVE_JOINTS_TIMED
        msg.header.identity.id = req_id
        msg.parameter = json.dumps({"joints": joints, "duration": duration})
        self._pub.publish(msg)

rclpy.init()
node = ArmMoveClient()
# Move all joints to zero over 3 seconds
node.move([0.0] * 18, duration=3.0)
rclpy.spin_once(node, timeout_sec=0.1)
rclpy.shutdown()
```

### Interaction with `/lowcmd`

`/lowcmd` joint commands are suppressed for exactly `duration` seconds from the moment `move_joints_timed` is accepted.  
After that window expires, `/lowcmd` resumes control.  
To hold the final pose after a timed move, ensure the next `/lowcmd` you publish carries the same joint targets that were sent to `move_joints_timed`.

---

## File Structure

```
h1/
├── README.md                      ← this file
├── launch/
│   └── h1_sim.launch.py           ← main launch file (backend / viewer selection)
└── topstar_h1/                    ← Python package (installed by colcon)
    ├── __init__.py
    ├── joint_defs.py              ← H1JointIndex enum, limits, HW↔MuJoCo scale factors
    ├── h1_ros2_node.py            ← ROS2 bridge node (thin topic ↔ backend layer)
    ├── h1_bridge.py               ← Unified bridge composing base + upper-body controllers
    ├── h1_upper_body.py           ← 18-DOF upper-body controller (mock or real InterpController)
    ├── h1_base.py                 ← 4-wheel swerve base controller
    ├── mock_xapi.py               ← Synchronous MuJoCo-backed drop-in for InterpController
    ├── h1_upper_body_jog.py       ← PySide6 GUI for manual joint jogging
    ├── h1_drive_example.py        ← Drive + arm-wave demo node
    ├── h1_send_velocity.py        ← Minimal base velocity sender utility
    ├── x5_monitor.py              ← Dual X5 arm monitor / jog GUI (xapi, PySide6)
    ├── backends/
    │   ├── __init__.py            ← Lazy factory: create_backend(name, **kwargs)
    │   ├── base.py                ← Abstract H1Backend interface
    │   ├── mujoco.py              ← MuJoCo backend (wraps H1Bridge + topstar_mujoco)
    │   ├── isaac.py               ← Isaac Sim backend (ZMQ PUSH/PULL ports 15555/15556)
    │   └── xapi.py                ← Hardware xapi backend (vendor wheel)
    └── vendor/
        └── topstar/               ← Vendored helpers (copied from xapi stack)
            ├── topstar_xapi.py
            ├── topstar_kine.py
            ├── pose_trajectory_interpolator.py
            ├── pose_util.py
            ├── precise_sleep.py
            ├── shared_memory_queue.py
            ├── shared_memory_ring_buffer.py
            ├── shared_memory_util.py
            └── shared_ndarray.py
```

---

## Backend Selection

The backend is chosen at launch via the `backend` argument or the `TOPSTAR_H1_BACKEND` environment variable.

| Backend | Value | Requirements | Use case |
|---|---|---|---|
| MuJoCo | `mujoco` (default) | `~/topstar_mujoco/simulate_python` present | Simulation (physics) |
| Isaac Sim | `isaac` | Isaac Sim running, `pyzmq` installed | Sim with RTX rendering |
| Hardware | `xapi` | `xapi` vendor wheel installed, robot connected | Real robot |

### MuJoCo sim path

Default: `~/topstar_mujoco/simulate_python`.  
Override at launch: `ros2 launch ... sim_path:=/other/path`  
or via env var: `export TOPSTAR_SIM_PATH=/other/path`

### Hardware upper-body config

Optional JSON config for arm motion limits / speed, passed via:
```bash
export TOPSTAR_H1_UPPER_BODY_CFG='{"max_speed": 0.5}'
```

### xapi vendor wheel

The hardware backend requires the vendor-supplied wheel:
```bash
pip3 install ~/lerobot4/xapi-3.2.5-cp310-cp310-linux_x86_64.whl
```

---

## Joint Index Reference

| Index | `H1JointIndex` name | URDF joint |
|---|---|---|
| 0 | `TORSO_LIFT` | Robot_Body_Movement_Joint |
| 1 | `TORSO_PITCH` | Robot_Body_Rotation_Joint |
| 2 | `HEAD_YAW` | Robot_Head_Rotation_Joint |
| 3 | `HEAD_PITCH` | Robot_Head_Tonod_Joint |
| 4 | `RIGHT_SHOULDER_BASE` | Robot_Right_Hand_base_Joint |
| 5 | `RIGHT_SHOULDER` | Robot_Right_Hand_1_Joint |
| 6 | `RIGHT_ELBOW_YAW` | Robot_Right_Hand_2_Joint |
| 7 | `RIGHT_ELBOW` | Robot_Right_Hand_3_Joint |
| 8 | `RIGHT_WRIST_YAW` | Robot_Right_Hand_4_Joint |
| 9 | `RIGHT_WRIST_PITCH` | Robot_Right_Hand_5_Joint |
| 10 | `RIGHT_WRIST_ROLL` | Robot_Right_Hand_6_Joint |
| 11 | `LEFT_SHOULDER_BASE` | Robot_Left_Hand_base_Joint |
| 12 | `LEFT_SHOULDER` | Robot_Left_Hand_1_Joint |
| 13 | `LEFT_ELBOW_YAW` | Robot_Left_Hand_2_Joint |
| 14 | `LEFT_ELBOW` | Robot_Left_Hand_3_Joint |
| 15 | `LEFT_WRIST_YAW` | Robot_Left_Hand_4_Joint |
| 16 | `LEFT_WRIST_PITCH` | Robot_Left_Hand_5_Joint |
| 17 | `LEFT_WRIST_ROLL` | Robot_Left_Hand_6_Joint |

---

## Architecture Notes

- `h1_ros2_node` is a **thin layer** — it only translates ROS2 messages to/from the active backend. All robot logic lives in the backend.
- Backend imports are **lazy**: only the selected backend's module is imported at runtime, so e.g. missing `zmq` does not break the MuJoCo path.
- `viewer:=true` spawns `topstar_mujoco.py` as a separate process. That process imports `H1Ros2Node` internally and calls `rclpy.spin()`, sharing the bridge with the MuJoCo physics loop.
- `vendor/topstar/` contains helpers vendored from the xapi stack with all inter-module imports rewritten to local relative imports.
