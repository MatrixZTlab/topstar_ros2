# H1 Robot — MuJoCo + ROS2 Setup Guide

## Repository Layout

```
~/topstar_ros2/
├── cyclonedds_ws/          # DDS infrastructure (messages, API)
│   └── src/topstar/
│       ├── topstar_hg/     # LowCmd / LowState message definitions
│       └── topstar_api/    # Topstar API library
└── example/                # All robot examples (H1 + H2) — mirrors unitree_ros2
    └── src/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        │   ├── h1/h1.hpp       # H1 joint enums + default PD gains (C++ header)
        │   └── h2/h2.hpp       # H2 joint enums
        ├── src/
        │   ├── h1/             # H1 Python nodes + launch file
        │   │   ├── topstar_h1/ # Python package (joint_defs, ros2 node, drive example)
        │   │   └── launch/h1_sim.launch.py
        │   └── h2/             # H2 C++ examples (lowlevel, high_level, ls_hand)
        ├── config/h1/          # controller_v2.yml, vr_configs.yml, mocap config
        └── urdf/h1/            # Topstar.urdf + 28 STL meshes

~/topstar_mujoco/
├── simulate_python/
│   ├── topstar_mujoco.py   # Main simulation entry point (patched for H1)
│   ├── h1_bridge.py        # H1Bridge: unified base + upper-body bridge
│   ├── h1_upper_body.py    # 18-DOF upper-body controller
│   ├── h1_base.py          # 4-wheel steerable base controller
│   ├── h1_joint_defs.py    # Joint index enums, sign conventions, limits
│   └── mock_xapi.py        # MockInterpController (sim backend for arms)
└── topstar_robots/h1/
    ├── h1.xml              # Generated MuJoCo model (27 joints, 26 actuators)
    └── scene.xml           # Scene wrapper (ground, lighting, integrator)
```

---

## Bug Fixes Applied

### 1. MuJoCo simulation instability (`Nan, Inf or huge value in QACC`)

**Root cause:** The default MuJoCo `euler` integrator is numerically unstable with
stiff position actuators on low-inertia arm joints (effective inertia as low as
0.012 kg·m²).

**Fix:** Added `implicitfast` integrator to `topstar_robots/h1/scene.xml`:

```xml
<option integrator="implicitfast" timestep="0.005"/>
```

`implicitfast` treats spring and damping forces implicitly, giving unconditional
stability for position-controlled joints without changing control gains.

---

### 2. Simulation deadlock (joints stuck at zero)

**Root cause:** `topstar_mujoco.py` held `locker` while calling `bridge.step()`,
which internally called `MockInterpController._write_actuator_commands()` — which
also tried to acquire the same `locker`. `threading.Lock` is not reentrant, so
the same thread deadlocked on itself. `mujoco.mj_step()` never ran.

**Fix:** Changed `locker` from `Lock` to `RLock` (reentrant lock):

```python
# topstar_mujoco.py
locker = threading.RLock()   # was: threading.Lock()
```

---

### 3. `rclpy.init()` / `Node()` atexit error in thread

**Root cause:** `rclpy.init()` and `rclpy.Node.__init__()` register Python
`atexit` handlers, which must be called from the **main thread**. They were
being called inside `SimulationThread`.

**Fix:** Moved bridge creation, `rclpy.init()`, and `H1Ros2Node()` instantiation
to `__main__` (main thread). `SimulationThread` now receives the bridge as an
argument:

```python
if __name__ == "__main__":
    bridge = H1Bridge(mj_model, mj_data, ..., lock=locker)

    rclpy.init()
    ros2_node = H1Ros2Node(bridge)
    Thread(target=rclpy.spin, args=(ros2_node,), daemon=True).start()

    sim_thread = Thread(target=SimulationThread, args=(bridge,))
    ...
```

---

### 4. ROS2 launch — libexec directory missing

**Root cause:** `ament_python` installs `console_scripts` to `bin/` by default,
but `ros2 run` looks for executables in `lib/<package_name>/`.

**Fix (original):** Added `setup.cfg` to the standalone `topstar_h1` ament_python
package to redirect scripts to `lib/topstar_h1/`.

**Current fix (after merge into `example`):** The `topstar_h1` Python package now
lives inside the `ament_cmake` package `topstar_ros2_example`. Entry points are
installed explicitly via `CMakeLists.txt`:

```cmake
install(PROGRAMS src/h1/topstar_h1/h1_ros2_node.py
  RENAME h1_ros2_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS src/h1/topstar_h1/h1_drive_example.py
  RENAME h1_drive_example DESTINATION lib/${PROJECT_NAME})
```

No `setup.cfg` is needed; the executables land in `lib/topstar_ros2_example/`.

---

### 5. `PackageNotFoundError: topstar-h1` at launch

**Root cause:** The `Node` launch action used `env={}` which **replaces** the
entire process environment, discarding `PYTHONPATH` and making the package
metadata unfindable.

**Fix:** Changed to `additional_env={}` in `h1_sim.launch.py`, which merges
with the inherited environment:

```python
Node(
    ...
    additional_env={          # was: env={}
        "TOPSTAR_ROBOT": "h1",
        "TOPSTAR_SIM_PATH": LaunchConfiguration("sim_path"),
    },
)
```

---

### 6. Relative `ROBOT_SCENE` path fails outside simulate_python/

**Root cause:** `config.ROBOT_SCENE` defaults to `"../topstar_robots/h1/scene.xml"`,
a path relative to `simulate_python/`. When started via `ros2 launch`, the
working directory is different.

**Fix:** Resolve the path relative to `sim_path` in `h1_ros2_node.py`:

```python
robot_scene = config.ROBOT_SCENE
if not os.path.isabs(robot_scene):
    robot_scene = os.path.normpath(os.path.join(sim_path, robot_scene))
mj_model = mujoco.MjModel.from_xml_path(robot_scene)
```

---

### 7. QoS incompatibility (`No messages will be received`)

**Root cause:** The H1 ROS2 node publishes `/h1/lowstate` with `BEST_EFFORT`
(sensor convention), but the example subscribed with default `RELIABLE` QoS.
ROS2 silently drops all messages when policies are incompatible.

**Fix:** Matched QoS profiles in `h1_drive_example.py` to match the node:

```python
_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, ...)  # lowstate sub
_CMD_QOS    = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,    ...)  # lowcmd/base_cmd pub
```

---

### 8. H1 package placed in wrong workspace

**Root cause:** Robot-specific code was placed in `cyclonedds_ws` alongside
infrastructure packages. The correct pattern (as with H2, mirroring unitree_ros2)
is to have all robot examples in a single `example` workspace.

**Fix:** Merged `topstar_h1` into `example/src/` alongside H2:

```
example/src/src/h1/topstar_h1/   # correct location (Python package)
example/src/include/h1/h1.hpp    # C++ header with joint enums + PD gains
example/src/urdf/h1/             # URDF + meshes (moved from ~/topstar_h1/)
example/src/config/h1/           # controller/VR/mocap configs
```

The `topstar_h1` Python package is installed via `ament_python_install_package`
inside the `ament_cmake` `topstar_ros2_example` package. Entry points
(`h1_ros2_node`, `h1_drive_example`) land in `lib/topstar_ros2_example/`.

---

## How to Build

### One-time: build infrastructure (only needed once or after message changes)

```bash
source /opt/ros/humble/setup.bash
cd ~/topstar_ros2/cyclonedds_ws
colcon build
```

### Build example package (H1 + H2)

```bash
source /opt/ros/humble/setup.bash
source ~/topstar_ros2/cyclonedds_ws/install/setup.bash
cd ~/topstar_ros2/example
colcon build --symlink-install
```

---

## How to Run

### Terminal 1 — MuJoCo simulation + ROS2 bridge

```bash
source ~/topstar_ros2/setup.sh
ros2 launch topstar_ros2_example h1_sim.launch.py viewer:=true
```

Or with custom sim path / publish rate:
```bash
ros2 launch topstar_ros2_example h1_sim.launch.py \
    sim_path:=~/topstar_mujoco/simulate_python \
    state_hz:=50
```

Expected output:
```
[INFO] [...] [h1_ros2_node]: H1Ros2Node started — publishing /h1/lowstate at 50 Hz
[H1] ROS2 node started — /h1/lowcmd  /h1/base_cmd  /h1/lowstate
```

### Terminal 2 — Example: drive forward + wave arms + print state

```bash
source ~/topstar_ros2/setup.sh
ros2 run topstar_ros2_example h1_drive_example
```

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/h1/lowstate` | `topstar_hg/LowState` | Published | Joint positions, velocities, IMU (50 Hz) |
| `/h1/lowcmd` | `topstar_hg/LowCmd` | Subscribed | Upper-body joint position targets (slots 0–17) |
| `/h1/base_cmd` | `geometry_msgs/Twist` | Subscribed | Base velocity (`vx`, `vy`, `angular.z`) |

### QoS policies

| Topic | Reliability | History | Depth |
|---|---|---|---|
| `/h1/lowstate` | `BEST_EFFORT` | `KEEP_LAST` | 1 |
| `/h1/lowcmd` | `RELIABLE` | `KEEP_LAST` | 10 |
| `/h1/base_cmd` | `RELIABLE` | `KEEP_LAST` | 10 |

---

## H1 Joint Index Reference

```
Slot  Joint                   Notes
─────────────────────────────────────────────
 0    TORSO_LIFT               Prismatic (metres), hw sign inverted
 1    TORSO_PITCH              hw sign inverted
 2    HEAD_YAW
 3    HEAD_PITCH               hw sign inverted
 4    RIGHT_SHOULDER_BASE
 5    RIGHT_SHOULDER
 6    RIGHT_ELBOW_YAW
 7    RIGHT_ELBOW
 8    RIGHT_WRIST_YAW
 9    RIGHT_WRIST_PITCH
10    RIGHT_WRIST_ROLL
11    LEFT_SHOULDER_BASE
12    LEFT_SHOULDER
13    LEFT_ELBOW_YAW
14    LEFT_ELBOW
15    LEFT_WRIST_YAW
16    LEFT_WRIST_PITCH
17    LEFT_WRIST_ROLL
```

LowCmd motor slots 0–17 map directly to these indices.
Base (4 wheels) is controlled separately via `/h1/base_cmd`.
