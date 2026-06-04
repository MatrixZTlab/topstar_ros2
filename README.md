# Topstar ROS2 Workspace

[简体中文说明](README.zh-CN.md)

[H2 Example Tutorial (中文)](H2_EXAMPLES_TUTORIAL.zh-CN.md)

This repository contains the ROS 2 interface packages and example nodes for
communicating with Topstar robots through CycloneDDS.

Two robots are supported:

- **H1** — 18-DOF wheeled humanoid (upper body + 4-wheel swerve base), Python-based
- **H2** — bipedal humanoid, C++-based

## Repository Layout

```
topstar_ros2/
├── cyclonedds_ws/          # ROS2 interface packages (shared by both robots)
│   └── src/topstar/
│       ├── topstar_hg/     # Low-level robot message definitions
│       └── topstar_api/    # API request/response message definitions
├── example/
│   ├── src/                # topstar_ros2_example (H1 Python nodes + mujoco_ros2_bridge)
│   ├── isaac_bridge/       # Isaac Sim ↔ ROS2 bridge scripts for H1
│   ├── build_h1.sh         # Build H1 package only
│   ├── h1_tune_env.sh      # H1 gain / tuning environment variables
│   ├── h2_motor_plot.py    # H2 joint motor visualizer
│   ├── run_motor_plot.sh   # Launch H2 motor visualizer with env setup
│   ├── test_jog_commands.sh
│   └── test_steering_stability.sh
├── setup.sh                # Robot on Ethernet (eno1)
├── setup_local.sh          # Local loopback (lo) for simulation
├── setup_default.sh        # ROS2 + CycloneDDS, no interface override
├── sync_to_jqr.sh          # Sync repo to remote machine (jqr@192.168.1.30)
└── zip_redeploy.sh         # Create clean archive for redeployment
```

Generated directories (`build/`, `install/`, `log/`) under `cyclonedds_ws` and
`example` are safe to delete and recreate.

## Requirements

**Common:**

- Ubuntu 22.04
- ROS 2 Humble
- CycloneDDS RMW: `rmw_cyclonedds_cpp`

```bash
sudo apt update
sudo apt install \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev
```

**H1 (Python, system Python 3.10):**

| Package | Required by | Install |
|---|---|---|
| `mujoco` | MuJoCo backend | `pip3 install mujoco` |
| `numpy` | all | already installed |
| `scipy ≥ 1.11` | xapi / hardware backend | `sudo pip3 install --upgrade scipy` |
| `pyzmq` | Isaac Sim backend | `sudo pip3 install pyzmq` |
| `atomics` | xapi hardware backend | `sudo pip3 install atomics` |
| `waiting` | xapi hardware backend | `sudo pip3 install waiting` |
| `xapi` | hardware backend | vendor wheel (see [H1 Hardware Backend](#h1-hardware-backend)) |
| `PySide6` | upper-body jog GUI | `sudo pip3 install PySide6` |

> ROS2 Humble uses `/usr/bin/python3` (3.10.12). Install packages system-wide
> with `sudo pip3 install` — no virtualenv needed.

**H2 (C++):**

No additional system packages beyond the common requirements.

---

## Build Interface Packages

Build the shared message packages first (required before building either robot):

```bash
source ~/topstar_ros2/setup.sh
cd ~/topstar_ros2/cyclonedds_ws
colcon build
```

---

## Environment Setup

Three setup helpers are provided:

| Script | DDS Interface | Use case |
|---|---|---|
| `setup.sh` | `eno1` (Ethernet) | Real robot over physical network |
| `setup_local.sh` | `lo` (loopback) | Local simulation, no robot |
| `setup_default.sh` | _(none set)_ | General — let CycloneDDS auto-detect |

Recommended usage:

- Connect to robot: `source ~/topstar_ros2/setup.sh`
- Test local simulation: `source ~/topstar_ros2/setup_local.sh`
- Once packages are built, sourcing either script above is sufficient.
  No extra `source /opt/ros/...` or `source .../install/setup.bash` is needed.

Before using `setup.sh`, confirm the interface name matches your machine
(default: `eno1`).

Typical robot network settings:

- Host IPv4: `192.168.1.10`
- Netmask: `255.255.255.0`

---

## H1 Robot

### Build

H1 lives in the `topstar_ros2_example` package (`example/src`).

```bash
source ~/topstar_ros2/setup_local.sh   # or setup.sh for real robot
cd ~/topstar_ros2/example
bash build_h1.sh                       # equivalent to colcon build --packages-select topstar_ros2_example --symlink-install
```

### Quick Start

```bash
# Launch with MuJoCo (default) — opens simulation + ROS2 bridge
ros2 launch topstar_ros2_example h1_sim.launch.py viewer:=true

# Headless MuJoCo
ros2 launch topstar_ros2_example h1_sim.launch.py

# Isaac Sim backend
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=isaac

# Hardware (xapi) backend
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi

# Upper-body jog GUI (separate terminal, after launch)
ros2 run topstar_ros2_example h1_upper_body_jog

# Drive + arm-wave demo
ros2 run topstar_ros2_example h1_drive_example

# Send a single base velocity command
ros2 run topstar_ros2_example h1_send_velocity
```

### H1 Executables

| Executable | Description |
|---|---|
| `h1_ros2_node` | ROS2 bridge node — translates ROS2 topics ↔ active backend |
| `h1_drive_example` | Drive + arm-wave demo node |
| `h1_send_velocity` | Minimal base velocity sender utility |
| `h1_upper_body_jog` | PySide6 GUI for manual upper-body joint jogging |

### H1 ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/h1/lowcmd` | `topstar_hg/LowCmd` | subscribed | Upper-body joint position commands (18 DOF) |
| `/h1/base_cmd` | `geometry_msgs/Twist` | subscribed | Base velocity (`vx`, `vy`, `omega`) |
| `/h1/lowstate` | `topstar_hg/LowState` | published | Joint + IMU state (18 DOF), 50 Hz default |
| `/api/arm/request` | `topstar_api/Request` | subscribed | Arm API requests |
| `/api/arm/response` | `topstar_api/Response` | published | Arm API responses |

State publication rate can be overridden at launch: `state_hz:=100`.

### H1 Backends

| Backend | `backend:=` value | Requirements | Use case |
|---|---|---|---|
| MuJoCo | `mujoco` (default) | `~/topstar_mujoco/simulate_python` present | Physics simulation |
| Isaac Sim | `isaac` | Isaac Sim running, `pyzmq` installed | Sim with RTX rendering |
| Hardware | `xapi` | `xapi` vendor wheel, robot connected | Real robot |

Override the MuJoCo sim binary path at launch:

```bash
ros2 launch topstar_ros2_example h1_sim.launch.py sim_path:=/other/path
# or
export TOPSTAR_SIM_PATH=/other/path
```

### H1 Hardware Backend

The hardware backend requires the vendor-supplied wheel:

```bash
pip3 install ~/topstar_ros2/xapi-3.3.8-cp310-cp310-linux_x86_64.whl
```

Optional arm motion config (speed / limits):

```bash
export TOPSTAR_H1_UPPER_BODY_CFG='{"max_speed": 0.5}'
```

### H1 Gain Tuning

`h1_tune_env.sh` exports all tunable gain and safety parameters as environment
variables (steer Kp/Kd, drive damping, overspeed thresholds, etc.). Source it
before launching to apply custom tuning:

```bash
source ~/topstar_ros2/example/h1_tune_env.sh
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi
```

### Isaac Sim Bridge (H1)

`example/isaac_bridge/` provides a two-process bridge between Isaac Sim and the
ROS2 stack over local ZMQ sockets (ports 15555 / 15556).

**Launch both processes together** (run on the Isaac Sim machine):

```bash
bash ~/topstar_ros2/example/isaac_bridge/launch_h1_bridge.sh          # GUI
bash ~/topstar_ros2/example/isaac_bridge/launch_h1_bridge.sh --headless
```

Isaac Sim takes 15–30 s to start; the script waits 25 s before starting the
bridge. The bridge auto-detects the LAN interface that reaches `192.168.1.0/24`.

Bridge ROS2 topics: `/h1/lowstate` (published), `/h1/lowcmd` (subscribed),
`/h1/base_cmd` (subscribed).

**Sync the repo to the Isaac Sim machine:**

```bash
bash ~/topstar_ros2/sync_to_jqr.sh                    # pushes to jqr@192.168.1.30
bash ~/topstar_ros2/sync_to_jqr.sh user@other-host    # custom target
```

The sync script excludes build artifacts and regenerates `h1_abs.urdf` on the
remote after each push.

### H1 Arm API

The H1 arm API uses the same `topstar_api` request/response envelope as H2.
Clients publish to `/api/arm/request`; responses arrive on `/api/arm/response`
matched by `header.identity.id`.

| `api_id` | Name | Description |
|---|---|---|
| `1001` | `move_joints_timed` | Move all 18 upper-body joints to target positions over a given duration |

`1001` request `parameter` JSON:

```json
{ "joints": [<float> × 18], "duration": <float> }
```

Response codes: `0` = success, `1001` = invalid parameters, `1002` = internal error.

---

## H2 Robot

### Build

H2 C++ examples live in the `topstar_ros2_h2_example` package (`example/src/src/h2/`).
`mujoco_ros2_bridge` lives in the `topstar_ros2_example` package (`example/src/`).

```bash
source ~/topstar_ros2/setup.sh
cd ~/topstar_ros2/example
colcon build --packages-select topstar_ros2_h2_example  # H2 examples only
# or
colcon build                                             # all packages
```

### H2 Executables

| Executable | Package | Description |
|---|---|---|
| `read_low_state_hg` | `topstar_ros2_h2_example` | Read and print low-level state topics |
| `h2_low_level_example` | `topstar_ros2_h2_example` | Low-level motor control example |
| `h2_ankle_swing_example` | `topstar_ros2_h2_example` | Ankle swing control example |
| `h2_joint_oscillation_example` | `topstar_ros2_h2_example` | Joint oscillation demo |
| `h2_arm_sdk_dds_example` | `topstar_ros2_h2_example` | DDS-based arm SDK example |
| `h2_arm_action_example` | `topstar_ros2_h2_example` | Arm action example |
| `h2_loco_client_example` | `topstar_ros2_h2_example` | Locomotion client example |
| `h2_ls_hand_example` | `topstar_ros2_h2_example` | LS hand control example |
| `mujoco_ros2_bridge` | `topstar_ros2_example` | DDS relay bridge for MuJoCo digital twin / kinematic mirror |

Run an example:

```bash
cd ~/topstar_ros2/example
source ~/topstar_ros2/setup.sh
ros2 run topstar_ros2_h2_example h2_joint_oscillation_example
```

### H2 Motor Visualizer

A real-time joint motor plot that reads `rt/lowstate` via ROS2:

```bash
# All joints, position + torque
bash ~/topstar_ros2/example/run_motor_plot.sh

# Specific joints or groups
bash ~/topstar_ros2/example/run_motor_plot.sh --joints left_leg --mode torque
bash ~/topstar_ros2/example/run_motor_plot.sh --joints legs --cols 4 --window 15
```

The script sources the Ethernet environment (`eno1`) automatically.

### H2 DDS Topics

| Topic | Message | Direction |
|---|---|---|
| `rt/lowstate` | `topstar_hg::msg::LowState` | Robot → ROS2 |
| `rt/lowcmd` | `topstar_hg::msg::LowCmd` | ROS2 → Robot |
| `rt/bms/state` | `topstar_hg::msg::BmsState` | Robot → ROS2 |
| `rt/bms/cmd` | `topstar_hg::msg::BmsCmd` | ROS2 → Robot |
| `rt/api/sport/request` | `topstar_api::msg::Request` | ROS2 → Robot |
| `rt/api/sport/response` | `topstar_api::msg::Response` | Robot → ROS2 |
| `rt/hand/left/cmd` | `topstar_hg::msg::HandCmd` | ROS2 → Robot |
| `rt/hand/left/state` | `topstar_hg::msg::HandState` | Robot → ROS2 |

### MuJoCo Bridge (`mujoco_ros2_bridge`)

Built automatically as part of `topstar_ros2_example` when both conditions are met:

- `~/topstar_mujoco/simulate/src/topstar_hg.c` exists (DDS type definitions)
- CycloneDDS is found by CMake (provided by ROS 2 Humble)

If either is missing the rest of the package still builds.

```bash
# Digital twin: relay rt/lowcmd from real robot into MuJoCo
ros2 run topstar_ros2_example mujoco_ros2_bridge

# Kinematic mirror: also reflect actual joint state in the MuJoCo viewer
# (run topstar_mujoco with --lowstate in a separate terminal)
ros2 run topstar_ros2_example mujoco_ros2_bridge   # relay side
~/topstar_mujoco/simulate/build/topstar_mujoco -n lo --lowstate  # viewer side
```

| Option | Default | Description |
|---|---|---|
| `--robot_interface=IF` | `eno1` | DDS interface for real-robot traffic |
| `--sim_interface=IF` | `lo` | DDS interface for MuJoCo traffic |

The bridge relays two DDS topics:

| Robot interface | → | Sim interface | Notes |
|---|---|---|---|
| `rt/lowcmd` | → | `rt/lowcmd` | Commands into MuJoCo actuators |
| `rt/lowstate` | → | `rt/lowstate_robot` | Read by `--lowstate` kinematic mirror |

---

## Utilities

### Redeployment Archive

Create a clean archive suitable for copying to another machine:

```bash
bash ~/topstar_ros2/zip_redeploy.sh             # topstar_ros2_redeploy_YYYYMMDD.zip
bash ~/topstar_ros2/zip_redeploy.sh custom.zip  # custom filename
```

The archive preserves the sibling layout expected by the code on the target
machine:

- `topstar_ros2/` source, setup scripts, CycloneDDS message workspace, examples
- `topstar_mujoco/` runtime/build inputs: `simulate/`, `simulate_python/`, `topstar_robots/`
- `topstar_h2/h2_model/` H2 meshes and URDF inputs

Build artifacts, logs, caches, and `__pycache__` are excluded. Generated
`topstar_h2/h2_model/urdf/h2_abs.urdf` is also excluded so the target machine
can regenerate it with local absolute mesh paths.

By default the script looks for `~/topstar_mujoco` and `~/topstar_h2` next to
`~/topstar_ros2`. Override those locations if needed:

```bash
TOPSTAR_MUJOCO_DIR=/path/to/topstar_mujoco \
TOPSTAR_H2_DIR=/path/to/topstar_h2 \
bash ~/topstar_ros2/zip_redeploy.sh
```

Extract the archive into `~` on the destination machine so the three folders end
up as siblings again.

Additional packaging behavior:

- If `topstar_mujoco/topstar_robots/h1` contains broken external mesh symlinks,
  `zip_redeploy.sh` auto-fills them from `topstar_ros2/example/src/urdf/h1/meshes`
  when available.
- If any required dependency path is missing, the script exits with a clear error
  instead of producing an incomplete bundle.
