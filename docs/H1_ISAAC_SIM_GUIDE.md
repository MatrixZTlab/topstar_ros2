# H1 Robot — Isaac Sim Bridge

Run the Topstar H1 wheeled humanoid in Isaac Sim and control it over ROS2 using the same `/h1/*` topics as the MuJoCo bridge and the real hardware.

---

## Architecture

Two processes communicate over ZMQ on localhost. Python version incompatibility between Isaac Sim (3.11) and ROS2 Humble (3.10) makes a single-process approach impossible.

**Two-machine setup** (Isaac Sim on a dedicated GPU machine):

```
Control PC (192.168.1.20)              Isaac Sim machine (jqr@192.168.1.30)
─────────────────────────              ──────────────────────────────────────
ros2 run ... h1_upper_body_jog         Process 1: h1_isaac_sim.py
  /h1/lowcmd  ──────────────────────►    (conda env_isaaclab, Python 3.11)
  /h1/base_cmd ─────────────────────►    Isaac Lab / PhysX simulation
  /h1/lowstate ◄─────────────────────    200 Hz physics, 50 Hz state pub
                                         ZMQ PUSH :15555 (state out)
              DDS / CycloneDDS           ZMQ PULL :15556 (cmd in)
                                                │
                                         Process 2: h1_isaac_ros2_bridge.py
                                           (Python 3.10, ROS2 Humble)
                                           topstar_hg messages
```

**Same-machine setup** (Isaac Sim and ROS2 node on the same host):

```
h1_isaac_sim.py                  h1_ros2_node (backend:=isaac)
(conda env_isaaclab, Py 3.11)    (ROS2 Humble, Py 3.10)
  ZMQ PUSH :15555 (state) ────►  H1IsaacBackend.start()
  ZMQ PULL :15556 (cmd)   ◄────    → /h1/lowstate (pub)
                                   ← /h1/lowcmd   (sub)
                                   ← /h1/base_cmd (sub)
```

Use `launch_h1_bridge.sh` for the two-machine setup (starts both processes). Use `ros2 launch topstar_ros2_example h1_sim.launch.py backend:=isaac` for the same-machine setup (starts only the ROS2 node; run `h1_isaac_sim.py` separately).

**ZMQ protocol (localhost only)**

| Socket | Direction | Port | Content |
|--------|-----------|------|---------|
| PUSH/PULL | sim → bridge | 15555 | JSON state packet |
| PUSH/PULL | bridge → sim | 15556 | JSON command packet |

State packet: `{"t": float, "q": [18], "dq": [18], "quat": [4], "gyro": [3], "acc": [3]}`  
LowCmd packet: `{"type": "lowcmd", "q": [18], "mode": [18]}`  
BasCmd packet: `{"type": "basecmd", "vx": float, "vy": float, "omega": float}`

---

## Prerequisites — Isaac Sim machine (jqr)

| Requirement | Notes |
|-------------|-------|
| Isaac Lab (`env_isaaclab` conda env) | `~/miniforge3/envs/env_isaaclab/` |
| ROS2 Humble | `/opt/ros/humble/` |
| `ros-humble-rmw-cyclonedds-cpp` | Must match the control PC's DDS vendor |
| Python ZMQ | `conda run -n env_isaaclab pip install pyzmq` |

Install the missing DDS package if not already done:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

---

## Repository layout

```
~/topstar_ros2/
  example/
    isaac_bridge/
      launch_h1_bridge.sh       # one-shot launcher (runs both processes)
      h1_isaac_sim.py           # Process 1 — Isaac Sim physics loop
      h1_isaac_ros2_bridge.py   # Process 2 — ROS2 ↔ ZMQ bridge
      make_abs_urdf.py          # generate h1_abs.urdf (run once after sync)
      topstar_h1.py             # ArticulationCfg (TOPSTAR_H1_CFG) with tuned kd gains — import in RL tasks instead of h1_isaac_sim.py's inline H1_ROBOT_CFG
    src/
      urdf/h1/
        Topstar.urdf            # source URDF (package:// mesh URIs)
        meshes/                 # STL mesh files
        h1_abs.urdf             # GENERATED — absolute mesh paths, git-ignored
  cyclonedds_ws/
    install/                    # pre-built topstar_hg/topstar_api Python bindings
    src/topstar/                # message source
  sync_to_jqr.sh                # deploy script
```

---

## First-time deployment

Run from the **control PC**:

```bash
bash ~/topstar_ros2/sync_to_jqr.sh
```

This script:
1. `rsync`s `~/topstar_ros2/` to `jqr@192.168.1.30:~/topstar_ros2/` (excludes build artifacts and `h1_abs.urdf`)
2. SSHs into jqr and runs `make_abs_urdf.py` to generate `h1_abs.urdf` with absolute mesh paths for that machine

### Updating an existing deployment

```bash
bash ~/topstar_ros2/sync_to_jqr.sh
```

Same command — rsync is incremental. `h1_abs.urdf` is regenerated automatically.

---

## Running the simulation

On **jqr**:

```bash
# GUI window (default)
bash ~/topstar_ros2/example/isaac_bridge/launch_h1_bridge.sh

# Headless (no window)
bash ~/topstar_ros2/example/isaac_bridge/launch_h1_bridge.sh --headless
```

The launcher:
1. Starts `h1_isaac_sim.py` via the `env_isaaclab` conda Python
2. Waits 25 s for Isaac Sim to initialise and load the scene
3. Detects the LAN interface (the one that reaches 192.168.1.0/24)
4. Starts `h1_isaac_ros2_bridge.py` with CycloneDDS bound to that interface
5. Ctrl-C shuts down both processes cleanly

Expected output:
```
[launch] Starting Isaac Sim ...
[launch] Isaac Sim PID=12345
[launch] Waiting 25 s for Isaac Sim to start ...
[launch] Starting ROS2 bridge (CycloneDDS, iface=enp3s0) ...
[launch] ROS2 bridge PID=12399
[launch] Both processes running. Ctrl-C to stop.
[launch] Topics: /h1/lowstate (pub), /h1/lowcmd (sub), /h1/base_cmd (sub)
[H1 Isaac] Ready — state on :15555, commands on :15556
```

---

## Controlling the robot from the control PC

### Build (one-time, or after code changes)

```bash
source ~/topstar_ros2/setup.sh        # Ethernet DDS (two-machine)
# or: source ~/topstar_ros2/setup_local.sh   # loopback DDS (same-machine)
cd ~/topstar_ros2/example
bash build_h1.sh                      # colcon build --packages-select topstar_ros2_example --symlink-install
source install/setup.bash
```

### Two-machine usage (Isaac Sim on jqr, control tools on this PC)

With `launch_h1_bridge.sh` running on jqr, the DDS topics are visible on the LAN. Run any of the control tools directly:

```bash
# Interactive joint jog GUI
ros2 run topstar_ros2_example h1_upper_body_jog

# Drive + arm-wave demo
ros2 run topstar_ros2_example h1_drive_example
```

### Same-machine usage (Isaac Sim and ROS2 node on the same host)

Start Isaac Sim first, then the ROS2 node in a separate terminal:

```bash
# Terminal 1 — Isaac Sim (just the physics process, no bridge)
conda run -n env_isaaclab \
  python ~/topstar_ros2/example/isaac_bridge/h1_isaac_sim.py --headless

# Terminal 2 — ROS2 node with Isaac backend
source ~/topstar_ros2/setup_local.sh
source ~/topstar_ros2/example/install/setup.bash
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=isaac
```

State publication rate can be overridden: `state_hz:=100`.

### Verify state is arriving

```bash
ros2 topic hz /h1/lowstate        # should read ~50 Hz
ros2 topic echo /h1/lowstate --once
```

---

## ROS2 topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/h1/lowstate` | `topstar_hg/LowState` | sim → PC | 50 Hz |
| `/h1/lowcmd` | `topstar_hg/LowCmd` | PC → sim | on demand |
| `/h1/base_cmd` | `geometry_msgs/Twist` | PC → sim | on demand |

`/h1/lowstate` and `/h1/lowcmd` use `motor_state[0–17]` / `motor_cmd[0–17]` (18 slots). The wheel base is driven exclusively via `/h1/base_cmd` (Twist) using swerve drive IK.

---

## Joint index table

| Slot | Joint name | DOF type | Range (hardware convention) |
|------|-----------|----------|-----------------------------|
| 0 | TORSO_LIFT | prismatic | 0 – 0.45 m |
| 1 | TORSO_PITCH | revolute | 0 – 1.658 rad |
| 2 | HEAD_YAW | revolute | −1.571 – 1.571 rad |
| 3 | HEAD_PITCH | revolute | −0.559 – 0.489 rad |
| 4 | RIGHT_SHOULDER_BASE | revolute | −1.57 – 3.14 rad |
| 5 | RIGHT_SHOULDER | revolute | −1.571 – 0.436 rad |
| 6 | RIGHT_ELBOW_YAW | revolute | −1.57 – 1.57 rad |
| 7 | RIGHT_ELBOW | revolute | −1.798 – 0.436 rad |
| 8 | RIGHT_WRIST_YAW | revolute | −2.932 – 2.932 rad |
| 9 | RIGHT_WRIST_PITCH | revolute | −2.356 – 2.356 rad |
| 10 | RIGHT_WRIST_ROLL | revolute | −2.880 – 2.880 rad |
| 11 | LEFT_SHOULDER_BASE | revolute | −1.57 – 3.14 rad |
| 12 | LEFT_SHOULDER | revolute | −1.80 – 0.436 rad |
| 13 | LEFT_ELBOW_YAW | revolute | −1.57 – 1.57 rad |
| 14 | LEFT_ELBOW | revolute | −0.436 – 1.798 rad |
| 15 | LEFT_WRIST_YAW | revolute | −2.932 – 2.932 rad |
| 16 | LEFT_WRIST_PITCH | revolute | −2.356 – 2.356 rad |
| 17 | LEFT_WRIST_ROLL | revolute | −2.880 – 2.880 rad |

### Sign convention

Slots 0, 1, and 3 (TORSO_LIFT, TORSO_PITCH, HEAD_PITCH) have their URDF joint axis pointing opposite to the hardware SDK convention. The bridge applies `hw_q = −urdf_q` automatically on both the command and state paths, so callers always work in hardware convention.

---

## Actuator PD gains

`topstar_h1.py` (`TOPSTAR_H1_CFG`) carries tuned kd values; `h1_isaac_sim.py`'s inline `H1_ROBOT_CFG` still uses older values. Prefer `TOPSTAR_H1_CFG` for new RL tasks.

| Group | Joints | kp | kd (`topstar_h1.py`) | kd (`h1_isaac_sim.py`) |
|-------|--------|----|----|---|
| torso_lift | Robot_Body_Movement_Joint | 5000 N/m | 652 N·s/m | 100 |
| torso_pitch | Robot_Body_Rotation_Joint | 3000 N·m/rad | 257 N·m·s/rad | 80 |
| head | Head_Rotation, Head_Tonod | 500 | 20 | 20 |
| shoulder | R/L Hand_base, Hand_1 | 2000 | 72 | 30 |
| elbow | R/L Hand_2, Hand_3 | 2000 | 30 | 30 |
| wrist | R/L Hand_4–6 | 600 | 20 | 20 |
| wheel_steer | Wheel_Rotation_*_1 | 200 | 10 | 10 |
| wheel_drive | Wheel_Rotation_*_2 | 0 (velocity) | 5 | 5 |

---

## Swerve drive base

`/h1/base_cmd` accepts a `geometry_msgs/Twist`:
- `linear.x` → forward velocity (m/s)
- `linear.y` → lateral velocity (m/s)
- `angular.z` → yaw rate (rad/s)

The bridge applies swerve IK to compute per-wheel steer angles and drive velocities. Wheel layout (base frame, metres):

```
front
 [3] (+0.22, +0.165)    [2] (+0.22, −0.165)
 [1] (−0.22, +0.165)    [0] (−0.22, −0.165)
```

Wheel radius: 0.0625 m.

---

## Phase 1 limitations

- `fix_base=True` — robot is pinned to world; base doesn't translate even when `/h1/base_cmd` is non-zero (wheel joints do turn, but no contact forces move the base)
- IMU linear acceleration is a constant `[0, 0, 9.81]` placeholder
- Torso lift gravity feedforward (208 N bias from MuJoCo) is not applied → slight sag at partial extension

To enable the mobile base: set `fix_base=False` in `H1_ROBOT_CFG` inside `h1_isaac_sim.py` (Phase 2).

---

## Troubleshooting

**`librmw_cyclonedds_cpp.so` not found**
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

**`/h1/lowstate` not received on the control PC**
- Both machines must use the same DDS vendor (CycloneDDS).
- Check that `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set on both sides.
- The bridge auto-detects the LAN interface; verify with `ip route get 192.168.1.1`.

**Mesh resolution errors (`Failed to resolve mesh`)**
- `h1_abs.urdf` has stale or wrong absolute paths. Regenerate:
  ```bash
  python3 ~/topstar_ros2/example/isaac_bridge/make_abs_urdf.py
  ```
- Delete the stale USD cache so Isaac Sim re-imports the URDF:
  ```bash
  rm -rf /tmp/IsaacLab/usd_*/
  ```

**Torso/head joints not responding**
- These joints use a negated sign convention (URDF axis opposite to hardware SDK). The fix is already in `h1_isaac_sim.py` (`HW_TO_URDF_SIGN`). If you see this after a fresh file edit, ensure the sign array is being applied on both command intake and state output.

**`spawn.joint_drive.gains.stiffness` is MISSING**
- `UrdfFileCfg` requires an explicit `joint_drive` block even when `actuators` overrides the gains. The block is present in `H1_ROBOT_CFG`; do not remove it.
