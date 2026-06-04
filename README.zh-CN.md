# Topstar ROS2 工作区

本仓库包含 Topstar 机器人与 ROS 2 / CycloneDDS 通信所需的接口包与示例节点。

支持两类机器人：

- H1：18 自由度轮式人形（上半身 + 四轮转向底盘），Python 实现
- H2：双足人形，C++ 实现

对应英文文档： [README.md](README.md)

H2 示例专项教程： [H2_EXAMPLES_TUTORIAL.zh-CN.md](H2_EXAMPLES_TUTORIAL.zh-CN.md)

## 仓库结构

```text
topstar_ros2/
├── cyclonedds_ws/          # ROS2 接口包（H1/H2 共用）
│   └── src/topstar/
│       ├── topstar_hg/     # 底层消息定义
│       └── topstar_api/    # API 请求/响应消息定义
├── example/
│   ├── src/                # topstar_ros2_example（H1 Python 节点 + mujoco_ros2_bridge）
│   ├── isaac_bridge/       # H1 的 Isaac Sim ↔ ROS2 桥接脚本
│   ├── build_h1.sh         # 仅构建 H1 包
│   ├── h1_tune_env.sh      # H1 增益/调参环境变量
│   ├── h2_motor_plot.py    # H2 电机关节可视化
│   ├── run_motor_plot.sh   # 运行 H2 可视化并自动设置环境
│   ├── test_jog_commands.sh
│   └── test_steering_stability.sh
├── setup.sh                # 实机网络（eno1）
├── setup_local.sh          # 本机回环（lo）仿真
├── setup_default.sh        # ROS2 + CycloneDDS（不强制网卡）
├── sync_to_jqr.sh          # 同步仓库到远端机器（jqr@192.168.1.30）
└── zip_redeploy.sh         # 打包可重部署归档
```

`cyclonedds_ws` 与 `example` 下的生成目录（`build/`、`install/`、`log/`）可删除后重建。

## 依赖

通用依赖：

- Ubuntu 22.04
- ROS 2 Humble
- CycloneDDS RMW：`rmw_cyclonedds_cpp`

```bash
sudo apt update
sudo apt install \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  libyaml-cpp-dev
```

H1（Python，系统 Python 3.10）常见依赖：

- `mujoco`：MuJoCo 后端
- `numpy`：基础数值库
- `scipy >= 1.11`：xapi/硬件后端
- `pyzmq`：Isaac Sim 后端
- `atomics`、`waiting`：xapi 硬件后端
- `xapi`：厂商 wheel 包
- `PySide6`：上半身 jog GUI

> ROS2 Humble 默认使用 `/usr/bin/python3`（3.10.12）。通常直接使用系统 `pip3` 安装。

H2（C++）：除通用依赖外，通常无需额外系统包。

---

## 构建接口包

先构建共享消息包（构建 H1/H2 前都需要）：

```bash
source ~/topstar_ros2/setup.sh
cd ~/topstar_ros2/cyclonedds_ws
colcon build
```

---

## 环境脚本

| 脚本 | DDS 网卡 | 用途 |
|---|---|---|
| `setup.sh` | `eno1`（以太网） | 实机网络 |
| `setup_local.sh` | `lo`（回环） | 本地仿真 |
| `setup_default.sh` | 不指定 | 由 CycloneDDS 自动选择 |

推荐使用方式：

- 连实机：`source ~/topstar_ros2/setup.sh`
- 本地仿真测试：`source ~/topstar_ros2/setup_local.sh`
- 包构建完成后，只需 source 上面其中一个脚本即可。
  不需要再额外执行 `source /opt/ros/...` 或 `source .../install/setup.bash`。

使用 `setup.sh` 前请确认网卡名与机器一致（默认 `eno1`）。

---

## H1 机器人

### 构建

```bash
source ~/topstar_ros2/setup_local.sh   # 实机可改为 setup.sh
cd ~/topstar_ros2/example
bash build_h1.sh
```

### 快速启动

```bash
# MuJoCo（默认）
ros2 launch topstar_ros2_example h1_sim.launch.py viewer:=true

# MuJoCo 无界面
ros2 launch topstar_ros2_example h1_sim.launch.py

# Isaac Sim 后端
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=isaac

# 硬件（xapi）后端
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi

# 上半身 jog GUI（另开终端）
ros2 run topstar_ros2_example h1_upper_body_jog

# 驱动 + 挥臂示例
ros2 run topstar_ros2_example h1_drive_example

# 发送单次底盘速度
ros2 run topstar_ros2_example h1_send_velocity
```

### H1 主要可执行程序

| 可执行程序 | 说明 |
|---|---|
| `h1_ros2_node` | ROS2 桥接节点（ROS2 话题 ↔ 后端） |
| `h1_drive_example` | 驱动 + 挥臂示例 |
| `h1_send_velocity` | 简易底盘速度发送 |
| `h1_upper_body_jog` | PySide6 上半身关节手动调节 GUI |

### H1 关键话题

| Topic | Type | 方向 | 说明 |
|---|---|---|---|
| `/h1/lowcmd` | `topstar_hg/LowCmd` | 订阅 | 上半身 18 关节位置命令 |
| `/h1/base_cmd` | `geometry_msgs/Twist` | 订阅 | 底盘速度命令（`vx`、`vy`、`omega`） |
| `/h1/lowstate` | `topstar_hg/LowState` | 发布 | 关节 + IMU 状态 |
| `/api/arm/request` | `topstar_api/Request` | 订阅 | 机械臂 API 请求 |
| `/api/arm/response` | `topstar_api/Response` | 发布 | 机械臂 API 响应 |

可通过启动参数调整状态发布频率：`state_hz:=100`。

### H1 后端

| 后端 | `backend:=` | 要求 | 场景 |
|---|---|---|---|
| MuJoCo | `mujoco`（默认） | 存在 `~/topstar_mujoco/simulate_python` | 物理仿真 |
| Isaac Sim | `isaac` | Isaac Sim 运行中 + `pyzmq` | 高保真渲染仿真 |
| 硬件 | `xapi` | 厂商 `xapi` wheel + 实机连接 | 实机运行 |

可覆盖 MuJoCo 路径：

```bash
ros2 launch topstar_ros2_example h1_sim.launch.py sim_path:=/other/path
# 或
export TOPSTAR_SIM_PATH=/other/path
```

### H1 硬件后端（xapi）

```bash
pip3 install ~/topstar_ros2/xapi-3.3.8-cp310-cp310-linux_x86_64.whl
```

可选机械臂参数：

```bash
export TOPSTAR_H1_UPPER_BODY_CFG='{"max_speed": 0.5}'
```

### H1 调参

`h1_tune_env.sh` 提供常用增益和安全参数环境变量。先 source 再启动：

```bash
source ~/topstar_ros2/example/h1_tune_env.sh
ros2 launch topstar_ros2_example h1_sim.launch.py backend:=xapi
```

### H1 Isaac Sim 桥接

在 Isaac Sim 机器上启动：

```bash
bash ~/topstar_ros2/example/isaac_bridge/launch_h1_bridge.sh
bash ~/topstar_ros2/example/isaac_bridge/launch_h1_bridge.sh --headless
```

桥接话题：`/h1/lowstate`（发布）、`/h1/lowcmd`（订阅）、`/h1/base_cmd`（订阅）。

同步到远端：

```bash
bash ~/topstar_ros2/sync_to_jqr.sh
bash ~/topstar_ros2/sync_to_jqr.sh user@other-host
```

脚本会排除构建产物，并在远端重新生成 `h1_abs.urdf`。

### H1 Arm API

沿用 `topstar_api` 的 request/response 封装：

- 请求话题：`/api/arm/request`
- 响应话题：`/api/arm/response`
- 通过 `header.identity.id` 匹配请求与响应

当前常用 `api_id`：

- `1001`：`move_joints_timed`，在给定时间内移动 18 个上半身关节

请求 `parameter` JSON 示例：

```json
{ "joints": [<float> x 18], "duration": <float> }
```

响应码：`0` 成功，`1001` 参数错误，`1002` 内部错误。

---

## H2 机器人

### 构建

```bash
source ~/topstar_ros2/setup.sh
cd ~/topstar_ros2/example
colcon build --packages-select topstar_ros2_h2_example
# 或
colcon build
```

### H2 主要可执行程序

| 可执行程序 | 包名 | 说明 |
|---|---|---|
| `read_low_state_hg` | `topstar_ros2_h2_example` | 读取并打印底层状态 |
| `h2_low_level_example` | `topstar_ros2_h2_example` | 低层电机控制示例 |
| `h2_ankle_swing_example` | `topstar_ros2_h2_example` | 踝关节摆动示例 |
| `h2_joint_oscillation_example` | `topstar_ros2_h2_example` | 关节振荡示例 |
| `h2_arm_sdk_dds_example` | `topstar_ros2_h2_example` | 机械臂 SDK DDS 示例 |
| `h2_arm_action_example` | `topstar_ros2_h2_example` | 机械臂动作示例 |
| `h2_loco_client_example` | `topstar_ros2_h2_example` | 运动客户端示例 |
| `h2_ls_hand_example` | `topstar_ros2_h2_example` | LS 手部控制示例 |
| `mujoco_ros2_bridge` | `topstar_ros2_example` | MuJoCo 数字孪生/运动镜像桥 |

运行示例：

```bash
cd ~/topstar_ros2/example
source ~/topstar_ros2/setup.sh
ros2 run topstar_ros2_h2_example h2_joint_oscillation_example
```

### H2 电机关节可视化

```bash
bash ~/topstar_ros2/example/run_motor_plot.sh
bash ~/topstar_ros2/example/run_motor_plot.sh --joints left_leg --mode torque
bash ~/topstar_ros2/example/run_motor_plot.sh --joints legs --cols 4 --window 15
```

### H2 DDS 话题

| Topic | Message | 方向 |
|---|---|---|
| `rt/lowstate` | `topstar_hg::msg::LowState` | Robot -> ROS2 |
| `rt/lowcmd` | `topstar_hg::msg::LowCmd` | ROS2 -> Robot |
| `rt/bms/state` | `topstar_hg::msg::BmsState` | Robot -> ROS2 |
| `rt/bms/cmd` | `topstar_hg::msg::BmsCmd` | ROS2 -> Robot |
| `rt/api/sport/request` | `topstar_api::msg::Request` | ROS2 -> Robot |
| `rt/api/sport/response` | `topstar_api::msg::Response` | Robot -> ROS2 |
| `rt/hand/left/cmd` | `topstar_hg::msg::HandCmd` | ROS2 -> Robot |
| `rt/hand/left/state` | `topstar_hg::msg::HandState` | Robot -> ROS2 |

### MuJoCo 桥接（mujoco_ros2_bridge）

满足以下条件时会随 `topstar_ros2_example` 自动构建：

- 存在 `~/topstar_mujoco/simulate/src/topstar_hg.c`
- CMake 能找到 CycloneDDS（ROS 2 Humble）

运行：

```bash
# 数字孪生：将实机 rt/lowcmd 转发到 MuJoCo
ros2 run topstar_ros2_example mujoco_ros2_bridge

# 运动镜像：同时把实机 lowstate 同步到 MuJoCo viewer
ros2 run topstar_ros2_example mujoco_ros2_bridge
~/topstar_mujoco/simulate/build/topstar_mujoco -n lo --lowstate
```

参数：

- `--robot_interface=IF`：默认 `eno1`（实机 DDS 网卡）
- `--sim_interface=IF`：默认 `lo`（MuJoCo DDS 网卡）

---

## 工具

### 可重部署归档（zip_redeploy.sh）

生成用于复制到另一台机器的干净归档：

```bash
bash ~/topstar_ros2/zip_redeploy.sh
bash ~/topstar_ros2/zip_redeploy.sh custom.zip
```

归档会按目标机器期望的兄弟目录结构打包：

- `topstar_ros2/`：源码、setup 脚本、CycloneDDS 消息工作区、example
- `topstar_mujoco/`：`simulate/`、`simulate_python/`、`topstar_robots/`
- `topstar_h2/h2_model/`：H2 mesh 与 URDF 输入

同时会排除构建/日志/缓存与 `__pycache__`。`topstar_h2/h2_model/urdf/h2_abs.urdf` 也会排除，因为它包含机器相关的绝对路径，需在目标机器重新生成。

默认会在 `~/topstar_ros2` 的同级目录查找 `~/topstar_mujoco` 和 `~/topstar_h2`，如有需要可覆盖：

```bash
TOPSTAR_MUJOCO_DIR=/path/to/topstar_mujoco \
TOPSTAR_H2_DIR=/path/to/topstar_h2 \
bash ~/topstar_ros2/zip_redeploy.sh
```

请将压缩包解压到目标机器的 `~` 下，保持三者同级。

附加行为：

- 如果 `topstar_mujoco/topstar_robots/h1` 中存在损坏的外部 mesh 软链接，脚本会优先尝试使用 `topstar_ros2/example/src/urdf/h1/meshes` 自动回填。
- 若关键依赖路径缺失，脚本会直接报错退出，避免产出“不完整但看似成功”的包。
