# H2 示例运行教程（中文）

本文基于当前仓库中的 H2 相关源码整理，覆盖：

- C++ 示例（8 个）：
  - read_low_state_hg
  - h2_low_level_example
  - h2_ankle_swing_example
  - h2_joint_oscillation_example
  - h2_arm_sdk_dds_example
  - h2_arm_action_example
  - h2_loco_client_example
  - h2_ls_hand_example
- Python H2 工具（3 个）：
  - h2_motor_plot.py
  - h2_lowcmd_gui.py
  - h2_isaac_jog.py

包归属约定（按你的项目实际使用）：

- 除 `mujoco_ros2_bridge` 外，所有 H2 示例都在 `topstar_ros2_h2_example`
- `mujoco_ros2_bridge` 在 `topstar_ros2_example`

## 1. 运行前准备

### 1.1 环境与构建

```bash
source ~/topstar_ros2/setup.sh

cd ~/topstar_ros2/cyclonedds_ws
colcon build

cd ~/topstar_ros2/example
colcon build
```

环境使用约定：

- 连实机：`source ~/topstar_ros2/setup.sh`
- 本地仿真测试：`source ~/topstar_ros2/setup_local.sh`
- 包构建完成后，只需 source 以上其中一个脚本即可，不需要额外
  `source /opt/ros/...` 或 `source .../install/setup.bash`。

### 1.2 固定包名约定

本教程统一使用以下包名：

```bash
export H2_PKG=topstar_ros2_h2_example
export BRIDGE_PKG=topstar_ros2_example
```

后文统一使用：

```bash
ros2 run $H2_PKG <executable> [args]
```

`mujoco_ros2_bridge` 单独使用：

```bash
ros2 run $BRIDGE_PKG mujoco_ros2_bridge
```

### 1.3 常用终端模板

终端 A（持续）：

```bash
source ~/topstar_ros2/setup.sh
```

## 2. C++ H2 示例总览

### 2.1 read_low_state_hg（状态订阅）

用途：订阅 lowstate 和 bms/state，周期打印 IMU/电机/BMS。

```bash
ros2 run $H2_PKG read_low_state_hg
```

可调项（需改源码重编）：

- HIGH_FREQ：true 订阅 lowstate；false 订阅 lf/lowstate
- INFO_IMU / INFO_MOTOR / INFO_BMS：控制打印类别
- LOWSTATE_LOG_INTERVAL_MS / BMS_LOG_INTERVAL_MS：日志节流周期

适用场景：先验证 DDS 链路是否通，再跑控制类示例。

### 2.2 h2_low_level_example（低层关节控制）

用途：先回零，再做踝关节 PR 摆动，同时带手腕滚转。

```bash
ros2 run $H2_PKG h2_low_level_example
```

选项：无 CLI 参数。

可调项（需改源码重编）：

- duration_（阶段切换时间）
- 摆动幅值（max_P/max_R）
- 关节增益（kp/kd）

注意：程序会尝试切到 FSM_MANUAL（fsm_id=9），未切成功时不会发 lowcmd。

### 2.3 h2_ankle_swing_example（踝关节 PR/AB 切换）

用途：分阶段控制踝关节：

- Stage 1：回零
- Stage 2：PR 模式摆动
- Stage 3：AB 模式摆动

```bash
ros2 run $H2_PKG h2_ankle_swing_example
```

选项：无 CLI 参数。

可调项（需改源码重编）：

- duration_（每阶段时间）
- PR/AB 阶段幅值参数（max_P/max_R/max_A/max_B）

### 2.4 h2_joint_oscillation_example（多关节振荡）

用途：对一组预定义关节做平滑进入 + 振荡测试。

```bash
ros2 run $H2_PKG h2_joint_oscillation_example
```

选项：无 CLI 参数。

可调项（需改源码重编）：

- CreateJointPlan()：关节列表、最小/最大角、频率、相位
- settle_duration_ / blend_duration_ / gain_ramp_duration_

说明：这是最适合做“批量关节扫频”与“联动观测”的示例。

### 2.5 h2_arm_sdk_dds_example（机械臂 DDS 控制序列）

用途：自动执行一组上肢动作序列（抬臂、回位等）。

```bash
ros2 run $H2_PKG h2_arm_sdk_dds_example
```

选项：无 CLI 参数。

可调项（需改源码重编）：

- kp_/kd_
- max_joint_velocity_
- MoveTo() 中各阶段 duration
- target_pos（目标姿态）

### 2.6 h2_arm_action_example（交互式动作触发）

用途：在终端输入 action id，触发预定义动作。

```bash
ros2 run $H2_PKG h2_arm_action_example
```

交互选项：

- 输入 0：打印支持的动作列表
- 输入整数 action_id：执行该动作
- 输入 q：退出

适用场景：快速验证动作库和 action 通道。

### 2.7 h2_loco_client_example（最全命令行参数示例）

用途：通过 CLI 直接调用 locomotion API，是参数最多的 H2 示例。

```bash
ros2 run $H2_PKG h2_loco_client_example --<key>[=<value>] ...
```

支持参数（按功能分组）：

查询类：

- --get_fsm_id
- --get_fsm_mode
- --get_balance_mode
- --get_swing_height
- --get_stand_height
- --get_phase

设置类：

- --set_fsm_id=N
- --set_balance_mode=N
- --set_swing_height=H
- --set_stand_height=H
- --set_speed_mode=N

运动类：

- --start
- --damp
- --squat
- --sit
- --stand_up
- --zero_torque
- --stop_move
- --move="vx vy omega"
- --set_velocity="vx vy omega" 或 --set_velocity="vx vy omega duration"

手臂任务：

- --set_arm_task=N
- --stop_arm_task

步态/模式：

- --balance_stand
- --continuous_gait=true|false|1|0
- --switch_move_mode=true|false|1|0

手势相关：

- --shake_hand[=N|JSON]
- --wave_hand[=bool|JSON]

原始 API：

- --raw_api=ID
- --raw_param='{"k":"v"}'
- 或 --raw_json='{"k":"v"}'

常用示例：

```bash
# 状态切换
ros2 run $H2_PKG h2_loco_client_example --damp              # 进入阻尼模式
ros2 run $H2_PKG h2_loco_client_example --stand_up          # 站立模式
ros2 run $H2_PKG h2_loco_client_example --start             # 进入运动模式

# 查询状态
ros2 run $H2_PKG h2_loco_client_example --get_fsm_id --get_fsm_mode

# 切状态 + 起步
ros2 run $H2_PKG h2_loco_client_example --set_fsm_id=9 --start

# 速度控制（1 秒）
ros2 run $H2_PKG h2_loco_client_example --set_velocity="0.2 0.0 0.0"

# 直接 move
ros2 run $H2_PKG h2_loco_client_example --move="0.1 0.0 0.2"
ros2 run $H2_PKG h2_loco_client_example --stop_move         # 停止移动

# 连续步态开关
ros2 run $H2_PKG h2_loco_client_example --continuous_gait=true

# 站立高度
ros2 run $H2_PKG h2_loco_client_example --set_stand_height=<高度值>
```


实机测试（shake_hand / wave_hand 速度参数调节）：

- 这些参数用于实机动作速度与过渡过程调节（`speed`、`ramp_in`、`ramp_out`、`ramp`、`oscillation_frequency`、`turn_frequency`）。
- 建议先从慢速安全参数开始，再逐步提高速度。

Shake hand：慢速安全起始参数

```bash
ros2 run $H2_PKG h2_loco_client_example --shake_hand='{"data":0,"speed":0.5,"ramp_in":2.0,"ramp_out":2.0,"oscillation_frequency":0.8}'
```

Shake hand：停止

```bash
ros2 run $H2_PKG h2_loco_client_example --shake_hand=1
```

Wave hand：不带转身，降低速度

```bash
ros2 run $H2_PKG h2_loco_client_example --wave_hand='{"data":false,"speed":0.5,"ramp":2.5,"oscillation_frequency":0.7}'
```

Wave hand：带转身，降低速度

```bash
ros2 run $H2_PKG h2_loco_client_example --wave_hand='{"data":true,"speed":0.6,"ramp_in":2.5,"ramp_out":2.5,"oscillation_frequency":0.7,"turn_frequency":0.25}'
```

停止手臂任务

```bash
ros2 run $H2_PKG h2_loco_client_example --set_arm_task=0 
```

### 2.8 h2_ls_hand_example（灵巧手交互控制）

用途：控制左/右手，支持旋转、抓握、放开、状态打印。

```bash
ros2 run $H2_PKG h2_ls_hand_example L
ros2 run $H2_PKG h2_ls_hand_example R
```

启动参数：

- L：左手
- R：右手

交互按键：

- r：ROTATE（慢速开合循环）
- g：GRIP（单次抓握）
- u：UNGRIP（单次张开）
- s：STOP（停止）
- p：打印状态
- h：帮助
- q：退出

注意：手未 operational/homed 时会拒绝动作并停在 STOP。

## 3. Python H2 工具（可视化与调试）

这些工具不一定通过 ros2 run 启动，通常用 python3 直接运行。

### 3.1 h2_motor_plot.py（电机曲线可视化）

```bash
source ~/topstar_ros2/setup.sh   # 本地仿真测试请改用 setup_local.sh
python3 ~/topstar_ros2/example/h2_motor_plot.py [options]
```

参数选项：

- --joints J [J ...]
  - 可填关节索引 0..28
  - 可填组名：left_leg right_leg legs left_arm right_arm arms torso all
- --mode pos|torque|both
- --window SEC（时间窗，默认 10）
- --cols N（子图列数，默认 3）
- --rate HZ（刷新率，默认 10）

示例：

```bash
python3 ~/topstar_ros2/example/h2_motor_plot.py --joints left_leg --mode torque
python3 ~/topstar_ros2/example/h2_motor_plot.py --joints 0 1 2 3 --mode both --window 5
python3 ~/topstar_ros2/example/h2_motor_plot.py --joints legs arms --cols 4 --window 15
```

也可用封装脚本（自动 source 环境）：

```bash
bash ~/topstar_ros2/example/run_motor_plot.sh --joints left_leg --mode torque
```

### 3.2 h2_lowcmd_gui.py（LowCmd GUI 面板）

```bash
source ~/topstar_ros2/setup.sh   # 本地仿真测试请改用 setup_local.sh
python3 ~/topstar_ros2/example/h2_lowcmd_gui.py
```

主要功能选项（GUI 内）：

- Start/Stop Publishing
- Send Once
- E-STOP
- ModePR 下拉选择
- 分组批量 Enable/Disable、零值、恢复默认增益
- 实时查看 q/dq/tau_est

### 3.3 h2_isaac_jog.py（Isaac Sim 关节 Jog）

```bash
source ~/topstar_ros2/setup.sh   # 本地仿真测试请改用 setup_local.sh
python3 ~/topstar_ros2/example/h2_isaac_jog.py
```

主要功能选项（GUI 内）：

- Enable/Disable publishing
- Home All
- E-STOP
- ModePR 切换
- 每关节 Slider + +/- 步进
- Record/Replay/Edit/Delete 关键点
- Save/Load 点位文件

## 4. 推荐调试流程（从易到难）

1. 先跑 read_low_state_hg，确认状态流稳定。
2. 再跑 h2_loco_client_example 做状态切换和基础速度命令。
3. 然后跑 h2_low_level_example / h2_ankle_swing_example 做低层执行链路验证。
4. 需要可视化时用 h2_motor_plot.py 或 h2_lowcmd_gui.py。
5. 机械臂与手部分别用 h2_arm_action_example、h2_arm_sdk_dds_example、h2_ls_hand_example。

## 5. 常见问题

### 5.1 ros2 run 找不到可执行程序

先看当前包里到底安装了哪些可执行程序：

```bash
ros2 pkg executables $H2_PKG
```

若列表为空，多半是 example 工作区未成功构建，或当前终端还未执行
`source ~/topstar_ros2/setup.sh`（本地仿真请改为 `setup_local.sh`）。

### 5.2 H2 命令发出后机器人不动作

- 检查是否已切到手动态（许多示例内部会尝试切 FSM_MANUAL）。
- 用 h2_loco_client_example 先执行状态/模式命令。
- 用 read_low_state_hg 看 mode_machine、电机状态与错误码。

### 5.3 画图窗口无数据

- 确认 lowstate 与 /lowcmd 有流量。
- 确认运行前已 `source ~/topstar_ros2/setup.sh`（本地仿真请用 `setup_local.sh`）。
- 如在远程环境，确认图形转发可用（TkAgg 后端）。
