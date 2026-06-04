# motor_gui.py 使用教程

本教程说明如何从本地终端，通过 SSH X11 转发连接至机器人控制器，远程运行 `motor_gui.py` 图形界面。

---

## 一、前置条件（本地机）

本地需要支持 X11 显示。

- **Linux**：默认支持，无需额外配置。
- **macOS**：需安装 [XQuartz](https://www.xquartz.org/)，安装后重新登录系统。
- **Windows**：需安装 [VcXsrv](https://sourceforge.net/projects/vcxsrv/) 或 [Xming](https://sourceforge.net/projects/xming/)，启动 X server 后再进行以下步骤。

---

## 二、SSH 连接至机器人控制器

在本地终端执行以下命令，`-X` 参数开启 X11 转发：

```bash
ssh -X test@192.168.1.10
```

提示输入密码时，输入：

```
123456
```

> **提示**：若连接缓慢或画面卡顿，可改用 `-XC`（启用压缩）：
> ```bash
> ssh -XC test@192.168.1.10
> ```

连接成功后，终端会显示机器人控制器的命令提示符。

---

## 三、启动 motor_gui.py

### 标准启动命令

配置文件 `ec_rt.conf` 是必填参数，必须通过 `-f` 指定。该文件决定哪些 EtherCAT 主站和关节被激活，GUI 据此只显示实际连接的关节面板。

```bash
cd ~/topstar_h2
python3 python/motor_gui.py -f ec_rt.conf
```

GUI 窗口会弹出在本地屏幕上。

### 启动选项说明

| 选项 | 是否必填 | 说明 |
|------|----------|------|
| `-f <配置文件>` | **必填** | 指定 `ec_rt.conf`，控制显示哪些关节 |
| `-m <XML文件>` | 可选 | MuJoCo 模型文件，用于加载关节限位（默认自动从 `~/topstar_h2/h2_model/urdf/h2.xml` 读取） |
| `-c <校准文件>` | **不建议** | 在 GUI 中加载校准偏移（见下方说明） |

> **关于校准文件（`-c`）**：正常使用时 **不需要** 也 **不应该** 通过 `-c` 指定校准文件。
> 控制器运行时（`ec_rt_thread`）在启动时已自动加载系统默认校准，所有关节的零位偏移和方向已在底层生效。
> 在 GUI 中额外加载校准文件会造成偏移量的重复叠加，导致位置显示和指令出现偏差。
> `-c` 选项仅供校准调试专用，日常操作请勿使用。

---

## 四、配置文件字段说明（`ec_rt.conf`）

配置文件为纯文本格式，`#` 开头为注释，字段格式为 `key = value`。

### 4.1 EtherCAT 主站配置

H2 机器人共有 4 个 EtherCAT 主站（master0～master3），各自管理一组关节：

| 主站 | 对应身体部位 | 关节数量 |
|------|-------------|----------|
| master0 | 左腿（Left Leg） | 最多 6 轴 |
| master1 | 右腿（Right Leg） | 最多 6 轴 |
| master2 | 腰部（Waist） | 最多 1 轴 |
| master3 | 头部 + 左臂 + 右臂（Head / Left Arm / Right Arm），顺序依次排列 | 最多 16 轴 |

每个主站有两个字段：

```
master<N>_enabled   = 1          # 1=启用该主站，0=禁用
master<N>_positions = 0 1 2 ...  # 该主站上各从站的 EtherCAT 总线位置编号
```

- `master<N>_enabled`：设为 `1` 启用，`0` 禁用。禁用后该主站管辖的所有关节不会出现在 GUI 中。
- `master<N>_positions`：空格分隔的整数列表，表示该主站 EtherCAT 总线上各从站（电机驱动器）的物理位置编号。列表中**条目的数量**决定激活的关节数；**顺序**决定关节映射（第1个从站对应该主站的第1个关节，依此类推）。master3 上从站顺序必须严格按照头部、左臂、右臂排列。

### 4.2 灵巧手配置

```
hands_enabled       = 1   # 1=启用 LS 灵巧手，0=禁用
left_hand_position  = 10  # 左手从站在 master3 总线上的位置编号
right_hand_position = 19  # 右手从站在 master3 总线上的位置编号
```

- `hands_enabled`：设为 `0` 时，GUI 的 Left Hand / Right Hand 标签页不会接收到手部数据。
- `left_hand_position` / `right_hand_position`：左/右手驱动器在 master3 EtherCAT 总线上的从站位置编号，需与实际硬件接线顺序一致。

### 4.3 踝关节运动学模式

```
ankle_mode = PR   # PR（并联）或 AB（串联）
```

- `PR`（Parallel/并联）：左右踝各由两个电机并联驱动，`ec_rt_thread` 在内部执行正逆运动学，SHM 中存储逻辑俯仰/侧转（pitch/roll）值。GUI 的 **Zero Ankle Pair** 按钮在此模式下可用，可同步清零两个物理电机编码器。
- `AB`（Series/串联）：两个电机独立驱动，SHM 直接存储各物理电机的原始编码器位置。

### 4.4 完整示例（`ec_rt.conf`）

```ini
# H2 全身配置
master0_enabled   = 1
master0_positions = 0 1 2 3 4 5      # 左腿 6 轴

master1_enabled   = 1
master1_positions = 0 1 2 3 4 5      # 右腿 6 轴

master2_enabled   = 1
master2_positions = 0                           # 腰部 1 轴（WaistYaw）

master3_enabled   = 1
master3_positions = 1 2 3 4 5 6 7 8 9 12 13 14 15 16 17 18  # 头部2轴 + 左臂7轴 + 右臂7轴

hands_enabled     = 1
left_hand_position  = 10
right_hand_position = 19

ankle_mode = PR
```

若只需调试腿部（例如：腰臂断开时），可将 master2/master3 禁用：

```ini
master2_enabled = 0
master3_enabled = 0
hands_enabled   = 0
```

---

## 五、界面功能介绍

### 5.1 连接共享内存

GUI 顶部显示连接状态：

```
● Disconnected
```

点击 **Connect to Shared Memory** 按钮，连接至 `ec_rt_thread` 实时线程的共享内存（`/ec_motor_shm`）。

- 连接成功后，状态变为绿色 `● Connected (29 motors)`，并显示实时循环计数器和 RT 周期时间。
- 若连接失败，说明 `ec_rt_thread` 尚未运行，请先在控制器上启动实时线程。

---

### 5.2 FSM 状态控制栏

连接后，顶部会出现 FSM（有限状态机）控制栏，显示当前机器人状态，并提供以下按钮：

| 按钮 | 功能 |
|------|------|
| **Enter MANUAL Mode** | 切换至手动模式（FSM=9），RT 线程停止覆写关节指令，GUI 接管直接控制 |
| **Zero Torque** | 切换至零力矩模式（FSM=0），所有电机失能 |
| **Damp** | 切换至阻尼模式（FSM=1），高阻尼保持当前姿态 |
| **Calibration…** | 打开关节校准对话框 |

> **操作关节前，务必先点击 Enter MANUAL Mode。** 否则 RT 线程会持续覆写 GUI 发出的指令。

---

### 5.3 身体部位标签页（Body Part Tabs）

主区域按身体部位分为以下标签页，每个标签页包含对应的关节控制面板：

| 标签页 | 关节 |
|--------|------|
| **Left Leg** | 左髋俯仰/侧转/偏转、左膝、左踝俯仰/侧转（共6轴） |
| **Right Leg** | 右髋俯仰/侧转/偏转、右膝、右踝俯仰/侧转（共6轴） |
| **Waist** | 腰部偏转 |
| **Head** | 头部偏转/俯仰 |
| **Left Arm** | 左肩俯仰/侧转/旋转、左肘、左腕旋转/俯仰/侧转（共7轴） |
| **Right Arm** | 右肩俯仰/侧转/旋转、右肘、右腕旋转/俯仰/侧转（共7轴） |

每个标签页顶部有：
- **Enable All** / **Disable All**：一键使能/失能该身体部位所有关节。
- **Zero Ankle Pair**（仅踝关节，PR模式）：同步将左/右踝两个物理电机编码器清零。

---

### 5.4 单关节控制面板

每个关节有独立的控制面板，分为 **Status（状态）** 和 **Control（控制）** 两区：

#### 状态显示

| 字段 | 说明 |
|------|------|
| `Type` | 电机类型：SE（伺服）或 LS（灵巧手） |
| `Pos` | 当前位置（弧度） |
| `Vel` | 当前速度（rad/s） |
| `Tau` | 当前力矩（N·m） |
| `EC` | EtherCAT 状态：OP（绿）/ SAFE-OP（橙）/ 其他（灰） |
| `Err` | 错误码（非零时红色高亮） |
| `ENABLED` / `DISABLED` | 电机使能状态 |

#### 控制操作

1. **Enable Motor**（勾选框）：使能/失能该关节电机，勾选后立即生效。

2. **Zero Encoder**（仅 SE 型电机）：将当前机械位置设为编码器零点。程序自动重试最多 5 次，验证位置回到接近 0（< 0.05 rad）后完成。

3. **控制模式**（Mode 下拉菜单）：

   | 模式 | 说明 |
   |------|------|
   | **Direct（直接）** | 手动指定目标位置、速度、前馈力矩，点击 **Send Once** 发送一次指令 |
   | **Sinusoidal（正弦）** | 按设定的幅值、频率、偏置做正弦运动，点击 ▶ Start 持续运行 |
   | **Velocity（速度）** | 按设定速度连续运动（每20ms积分位置），点击 ▶ Start 持续运行 |

4. **KP / KD**：位置增益与速度阻尼增益，默认值来自机器人配置（不同关节不同）。

5. **▶ Start / ■ Stop**：启动/停止正弦或速度模式的连续运动。

6. **Send Once**：Direct 模式下，将当前面板的目标值发送一次至共享内存。

---

### 5.5 动作录制与回放（Recorder 标签页）

Recorder 标签页可录制多个关节的关键帧，并以样条插值平滑回放。

**录制步骤：**

1. 将机器人手动摆至期望姿态（或通过 GUI 发送指令到位）。
2. 点击 **● Add Point** 捕获所有活跃关节的当前位置为一个关键帧。
3. 重复上述步骤，每个关键帧之间的时间间隔由 **Step (s)** 设置（默认 2 秒）。

**回放步骤：**

1. 录制至少 2 个关键帧后，**▶ Play** 按钮变为可用。
2. 可选择勾选 **Loop** 循环播放，或调整 **Speed** 倍速（1.0 = 实时速度）。
3. 点击 **▶ Play** 开始回放，插值方式自动选择：
   - ≥ 3 个关键帧且安装了 scipy：三次样条插值（更平滑）
   - 其他情况：线性插值
4. 点击 **■ Stop** 随时停止。

**其他操作：**

- **Save… / Load…**：将关键帧序列保存为 JSON 文件，或从文件载入。
- **✕ Delete**：删除选中的关键帧行。
- **Clear All**：清空所有关键帧。

> **注意**：回放时只会控制 Enable 勾选框已勾上的关节，未使能的关节会被跳过。

---

### 5.6 电池状态（Battery 标签页）

实时显示 BMS（电池管理系统）数据：

| 信息 | 说明 |
|------|------|
| Pack Voltage | 总电压（V） |
| SOC | 电量百分比（>80% 绿色，20-80% 橙色，<20% 红色） |
| Current | 电流，正值=充电，负值=放电 |
| Remaining Cap / Full Capacity | 剩余/满电容量（mAh） |
| Cycles | 充放电循环次数 |
| Cell Voltages | 最小/最大/平均/差值（mV），差值 >10mV 时红色警告 |
| Temperatures | NTC1 / NTC2 温度（°C） |
| Status | 保护告警信息（如无告警显示 ✓ OK） |

---

### 5.7 灵巧手控制（Left Hand / Right Hand 标签页）

用于控制 LS 灵巧手（6 自由度，每只手）：

**状态显示：**
- 手的 ID、Master/Slave 索引
- 运行状态：OP（运行）、Enabled（使能）、Homed（已回零）、DOF 数量
- 控制模式、错误码、告警标志

**控制操作：**

| 按钮/控件 | 说明 |
|-----------|------|
| **Control Mode** | 选择 Position / Velocity / Torque 模式 |
| **Enable / Disable** | 使能/失能灵巧手 |
| **Home** | 发送回零指令（RT 侧执行 home_motors） |
| **Send Finger Command** | 将当前面板的目标位置/速度/最大电流发送至共享内存 |

每根手指（1-6）可独立设置：
- **Target Pos**（目标位置，原始编码器单位）
- **Target Vel**（目标速度）
- **Max Cur**（最大电流，mA，默认 800）

---

### 5.8 关节校准（Calibration 对话框）

点击顶部 **Calibration…** 按钮打开：

- **Capture All**：以所有关节当前编码器位置作为 Home 零点偏移量（先将机器人摆至标准零位姿态）。
- **Capture Selected**：只捕获表格中选中行的关节。
- **Clear All**：将所有偏移清零。
- **Direction**：可为每个关节设置电机方向（+1 或 -1）。
- **Save… / Load…**：保存/加载校准文件（纯文本格式，兼容 `ec_rt_thread -k` 选项）。

---

## 六、典型操作流程

```
1. ssh -X test@192.168.1.10                          # 连接控制器
2. cd ~/topstar_h2
3. python3 python/motor_gui.py -f ec_rt.conf         # 启动 GUI（窗口出现在本地屏幕）
4. 点击 "Connect to Shared Memory"                    # 连接实时线程
5. 点击 "Enter MANUAL Mode"                           # 切换至手动控制
6. 选择目标身体部位标签页
7. 勾选 "Enable Motor"                                # 使能关节
8. 选择控制模式，设置参数，点击 Send Once 或 ▶ Start
9. 操作完毕后点击 "Zero Torque"                       # 安全停止
```

---

## 七、常见问题

**Q：GUI 窗口不弹出 / 报错 `cannot connect to X server`**
> 检查本地 X server 是否启动（macOS 需运行 XQuartz）；确认 SSH 连接时加了 `-X` 参数。

**Q：点击 Connect 后提示 `Failed to connect - is ec_rt_thread running?`**
> 需要先在控制器上启动 `ec_rt_thread`（或 `ec_rt_thread_v2`）实时线程，GUI 才能访问共享内存。

**Q：发送指令后关节没有响应**
> 确认已点击 **Enter MANUAL Mode**；若 FSM 不在 MANUAL 状态，RT 线程会持续覆写 GUI 指令。

**Q：Recorder 播放时提示缺少 numpy**
> 在控制器上安装依赖：`pip3 install numpy scipy`
