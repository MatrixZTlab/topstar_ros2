// =============================================================================
// 正弦波电机驱动例程（单文件，结构演示用）
// -----------------------------------------------------------------------------
// 功能：让机器人 29 个电机各自在【自定义的非对称区间 [lower, upper]】内按正弦往复运动。
//      每个电机可【单独】配置：下限 lower、上限 upper、频率 frequency、kp、kd、是否使能 enable。
//      区间参考系由全局 bounds_relative 决定（相对首帧位置 / 绝对电机弧度）。
//      所有配置集中在同目录的 sine_wave_config.yaml，改完重启即可生效（无需改代码）。
//
// 本例程只演示与下位机通信的核心结构，：
//   1) 订阅 lowstate（topstar_hg/msg/LowState）—— 读取每个电机的当前位置/速度/力矩、IMU；
//   2) 发布 lowcmd （topstar_hg/msg/LowCmd） —— 给每个电机下发 位置/速度/力矩/Kp/Kd 指令；
//   3) 发送 LowCmd 前必须调用 get_crc() 计算校验，否则下位机会丢弃该帧。
//
// 通信约定：
//   - 电机数量 TH010_NUM_MOTOR = 29，顺序：
//       左腿 0-5 | 右腿 6-11 | 腰(12)/头yaw(13)/头pitch(14) | 左臂 15-21 | 右臂 22-28
//   - LowState 话题名：HIGH_FREQ 时为 "lowstate"，否则 "lf/lowstate"（可在 yaml 配置）
//   - LowCmd  话题名："/lowcmd"
//   - MotorCmd 字段：mode(1=使能,0=失能), q(位置rad), dq(速度rad/s), tau(前馈力矩Nm), kp, kd
//   - 电机执行的力矩 ≈ kp*(q-q_now) + kd*(dq-dq_now) + tau
//
// ⚠️ 安全 / 注意事项（务必先读）：
//   - 这里发布的 q 是【电机空间原始角度】。框架正常运行时还会做方向(direction)/零位(bias)
//     变换，且踝关节(电机 4,5,10,11)是并联机构需要解算；本例直接对原始电机角加正弦。
//     踝电机已按需求开启(用于测试)，但 q 是原始电机角而非真实踝关节角，请用更小幅值谨慎试验。
//   - 真机需先让机器人处于“接受 /lowcmd 的模式”（如 Manual/FSM 已使能），并把机器人吊起、
//     先用很小的 amplitude 和较小的 kp 试验。
//   - 程序记录“收到首帧 lowstate 时的实际位置”，在 ramp_sec 内从该位置平滑过渡到正弦轨迹，
//     避免上电瞬间跳变；bounds_relative=true 时区间还以该位置为基准。
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <topstar_hg/msg/low_cmd.hpp>
#include <topstar_hg/msg/low_state.hpp>

// 与 legged_system 一致：发送 LowCmd 前计算 CRC。
// 声明在 legged_system/include/motor_crc_hg.h，实现在 legged_system 内。
//   void get_crc(topstar_hg::msg::LowCmd &msg);
#include "motor_crc_hg.h"

namespace {
constexpr int kNumMotor = 29;  // TH010_NUM_MOTOR

// 单个电机的正弦配置
struct MotorCfg {
  std::string name;
  double lower{0.0};       // 运动区间下限 (rad)，可非对称
  double upper{0.0};       // 运动区间上限 (rad)
  double frequency{0.5};   // 正弦频率 (Hz)
  double kp{30.0};
  double kd{1.0};
  bool   enable{true};     // false -> 该电机 mode=0(失能)
};
}  // namespace

class SineWaveExample : public rclcpp::Node {
public:
  SineWaveExample() : rclcpp::Node("sine_wave_example") {
    config_file_ = declare_parameter<std::string>("config_file", "sine_wave_config.yaml");
    loadConfig(config_file_);

    last_q_.fill(0.0f);
    center_.fill(0.0);

    // ---- 通信接口：订阅 lowstate（读状态）、发布 lowcmd（下指令）----
    lowstate_sub_ = create_subscription<topstar_hg::msg::LowState>(
        lowstate_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SineWaveExample::onLowState, this, std::placeholders::_1));
    lowcmd_pub_ = create_publisher<topstar_hg::msg::LowCmd>("/lowcmd", 10);

    // ---- 控制定时器：按 rate_hz_ 周期计算正弦目标并下发 ----
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&SineWaveExample::onTimer, this));

    RCLCPP_INFO(get_logger(),
                "sine_wave_example 启动：等待首帧 %s 后开始；rate=%.0fHz ramp=%.1fs，配置文件=%s",
                lowstate_topic_.c_str(), rate_hz_, ramp_sec_, config_file_.c_str());
  }

private:
  // ====== 读取 yaml，填充每个电机的独立配置 ======
  void loadConfig(const std::string &path) {
    // 先用代码内的安全默认值填满，再用 yaml 覆盖，保证缺项也能跑
    for (int i = 0; i < kNumMotor; ++i) {
      cfg_[i] = MotorCfg{};
      cfg_[i].name = "motor_" + std::to_string(i);  // lower=upper=0 默认不动
    }
    try {
      YAML::Node root = YAML::LoadFile(path);

      if (root["global"]) {
        const auto &g = root["global"];
        if (g["rate_hz"])         rate_hz_         = g["rate_hz"].as<double>();
        if (g["ramp_sec"])        ramp_sec_        = g["ramp_sec"].as<double>();
        if (g["lowstate_topic"])  lowstate_topic_  = g["lowstate_topic"].as<std::string>();
        if (g["bounds_relative"]) bounds_relative_ = g["bounds_relative"].as<bool>();
      }

      // defaults：作为每个电机未显式给出字段的回退值
      MotorCfg def;
      if (root["defaults"]) {
        const auto &d = root["defaults"];
        if (d["lower"])     def.lower     = d["lower"].as<double>();
        if (d["upper"])     def.upper     = d["upper"].as<double>();
        if (d["frequency"]) def.frequency = d["frequency"].as<double>();
        if (d["kp"])        def.kp        = d["kp"].as<double>();
        if (d["kd"])        def.kd        = d["kd"].as<double>();
        if (d["enable"])    def.enable    = d["enable"].as<bool>();
      }
      for (int i = 0; i < kNumMotor; ++i) {
        const std::string nm = cfg_[i].name;
        cfg_[i] = def;
        cfg_[i].name = nm;
      }

      // motors：逐个电机覆盖（按 index 定位）
      if (root["motors"]) {
        for (const auto &m : root["motors"]) {
          if (!m["index"]) continue;
          const int idx = m["index"].as<int>();
          if (idx < 0 || idx >= kNumMotor) continue;
          MotorCfg &c = cfg_[idx];
          if (m["name"])      c.name      = m["name"].as<std::string>();
          if (m["lower"])     c.lower     = m["lower"].as<double>();
          if (m["upper"])     c.upper     = m["upper"].as<double>();
          if (m["frequency"]) c.frequency = m["frequency"].as<double>();
          if (m["kp"])        c.kp        = m["kp"].as<double>();
          if (m["kd"])        c.kd        = m["kd"].as<double>();
          if (m["enable"])    c.enable    = m["enable"].as<bool>();
        }
      }
      RCLCPP_INFO(get_logger(), "已加载电机配置：%s (bounds_relative=%s)",
                  path.c_str(), bounds_relative_ ? "true(相对首帧)" : "false(绝对弧度)");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(),
                   "读取配置文件失败(%s)：%s —— 将以全 0 幅值(不动)运行，请检查 config_file 路径",
                   path.c_str(), e.what());
    }
  }

  // ====== 接收：从 lowstate 读取电机当前状态 ======
  void onLowState(topstar_hg::msg::LowState::SharedPtr msg) {
    mode_machine_ = msg->mode_machine;  // 回传给 lowcmd，保持与下位机一致
    for (int i = 0; i < kNumMotor; ++i) {
      last_q_[i] = msg->motor_state[i].q;   // 也可读 .dq / .tau_est / .temperature / .vol
    }
    if (!state_ready_) {
      // 第一帧：以当前实际位置为每个电机的正弦中心，避免上电瞬间跳变
      for (int i = 0; i < kNumMotor; ++i) center_[i] = msg->motor_state[i].q;
      t0_ = now();
      state_ready_ = true;
      RCLCPP_INFO(get_logger(), "已收到首帧 lowstate：以当前各电机位置为正弦中心，开始运动");
    }
  }

  // ====== 发送：计算正弦目标并发布 lowcmd ======
  void onTimer() {
    if (!state_ready_) return;  // 必须先拿到反馈，保证正弦中心有效

    const double t = (now() - t0_).seconds();
    // 起始过渡：在 ramp_sec 内从首帧实测位置平滑过渡到正弦轨迹。
    // 用 smoothstep(S 形)插值代替线性插值：端点速度为 0，避免 t=0 时刻速度突变导致的跳/顿。
    //   r 线性归一化到 [0,1]，blend = 3r^2 - 2r^3（S 曲线，blend'(0)=blend'(1)=0）
    const double r = (ramp_sec_ > 0.0) ? std::min(1.0, t / ramp_sec_) : 1.0;
    const double blend = r * r * (3.0 - 2.0 * r);

    topstar_hg::msg::LowCmd cmd;  // motor_cmd 为定长 35，只用前 29 个
    cmd.mode_pr = 0;
    cmd.mode_machine = mode_machine_;

    for (int i = 0; i < kNumMotor; ++i) {
      const MotorCfg &c = cfg_[i];
      auto &m = cmd.motor_cmd[i];
      if (!c.enable) {
        m.mode = 0;  // 失能：不参与控制
        m.q = m.dq = m.tau = m.kp = m.kd = 0.0f;
        continue;
      }
      // 非对称区间 [lo, hi]：bounds_relative 时为相对首帧位置的偏移
      const double lo = bounds_relative_ ? center_[i] + c.lower : c.lower;
      const double hi = bounds_relative_ ? center_[i] + c.upper : c.upper;
      const double mid  = 0.5 * (hi + lo);   // 区间中点
      const double half = 0.5 * (hi - lo);   // 半幅
      const double w = 2.0 * M_PI * c.frequency;

      // 目标正弦轨迹（每个电机用自己的区间/频率，互不影响）
      const double target = mid + half * std::sin(w * t);
      // 用 smoothstep 系数在“首帧位置”和“目标轨迹”之间插值，平滑切入（绝对模式下尤其重要）
      const double q_des = (1.0 - blend) * center_[i] + blend * target;

      m.mode = 1;                            // 1=使能
      m.q   = static_cast<float>(q_des);     // 目标位置 (rad)
      m.dq  = 0.0f;                          // 目标速度给 0：kd 项作纯阻尼 kd*(0-dq)，不做速度前馈
      m.tau = 0.0f;                          // 前馈力矩 (Nm)
      m.kp  = static_cast<float>(c.kp);
      m.kd  = static_cast<float>(c.kd);
    }

    get_crc(cmd);                            // ★ 必须：计算 CRC 后下位机才接受该帧
    lowcmd_pub_->publish(cmd);
  }

  // ---- 全局参数 ----
  std::string config_file_;
  double rate_hz_{500.0};
  double ramp_sec_{2.0};
  bool   bounds_relative_{true};  // true: lower/upper 相对首帧位置；false: 绝对电机弧度
  std::string lowstate_topic_{"lowstate"};

  // ---- 每电机配置 ----
  std::array<MotorCfg, kNumMotor> cfg_{};

  // ---- 状态 ----
  bool state_ready_{false};
  rclcpp::Time t0_;
  uint8_t mode_machine_{0};
  std::array<float, kNumMotor> last_q_{};    // 最新反馈位置
  std::array<double, kNumMotor> center_{};   // 每电机正弦中心（=首帧实测位置）

  // ---- 通信 ----
  rclcpp::Subscription<topstar_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Publisher<topstar_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineWaveExample>());
  rclcpp::shutdown();
  return 0;
}
