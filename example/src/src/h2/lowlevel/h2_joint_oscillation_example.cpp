/**
 * This example demonstrates how to use ROS2 to drive a configurable set of H2
 * joints with simultaneous oscillatory motion.
 */
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string_view>
#include <thread>
#include <vector>

#include "common/motor_crc_hg.h"
#include "h2/h2_loco_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topstar_hg/msg/low_cmd.hpp"
#include "topstar_hg/msg/low_state.hpp"

const auto HG_CMD_TOPIC = "lowcmd";
const auto HG_STATE_TOPIC = "lowstate";
constexpr float PI = 3.14159265358979323846F;
constexpr int H2_NUM_MOTOR = 29;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& new_data) {
    std::lock_guard<std::mutex> const lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() {
    std::lock_guard<std::mutex> const lock(mutex_);
    return data_ ? data_ : nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::mutex mutex_;
};

struct MotorCommand {
  std::array<float, H2_NUM_MOTOR> q_target = {};
  std::array<float, H2_NUM_MOTOR> dq_target = {};
  std::array<float, H2_NUM_MOTOR> kp = {};
  std::array<float, H2_NUM_MOTOR> kd = {};
  std::array<float, H2_NUM_MOTOR> tau_ff = {};
};

struct MotorState {
  std::array<float, H2_NUM_MOTOR> q = {};
  std::array<float, H2_NUM_MOTOR> dq = {};
};

const std::array<float, H2_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,     // left leg
    60, 60, 60, 100, 40, 40,     // right leg
    60,                          // waist
    20, 20,                      // head
    40, 40, 40, 40, 40, 40, 40,  // left arm
    40, 40, 40, 40, 40, 40, 40   // right arm
};

const std::array<float, H2_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,    // left leg
    1, 1, 1, 2, 1, 1,    // right leg
    1,                   // waist
    1, 1,                // head
    1, 1, 1, 1, 1, 1, 1, // left arm
    1, 1, 1, 1, 1, 1, 1  // right arm
};

enum class Mode {
  PR = 0,
  AB = 1
};

enum H2JointIndex {
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_ROLL = 5,
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_ROLL = 11,
  WAIST_YAW = 12,
  HEAD_YAW = 13,
  HEAD_PITCH = 14,
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_YAW = 19,
  LEFT_WRIST_PITCH = 20,
  LEFT_WRIST_ROLL = 21,
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_YAW = 26,
  RIGHT_WRIST_PITCH = 27,
  RIGHT_WRIST_ROLL = 28
};

struct JointOscillationSpec {
  std::string_view name;
  int index;
  float min_position;
  float max_position;
  float frequency_hz;
  float phase;
  float kp;
  float kd;
};

struct StartupJointState {
  float initial_position{0.0F};
  float entry_position{0.0F};
};

class H2JointOscillationSender : public rclcpp::Node {
 public:
  H2JointOscillationSender()
      : Node("h2_joint_oscillation_sender"),
        mode_machine_(0),
        loco_client_(this),
        controlled_joints_(CreateJointPlan()) {
    lowstate_subscriber_ = this->create_subscription<topstar_hg::msg::LowState>(
        HG_STATE_TOPIC, rclcpp::SensorDataQoS(),
        [this](topstar_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });

    lowcmd_publisher_ =
        this->create_publisher<topstar_hg::msg::LowCmd>(HG_CMD_TOPIC, 10);

    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                             [this] { Control(); });
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                             [this] { WriteLowCommand(); });
    manual_thread_ = std::thread([this] { ensureManualMode(); });

    LogJointPlan();
  }

  ~H2JointOscillationSender() override {
    stop_manual_thread_ = true;
    if (manual_thread_.joinable()) {
      manual_thread_.join();
    }
  }

 private:
  static std::vector<JointOscillationSpec> CreateJointPlan() {
    // return {
    //     {"right_wrist_pitch", RIGHT_WRIST_PITCH, -0.5F, 0.5F, 0.25F, 0.0F,
    //      Kp[RIGHT_WRIST_PITCH], Kd[RIGHT_WRIST_PITCH]},
    //     {"right_wrist_roll", RIGHT_WRIST_ROLL, -0.5F, 0.5F, 0.25F, 0.0F,
    //      Kp[RIGHT_WRIST_ROLL], Kd[RIGHT_WRIST_ROLL]},
    //     {"left_hip_pitch", LEFT_HIP_PITCH, -1.1F, 0.10F, 0.25F, 0.0F,
    //      Kp[LEFT_HIP_PITCH], Kd[LEFT_HIP_PITCH]},
    //     {"left_knee", LEFT_KNEE, -0.05F, 0.70F, 0.25F, 3.14F, Kp[LEFT_KNEE],
    //      Kd[LEFT_KNEE]},
    //     {"right_hip_pitch", RIGHT_HIP_PITCH, -0.1F, 0.9F, 0.25F, 3.14F,
    //      Kp[RIGHT_HIP_PITCH], Kd[RIGHT_HIP_PITCH]},
    //     {"right_knee", RIGHT_KNEE, -0.05F, 0.60F, 0.25F, 3.14F, Kp[RIGHT_KNEE],
    //      Kd[RIGHT_KNEE]}, 
    //     {"right_shoulder_roll", RIGHT_SHOULDER_ROLL, -0.8F, -0.2F, 0.25F, 3.14F,
    //      Kp[RIGHT_SHOULDER_ROLL], Kd[RIGHT_SHOULDER_ROLL]},
    //     {"left_shoulder_roll", LEFT_SHOULDER_ROLL, 0.2F, 0.8F, 0.25F, 0.0F,
    //      Kp[LEFT_SHOULDER_ROLL], Kd[LEFT_SHOULDER_ROLL]},
    //     {"waist_yaw", WAIST_YAW, -0.2F, 0.2F, 0.25F, 0.0F,
    //      Kp[WAIST_YAW], Kd[WAIST_YAW]},
    //     {"right_shoulder_pitch", RIGHT_SHOULDER_PITCH, -0.3F, 0.3F, 0.25F, 3.14F,
    //      Kp[RIGHT_SHOULDER_PITCH], Kd[RIGHT_SHOULDER_PITCH]},
    //     {"left_shoulder_pitch", LEFT_SHOULDER_PITCH, -0.3F, 0.3F, 0.25F, 0.0F,
    //      Kp[LEFT_SHOULDER_PITCH], Kd[LEFT_SHOULDER_PITCH]},
    // };
    // return {
    //     {"right_shoulder_roll", RIGHT_SHOULDER_ROLL, -1.2F, -0.2F, 0.25F, 0.0F,
    //      Kp[RIGHT_SHOULDER_ROLL], Kd[RIGHT_SHOULDER_ROLL]},
    //     {"right_shoulder_pitch", RIGHT_SHOULDER_PITCH, -0.3F, 0.0F, 0.25F, 0.0F,
    //      Kp[RIGHT_SHOULDER_PITCH], Kd[RIGHT_SHOULDER_PITCH]},
    //     {"right_elbow", RIGHT_ELBOW, 0.0F, 1.0F, 0.25F, 3.14F,
    //      Kp[RIGHT_ELBOW], Kd[RIGHT_ELBOW]},
    //     {"right_shoulder_yaw", RIGHT_SHOULDER_YAW, -0.8F, 0.0F, 0.25F, 0.0F,
    //      Kp[RIGHT_SHOULDER_YAW], Kd[RIGHT_SHOULDER_YAW]},

    //     {"left_shoulder_roll", LEFT_SHOULDER_ROLL, 0.2F, 1.2F, 0.25F, 3.14F,
    //      Kp[LEFT_SHOULDER_ROLL], Kd[LEFT_SHOULDER_ROLL]},
    //     {"left_shoulder_pitch", LEFT_SHOULDER_PITCH, -0.3F, 0.0F, 0.25F, 0.0F,
    //      Kp[LEFT_SHOULDER_PITCH], Kd[LEFT_SHOULDER_PITCH]},
    //     {"left_elbow", LEFT_ELBOW, -1.0F, 0.0F, 0.25F, 0.0F,
    //      Kp[LEFT_ELBOW], Kd[LEFT_ELBOW]},
    //     {"left_shoulder_yaw", LEFT_SHOULDER_YAW, 0.0F, 0.8F, 0.25F, 3.14F,
    //      Kp[LEFT_SHOULDER_YAW], Kd[LEFT_SHOULDER_YAW]},

    //     {"right_hip_roll", RIGHT_HIP_ROLL, -0.8F, 0.0F, 0.25F, 0.0F,
    //      Kp[RIGHT_HIP_ROLL], Kd[RIGHT_HIP_ROLL]},
    //     {"right_knee", RIGHT_KNEE, 0.0F, 0.6F, 0.25F, 3.14F, Kp[RIGHT_KNEE],
    //      Kd[RIGHT_KNEE]},
    //     {"right_ankle_pitch", RIGHT_ANKLE_PITCH, -0.1F, 0.0F, 0.25F, 0.0F,
    //      Kp[RIGHT_ANKLE_PITCH], Kd[RIGHT_ANKLE_PITCH]},
    //     {"left_hip_roll", LEFT_HIP_ROLL, 0.0F, 0.8F, 0.25F, 3.14F,
    //      Kp[LEFT_HIP_ROLL], Kd[LEFT_HIP_ROLL]},
    //     {"left_knee", LEFT_KNEE, 0.0F, 0.6F, 0.25F, 3.14F, Kp[LEFT_KNEE],
    //      Kd[LEFT_KNEE]},
    //     {"left_ankle_pitch", LEFT_ANKLE_PITCH, -0.1F, 0.0F, 0.25F, 0.0F,
    //      Kp[LEFT_ANKLE_PITCH], Kd[LEFT_ANKLE_PITCH]},
    // };
    return {
      {"right_hip_pitch", RIGHT_HIP_PITCH, -0.6F, 0.6F, 0.25F, 0.0F,
        Kp[RIGHT_HIP_PITCH], Kd[RIGHT_HIP_PITCH]},
      {"right_knee", RIGHT_KNEE, 0.0F, 0.6F, 0.25F, 0.0F, Kp[RIGHT_KNEE],
        Kd[RIGHT_KNEE]},
      {"right_ankle_pitch", RIGHT_ANKLE_PITCH, -0.3F, 0.3F, 0.25F, 0.0F,
        Kp[RIGHT_ANKLE_PITCH], Kd[RIGHT_ANKLE_PITCH]},
      {"right_ankle_roll", RIGHT_ANKLE_ROLL, 0.0F, 0.0F, 0.25F, 0.0F,
        Kp[RIGHT_ANKLE_ROLL], Kd[RIGHT_ANKLE_ROLL]},

      {"left_hip_pitch", LEFT_HIP_PITCH, -0.6F, 0.6F, 0.25F, 3.14F,
        Kp[LEFT_HIP_PITCH], Kd[LEFT_HIP_PITCH]},
      {"left_knee", LEFT_KNEE, 0.0F, 0.6F, 0.25F, 3.14F, Kp[LEFT_KNEE],
        Kd[LEFT_KNEE]},
      {"left_ankle_pitch", LEFT_ANKLE_PITCH, -0.3F, 0.3F, 0.25F, 0.0F,
        Kp[LEFT_ANKLE_PITCH], Kd[LEFT_ANKLE_PITCH]},
      {"left_ankle_roll", LEFT_ANKLE_ROLL, 0.0F, 0.0F, 0.25F, 0.0F,
        Kp[LEFT_ANKLE_ROLL], Kd[LEFT_ANKLE_ROLL]},
    };
  
    //     return {
    //     {"right_shoulder_roll", RIGHT_SHOULDER_ROLL, -1.5F, -0.2F, 0.35F, 0.0F,
    //      Kp[RIGHT_SHOULDER_ROLL], Kd[RIGHT_SHOULDER_ROLL]},
    //     {"right_shoulder_pitch", RIGHT_SHOULDER_PITCH, -0.8F, 0.0F, 0.35F, 0.0F,
    //      Kp[RIGHT_SHOULDER_PITCH], Kd[RIGHT_SHOULDER_PITCH]},
    //     {"right_elbow", RIGHT_ELBOW, 0.0F, 1.0F, 0.35F, 3.14F,
    //      Kp[RIGHT_ELBOW], Kd[RIGHT_ELBOW]},
    //     {"right_shoulder_yaw", RIGHT_SHOULDER_YAW, -0.8F, 0.0F, 0.35F, 0.0F,
    //      Kp[RIGHT_SHOULDER_YAW], Kd[RIGHT_SHOULDER_YAW]},
    //     {"right_wrist_roll", RIGHT_WRIST_ROLL, -0.8F, 0.8F, 0.35F, 3.14F,
    //      Kp[RIGHT_WRIST_ROLL], Kd[RIGHT_WRIST_ROLL]},
    //     {"right_wrist_pitch", RIGHT_WRIST_PITCH, -0.8F, 0.8F, 0.35F, 3.14F,
    //      Kp[RIGHT_WRIST_PITCH], Kd[RIGHT_WRIST_PITCH]},

    //     {"left_shoulder_roll", LEFT_SHOULDER_ROLL, 0.2F, 1.5F, 0.35F, 3.14F,
    //      Kp[LEFT_SHOULDER_ROLL], Kd[LEFT_SHOULDER_ROLL]},
    //     {"left_shoulder_pitch", LEFT_SHOULDER_PITCH, -0.8F, 0.0F, 0.35F, 0.0F,
    //      Kp[LEFT_SHOULDER_PITCH], Kd[LEFT_SHOULDER_PITCH]},
    //     {"left_elbow", LEFT_ELBOW, -1.0F, 0.0F, 0.35F, 0.0F,
    //      Kp[LEFT_ELBOW], Kd[LEFT_ELBOW]},
    //     {"left_shoulder_yaw", LEFT_SHOULDER_YAW, 0.0F, 0.8F, 0.35F, 3.14F,
    //      Kp[LEFT_SHOULDER_YAW], Kd[LEFT_SHOULDER_YAW]},
    //     {"left_wrist_roll", LEFT_WRIST_ROLL, -0.8F, 0.8F, 0.35F, 0.0F,
    //      Kp[LEFT_WRIST_ROLL], Kd[LEFT_WRIST_ROLL]},
    //     {"left_wrist_pitch", LEFT_WRIST_PITCH, -0.8F, 0.8F, 0.35F, 3.14F,
    //      Kp[LEFT_WRIST_PITCH], Kd[LEFT_WRIST_PITCH]},

    //     {"waist_yaw", WAIST_YAW, -0.3F, 0.3F, 0.35F, 0.0F,
    //      Kp[WAIST_YAW], Kd[WAIST_YAW]},
    // };
  }

  void LogJointPlan() {
    for (const auto& joint : controlled_joints_) {
      RCLCPP_INFO(this->get_logger(),
                  "Joint plan: %s idx=%d range=[%.3f, %.3f] rad freq=%.3f Hz",
                  joint.name.data(), joint.index, joint.min_position,
                  joint.max_position, joint.frequency_hz);
    }
  }

  void ensureManualMode() {
    while (rclcpp::ok() && !stop_manual_thread_ && !manual_mode_ready_) {
      // If already in Manual mode (e.g. from a previous run killed with Ctrl+C),
      // the sport service rejects SetFsmId(9) with a non-zero error. Check first.
      int current_fsm = -1;
      if (loco_client_.GetFsmId(current_fsm) == 0 && current_fsm == 9) {
        manual_mode_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Robot already in FSM_MANUAL (fsm_id=9)");
        return;
      }

      const int32_t ret = loco_client_.Manual();
      if (ret == 0) {
        manual_mode_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Switched robot to FSM_MANUAL (fsm_id=9) before lowcmd publishing");
        return;
      }

      RCLCPP_WARN(this->get_logger(),
                  "Waiting for /api/sport service to accept FSM_MANUAL (ret=%d)",
                  ret);

      for (int i = 0; i < 5 && rclcpp::ok() && !stop_manual_thread_ &&
                          !manual_mode_ready_;
           ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void Control() {
    if (!manual_mode_ready_ || motor_fault_active_) {
      return;
    }

    const std::shared_ptr<const MotorState> motor_state =
        motor_state_buffer_.GetData();
    if (!motor_state) {
      return;
    }

    if (!startup_state_ready_) {
      CaptureStartupState(*motor_state);
    }
    if (!startup_state_ready_) {
      return;
    }

    MotorCommand command;
    for (int index = 0; index < H2_NUM_MOTOR; ++index) {
      command.q_target.at(index) = startup_positions_.at(index).initial_position;
      command.dq_target.at(index) = 0.0F;
      command.kp.at(index) = 0.0F;
      command.kd.at(index) = 0.0F;
      command.tau_ff.at(index) = 0.0F;
    }

    elapsed_time_ += control_dt_;
    const float settle_ratio = static_cast<float>(
        SmoothStep(clamp(elapsed_time_ / settle_duration_, 0.0, 1.0)));
    const float gain_ratio = static_cast<float>(
        SmoothStep(clamp(elapsed_time_ / gain_ramp_duration_, 0.0, 1.0)));

    for (const auto& joint : controlled_joints_) {
      const float center = 0.5F * (joint.min_position + joint.max_position);
      const float amplitude = 0.5F * (joint.max_position - joint.min_position);
      const StartupJointState& startup = startup_positions_.at(joint.index);
      const float staged_target =
          Lerp(startup.initial_position, startup.entry_position, settle_ratio);

      float target = staged_target;
      if (elapsed_time_ > settle_duration_) {
        const double oscillation_time = elapsed_time_ - settle_duration_;
        const float envelope = static_cast<float>(
            SmoothStep(clamp(oscillation_time / blend_duration_, 0.0, 1.0)));
        const float angle = static_cast<float>(2.0 * PI * joint.frequency_hz *
                                               oscillation_time + joint.phase);
        const float oscillation_target = center + amplitude * std::sin(angle);
        target = Lerp(startup.entry_position, oscillation_target, envelope);
      }

      command.q_target.at(joint.index) = target;
      command.kp.at(joint.index) = gain_ratio * joint.kp;
      command.kd.at(joint.index) = gain_ratio * joint.kd;
    }

    motor_command_buffer_.SetData(command);
  }

  void CaptureStartupState(const MotorState& motor_state) {
    for (const auto& joint : controlled_joints_) {
      StartupJointState startup;
      startup.initial_position = motor_state.q.at(joint.index);

      const float center = 0.5F * (joint.min_position + joint.max_position);
      const float amplitude = 0.5F * (joint.max_position - joint.min_position);
      startup.entry_position =
          center + amplitude * std::sin(joint.phase);
      startup.entry_position = std::clamp(startup.entry_position,
                                          joint.min_position,
                                          joint.max_position);
      startup_positions_.at(joint.index) = startup;
    }

    startup_state_ready_ = true;
  }

  void WriteLowCommand() {
    if (!manual_mode_ready_ || motor_fault_active_) {
      return;
    }

    const std::shared_ptr<const MotorCommand> command =
        motor_command_buffer_.GetData();
    if (!command) {
      return;
    }

    topstar_hg::msg::LowCmd low_command;
    low_command.mode_pr = static_cast<uint8_t>(mode_pr_);
    low_command.mode_machine = mode_machine_;

    for (int index = 0; index < H2_NUM_MOTOR; ++index) {
      low_command.motor_cmd.at(index).mode = 0;
      low_command.motor_cmd.at(index).tau = 0.0F;
      low_command.motor_cmd.at(index).q = 0.0F;
      low_command.motor_cmd.at(index).dq = 0.0F;
      low_command.motor_cmd.at(index).kp = 0.0F;
      low_command.motor_cmd.at(index).kd = 0.0F;
    }

    for (const auto& joint : controlled_joints_) {
      const int index = joint.index;
      low_command.motor_cmd.at(index).mode = 1;
      low_command.motor_cmd.at(index).tau = command->tau_ff.at(index);
      low_command.motor_cmd.at(index).q = command->q_target.at(index);
      low_command.motor_cmd.at(index).dq = command->dq_target.at(index);
      low_command.motor_cmd.at(index).kp = command->kp.at(index);
      low_command.motor_cmd.at(index).kd = command->kd.at(index);
    }

    get_crc(low_command);
    lowcmd_publisher_->publish(low_command);
  }

  void LowStateHandler(topstar_hg::msg::LowState::SharedPtr message) {
    // ── Latency / drop detection ─────────────────────────────────────────────
    {
      const auto now = std::chrono::steady_clock::now();

      // 1. Inter-callback gap (no clock sync needed — detects subscriber stalls)
      if (last_lowstate_time_.time_since_epoch().count() != 0) {
        const long gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_lowstate_time_).count();
        // RCLCPP_WARN(this->get_logger(),
        //             "[LATENCY] lowstate gap %ld ms since last callback", gap_ms);    
        if (gap_ms > 50) {
          RCLCPP_WARN(this->get_logger(),
                      "[LATENCY] lowstate gap %ld ms since last callback", gap_ms);
        }
      }
      last_lowstate_time_ = now;

      // 2. Sequence number drop detection (distinguishes drop from delay)
      const uint32_t seq = message->reserve[3];
      if (lowstate_seq_initialized_) {
        const uint32_t expected = last_lowstate_seq_ + 1;
        if (seq != expected) {
          RCLCPP_WARN(this->get_logger(),
                      "[LATENCY] lowstate seq jump: expected %u got %u (%u dropped)",
                      expected, seq, seq - expected);
        }
      }
      last_lowstate_seq_ = seq;
      lowstate_seq_initialized_ = true;

      // 3. Wall-clock e2e deviation — no NTP needed. First message captures the
      //    static clock skew between machines as a baseline; later messages report
      //    deviation from that baseline, which reflects real latency change.
      const uint64_t write_ns =
          (uint64_t)message->reserve[1] | ((uint64_t)message->reserve[2] << 32);
      if (write_ns != 0) {
        struct timespec ts_now;
        clock_gettime(CLOCK_REALTIME, &ts_now);
        const uint64_t now_ns =
            (uint64_t)ts_now.tv_sec * 1000000000ULL + (uint64_t)ts_now.tv_nsec;
        const int64_t offset_ms =
            (static_cast<int64_t>(now_ns) - static_cast<int64_t>(write_ns)) / 1000000;
        if (!wall_clock_baseline_set_) {
          wall_clock_baseline_ms_ = offset_ms;
          wall_clock_baseline_set_ = true;
          RCLCPP_WARN(this->get_logger(),
                      "[LATENCY] wall-clock baseline offset %ld ms (static clock skew)",
                      offset_ms);
        } else {
          const int64_t deviation_ms = offset_ms - wall_clock_baseline_ms_;
          if (deviation_ms > 50 || deviation_ms < -50) {
            RCLCPP_WARN(this->get_logger(),
                        "[LATENCY] lowstate e2e deviation %ld ms from baseline",
                        deviation_ms);
          }
        }
      }
    }
    // ────────────────────────────────────────────────────────────────────────

    // ── Motor fault detection ────────────────────────────────────────────────
    // Primary: per-motor motorstate field (non-zero = fault code from EtherCAT).
    // Secondary: reserve[0] system flag packed by topstar_bridge
    //   bits 0-7  = motor_fault_active, bits 8-15 = faulted_motor_id.
    const bool sys_fault = (message->reserve[0] & 0xFFU) != 0U;
    const int  sys_fault_id = static_cast<int>((message->reserve[0] >> 8) & 0xFFU);

    int per_motor_fault_id = -1;
    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      if (message->motor_state[i].motorstate != 0U) {
        per_motor_fault_id = i;
        break;
      }
    }

    const bool fault_now = sys_fault || (per_motor_fault_id >= 0);
    if (fault_now) {
      if (!motor_fault_active_.exchange(true)) {
        const int fid = sys_fault ? sys_fault_id : per_motor_fault_id;
        const uint32_t ms = (fid >= 0 && fid < H2_NUM_MOTOR)
                                ? message->motor_state[fid].motorstate : 0U;
        RCLCPP_ERROR(this->get_logger(),
                     "[FAULT] Motor %d fault (motorstate=0x%08x) — stopping oscillation",
                     fid, ms);
      }
    } else {
      if (motor_fault_active_.exchange(false)) {
        RCLCPP_WARN(this->get_logger(), "[FAULT] Motor fault cleared");
      }
    }
    // ────────────────────────────────────────────────────────────────────────

    MotorState motor_state;
    for (int index = 0; index < H2_NUM_MOTOR; ++index) {
      motor_state.q.at(index) = message->motor_state[index].q;
      motor_state.dq.at(index) = message->motor_state[index].dq;
    }
    motor_state_buffer_.SetData(motor_state);
    mode_machine_ = message->mode_machine;

    if (++counter_ % 500 == 0) {
      counter_ = 0;
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(2);
      for (const auto& joint : controlled_joints_) {
        stream << joint.name << ": q=" << motor_state.q.at(joint.index)
               << " dq=" << motor_state.dq.at(joint.index) << "  ";
      }
      RCLCPP_INFO(this->get_logger(), "Controlled joints: %s",
                  stream.str().c_str());
    }
  }

  static double clamp(double value, double low, double high) {
    if (value < low) {
      return low;
    }
    if (value > high) {
      return high;
    }
    return value;
  }

  static float Lerp(float start, float end, float ratio) {
    return start + (end - start) * ratio;
  }

  static double SmoothStep(double ratio) {
    return ratio * ratio * (3.0 - 2.0 * ratio);
  }

  rclcpp::Publisher<topstar_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<topstar_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::thread manual_thread_;

  const double control_dt_{0.002};
  const double settle_duration_{6.0};
  const double blend_duration_{2.0};
  const double gain_ramp_duration_{2.0};
  double elapsed_time_{0.0};
  int32_t counter_{0};
  Mode mode_pr_{Mode::PR};
  std::atomic<uint8_t> mode_machine_;
  std::atomic<bool> manual_mode_ready_{false};
  std::atomic<bool> stop_manual_thread_{false};
  std::atomic<bool> motor_fault_active_{false};
  bool startup_state_ready_{false};

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  std::chrono::steady_clock::time_point last_lowstate_time_{};
  uint32_t last_lowstate_seq_{0};
  bool lowstate_seq_initialized_{false};
  bool wall_clock_baseline_set_{false};
  int64_t wall_clock_baseline_ms_{0};
  topstar::robot::h2::LocoClient loco_client_;
  std::vector<JointOscillationSpec> controlled_joints_;
  std::array<StartupJointState, H2_NUM_MOTOR> startup_positions_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H2JointOscillationSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
