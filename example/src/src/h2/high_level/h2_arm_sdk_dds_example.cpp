#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <topstar_hg/msg/low_cmd.hpp>
#include <topstar_hg/msg/low_state.hpp>

#include "h2/h2.hpp"

using namespace std::chrono_literals;
using LowCmd = topstar_hg::msg::LowCmd;
using LowState = topstar_hg::msg::LowState;

// H2 has 7-DOF arms (ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristYaw, WristPitch, WristRoll)
static constexpr int NUM_ARM_JOINTS = 16;  // 7 left + 7 right + waist(1); 7+7+1=15 controlled + waist
// We control: 7 left arm + 7 right arm + WaistYaw = 15 joints
// For consistency with G1 arm_sdk: left arm (7) + right arm (7) + waist (1) = 15
static constexpr int NUM_CONTROLLED_JOINTS = 15;

std::array<H2ArmJointIndex, NUM_CONTROLLED_JOINTS> arm_joints = {
    H2ArmJointIndex::LEFT_SHOULDER_PITCH,
    H2ArmJointIndex::LEFT_SHOULDER_ROLL,
    H2ArmJointIndex::LEFT_SHOULDER_YAW,
    H2ArmJointIndex::LEFT_ELBOW,
    H2ArmJointIndex::LEFT_WRIST_YAW,
    H2ArmJointIndex::LEFT_WRIST_PITCH,
    H2ArmJointIndex::LEFT_WRIST_ROLL,
    H2ArmJointIndex::RIGHT_SHOULDER_PITCH,
    H2ArmJointIndex::RIGHT_SHOULDER_ROLL,
    H2ArmJointIndex::RIGHT_SHOULDER_YAW,
    H2ArmJointIndex::RIGHT_ELBOW,
    H2ArmJointIndex::RIGHT_WRIST_YAW,
    H2ArmJointIndex::RIGHT_WRIST_PITCH,
    H2ArmJointIndex::RIGHT_WRIST_ROLL,
    H2ArmJointIndex::WAIST_YAW};

// Target positions: arms raised (PI/2 elbow)
std::array<float, NUM_CONTROLLED_JOINTS> target_pos = {
    0.0F,  PI_2,  0.0F, -PI_2, 0.0F, 0.0F, 0.0F,  // left arm
    0.0F, -PI_2,  0.0F, -PI_2, 0.0F, 0.0F, 0.0F,  // right arm
    0.0F};                                         // waist yaw

class ArmLowLevelController : public rclcpp::Node {
 public:
  ArmLowLevelController() : Node("arm_lowlevel_controller") {
    pub_ = this->create_publisher<LowCmd>("/arm_sdk", 10);
    sub_ = this->create_subscription<LowState>(
        "/lowstate", rclcpp::SensorDataQoS(),
        [this](const LowState::SharedPtr msg) { StateCallback(msg); });

    sleep_time_ =
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000));

    init_pos_.fill(0.0F);

    thread_ = std::thread([this]() { ControlLoop(); });
  }

 private:
  rclcpp::Publisher<LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<LowState>::SharedPtr sub_;
  std::thread thread_;

  LowState last_state_;
  std::mutex state_mutex_;
  bool state_received_ = false;

  float control_dt_{0.02F};
  float kp_{60.0F}, kd_{1.5F};
  float max_joint_velocity_{0.5F};
  std::chrono::milliseconds sleep_time_{};

  std::array<float, NUM_CONTROLLED_JOINTS> init_pos_{};
  std::array<float, NUM_CONTROLLED_JOINTS> current_jpos_{};

  void StateCallback(const LowState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_ = *msg;

    if (state_received_) {
      return;
    }
    for (size_t i = 0; i < arm_joints.size(); ++i) {
      current_jpos_[i] =
          last_state_.motor_state[static_cast<int>(arm_joints[i])].q;
    }

    state_received_ = true;
  }

  void ControlLoop() {
    while (!state_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waiting for LowState...");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(this->get_logger(), "LowState received. Starting control...");
    StartControlSequence();
  }

  void StartControlSequence() {
    RCLCPP_INFO(this->get_logger(), "Starting control sequence...");
    auto start_pos = current_jpos_;

    RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
    MoveTo(init_pos_, current_jpos_, 3.0F, true);

    RCLCPP_INFO(this->get_logger(), "Lifting arms...");
    MoveTo(target_pos, current_jpos_, 5.0F, false);

    RCLCPP_INFO(this->get_logger(), "Putting arms down...");
    MoveTo(init_pos_, current_jpos_, 5.0F, false);

    RCLCPP_INFO(this->get_logger(), "Returning to start...");
    MoveTo(start_pos, current_jpos_, 3.0F, false);

    StopControl();
  }

  void MoveTo(const std::array<float, NUM_CONTROLLED_JOINTS>& target,
              std::array<float, NUM_CONTROLLED_JOINTS>& current, float duration,
              bool smooth) {
    const int steps = static_cast<int>(duration / control_dt_);
    const float max_delta = max_joint_velocity_ * control_dt_;

    for (int i = 0; i < steps; ++i) {
      float phase = static_cast<float>(i) / static_cast<float>(steps);

      for (size_t j = 0; j < arm_joints.size(); ++j) {
        if (smooth) {
          current[j] = current[j] * (1 - phase) + target[j] * phase;
        } else {
          float diff = target[j] - current[j];
          current[j] += std::clamp(diff, -max_delta, max_delta);
        }
      }

      SendPositionCommand(current);
      std::this_thread::sleep_for(sleep_time_);
    }
  }

  void SendPositionCommand(
      const std::array<float, NUM_CONTROLLED_JOINTS>& positions) {
    LowCmd cmd;

    for (size_t i = 0; i < arm_joints.size(); ++i) {
      int idx = static_cast<int>(arm_joints[i]);
      cmd.motor_cmd[idx].q = positions[i];
      cmd.motor_cmd[idx].dq = 0.0F;
      cmd.motor_cmd[idx].tau = 0.0F;
      cmd.motor_cmd[idx].kp = kp_;
      cmd.motor_cmd[idx].kd = kd_;
    }

    pub_->publish(cmd);
  }

  void StopControl() {
    RCLCPP_INFO(this->get_logger(), "Stopping control...");

    const int steps = static_cast<int>(2.0F / control_dt_);
    const float delta_w = 0.2F * control_dt_;
    float weight = 1.0F;

    for (int i = 0; i < steps; ++i) {
      weight -= delta_w;
      weight = std::clamp(weight, 0.0F, 1.0F);

      LowCmd cmd;

      for (size_t j = 0; j < arm_joints.size(); ++j) {
        int idx = static_cast<int>(arm_joints[j]);
        cmd.motor_cmd[idx].q = current_jpos_[j];
        cmd.motor_cmd[idx].dq = 0.0F;
        cmd.motor_cmd[idx].kp = kp_ * weight;
        cmd.motor_cmd[idx].kd = kd_;
        cmd.motor_cmd[idx].tau = 0.0F;
      }
      pub_->publish(cmd);

      rclcpp::sleep_for(sleep_time_);
    }

    LowCmd cmd;
    pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Control stopped.");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmLowLevelController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
