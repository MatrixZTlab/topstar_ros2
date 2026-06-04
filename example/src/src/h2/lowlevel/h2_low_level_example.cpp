/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of
 * Topstar H2 robot
 **/
#include <thread>

#include "common/motor_crc_hg.h"
#include "h2/h2_loco_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topstar_hg/msg/low_cmd.hpp"
#include "topstar_hg/msg/low_state.hpp"
#include "topstar_hg/msg/motor_cmd.hpp"

constexpr bool INFO_IMU = true;
constexpr bool INFO_MOTOR = true;
constexpr bool HIGH_FREQ = true;

enum PRorAB { PR = 0, AB = 1 };

constexpr int H2_NUM_MOTOR = 29;

// H2 joint indices (29 DOF)
enum H2JointIndex {
  // Left Leg (0-5)
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_ROLL = 5,
  // Right Leg (6-11)
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_ROLL = 11,
  // Waist (12)
  WAIST_YAW = 12,
  // Head (13-14)
  HEAD_YAW = 13,
  HEAD_PITCH = 14,
  // Left Arm (15-21)
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_YAW = 19,
  LEFT_WRIST_PITCH = 20,
  LEFT_WRIST_ROLL = 21,
  // Right Arm (22-28)
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_YAW = 26,
  RIGHT_WRIST_PITCH = 27,
  RIGHT_WRIST_ROLL = 28
};

class LowLevelCmdSender : public rclcpp::Node {
 public:
  LowLevelCmdSender() : Node("low_level_cmd_sender"), loco_client_(this) {
    const auto* topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }

    lowstate_subscriber_ = this->create_subscription<topstar_hg::msg::LowState>(
        topic_name, rclcpp::SensorDataQoS(),
        [this](const topstar_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });

    lowcmd_publisher_ =
        this->create_publisher<topstar_hg::msg::LowCmd>("/lowcmd", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dt_),
                                     [this] { Control(); });
    manual_thread_ = std::thread([this] { ensureManualMode(); });

    time_ = 0;
    duration_ = 3;  // 3 s
  }

  ~LowLevelCmdSender() override {
    stop_manual_thread_ = true;
    if (manual_thread_.joinable()) {
      manual_thread_.join();
    }
  }

 private:
  void ensureManualMode() {
    while (rclcpp::ok() && !stop_manual_thread_ && !manual_mode_ready_) {
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
    if (!manual_mode_ready_) {
      return;
    }

    time_ += control_dt_;
    low_command_.mode_pr = mode_;
    low_command_.mode_machine = mode_machine_;
    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      low_command_.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
      low_command_.motor_cmd[i].tau = 0.0;
      low_command_.motor_cmd[i].q = 0.0;
      low_command_.motor_cmd[i].dq = 0.0;
      low_command_.motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0;
      low_command_.motor_cmd[i].kd = 1.0;
    }

    if (time_ < duration_) {
      // [Stage 1]: set robot to zero posture
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        double const ratio = clamp(time_ / duration_, 0.0, 1.0);
        low_command_.motor_cmd[i].q =
            static_cast<float>((1. - ratio) * motor_[i].q);
      }
    } else {
      // [Stage 2]: swing ankle's PR
      mode_ = PRorAB::PR;
      double const max_P = 0.25;  // [rad] within pitch limit [-0.65, 0.42]
      double const max_R = 0.1;   // [rad] roll joint limit
      double const t = time_ - duration_;
      double const L_P_des = max_P * std::cos(2.0 * M_PI * t);
      double const L_R_des = max_R * std::sin(2.0 * M_PI * t);
      double const R_P_des = max_P * std::cos(2.0 * M_PI * t);
      double const R_R_des = -max_R * std::sin(2.0 * M_PI * t);

      float const Kp_Pitch = 80;
      float const Kd_Pitch = 1;
      float const Kp_Roll = 80;
      float const Kd_Roll = 1;

      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_PITCH].q =
          static_cast<float>(L_P_des);
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_PITCH].dq = 0;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_PITCH].kp = Kp_Pitch;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_PITCH].kd = Kd_Pitch;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_PITCH].tau = 0;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_ROLL].q =
          static_cast<float>(L_R_des);
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_ROLL].dq = 0;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_ROLL].kp = Kp_Roll;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_ROLL].kd = Kd_Roll;
      low_command_.motor_cmd[H2JointIndex::LEFT_ANKLE_ROLL].tau = 0;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_PITCH].q =
          static_cast<float>(R_P_des);
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_PITCH].dq = 0;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_PITCH].kp = Kp_Pitch;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_PITCH].kd = Kd_Pitch;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_PITCH].tau = 0;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_ROLL].q =
          static_cast<float>(R_R_des);
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_ROLL].dq = 0;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_ROLL].kp = Kp_Roll;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_ROLL].kd = Kd_Roll;
      low_command_.motor_cmd[H2JointIndex::RIGHT_ANKLE_ROLL].tau = 0;

      double const max_wrist_roll_angle = 0.5;  // [rad]
      double const WristRoll_des =
          max_wrist_roll_angle * std::sin(2.0 * M_PI * t);
      low_command_.motor_cmd[H2JointIndex::LEFT_WRIST_ROLL].q =
          static_cast<float>(WristRoll_des);
      low_command_.motor_cmd[H2JointIndex::LEFT_WRIST_ROLL].dq = 0;
      low_command_.motor_cmd[H2JointIndex::LEFT_WRIST_ROLL].kp = 50;
      low_command_.motor_cmd[H2JointIndex::LEFT_WRIST_ROLL].kd = 1;
      low_command_.motor_cmd[H2JointIndex::LEFT_WRIST_ROLL].tau = 0;

      low_command_.motor_cmd[H2JointIndex::RIGHT_WRIST_ROLL].q =
          static_cast<float>(WristRoll_des);
      low_command_.motor_cmd[H2JointIndex::RIGHT_WRIST_ROLL].dq = 0;
      low_command_.motor_cmd[H2JointIndex::RIGHT_WRIST_ROLL].kp = 50;
      low_command_.motor_cmd[H2JointIndex::RIGHT_WRIST_ROLL].kd = 1;
      low_command_.motor_cmd[H2JointIndex::RIGHT_WRIST_ROLL].tau = 0;
    }
    get_crc(low_command_);
    lowcmd_publisher_->publish(low_command_);
  }

  void LowStateHandler(const topstar_hg::msg::LowState::SharedPtr& message) {
    mode_machine_ = static_cast<int>(message->mode_machine);
    imu_ = message->imu_state;
    for (int i = 0; i < H2_NUM_MOTOR; i++) {
      motor_[i] = message->motor_state[i];
    }

    if (INFO_IMU) {
      RCLCPP_INFO(this->get_logger(),
                  "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
                  imu_.rpy[1], imu_.rpy[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                  imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
                  imu_.quaternion[3]);
      RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                  imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
      RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                  imu_.accelerometer[0], imu_.accelerometer[1],
                  imu_.accelerometer[2]);
    }
    if (INFO_MOTOR) {
      for (int i = 0; i < H2_NUM_MOTOR; i++) {
        motor_[i] = message->motor_state[i];
        RCLCPP_INFO(this->get_logger(),
                    "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                    motor_[i].tau_est);
      }
    }
  }

  static double clamp(double value, double low, double high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::thread manual_thread_;
  rclcpp::Publisher<topstar_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<topstar_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  topstar_hg::msg::LowCmd low_command_;
  topstar_hg::msg::IMUState imu_;
  std::array<topstar_hg::msg::MotorState, H2_NUM_MOTOR> motor_;
  double control_dt_ = 0.002;
  int timer_dt_ = static_cast<int>(control_dt_ * 1000);
  double time_;
  double duration_;
  PRorAB mode_ = PRorAB::PR;
  int mode_machine_{};
  std::atomic<bool> manual_mode_ready_{false};
  std::atomic<bool> stop_manual_thread_{false};
  topstar::robot::h2::LocoClient loco_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::TimerBase::SharedPtr const timer_;
  auto node = std::make_shared<LowLevelCmdSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
