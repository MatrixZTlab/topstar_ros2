/**
 * This example demonstrates how to use ROS2 to control ankle commands of
 * Topstar H2 robot
 **/
#include <iomanip>
#include <thread>

#include "common/motor_crc_hg.h"
#include "gamepad.hpp"
#include "h2/h2_loco_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topstar_hg/msg/imu_state.hpp"
#include "topstar_hg/msg/low_cmd.hpp"
#include "topstar_hg/msg/low_state.hpp"

const auto HG_CMD_TOPIC = "lowcmd";
const auto HG_IMU_TORSO = "secondary_imu";
const auto HG_STATE_TOPIC = "lowstate";
constexpr float PI = 3.14159265358979323846F;

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

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::mutex mutex_;
};

const int H2_NUM_MOTOR = 29;

struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
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

// Stiffness for all H2 Joints
const std::array<float, H2_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,     // left leg
    60, 60, 60, 100, 40, 40,     // right leg
    60,                          // waist
    20, 20,                      // head
    40, 40, 40, 40, 40, 40, 40,  // left arm
    40, 40, 40, 40, 40, 40, 40   // right arm
};

// Damping for all H2 Joints
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

// H2 joint indices
enum H2JointIndex {
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_B = 4,
  LEFT_ANKLE_ROLL = 5,
  LEFT_ANKLE_A = 5,
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_B = 10,
  RIGHT_ANKLE_ROLL = 11,
  RIGHT_ANKLE_A = 11,
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

class H2AnkleSwingSender : public rclcpp::Node {
 public:
  H2AnkleSwingSender()
      : Node("h2_ankle_swing_sender"),
        mode_machine_(0),
        loco_client_(this) {
    lowstate_subscriber_ = this->create_subscription<topstar_hg::msg::LowState>(
        HG_STATE_TOPIC, rclcpp::SensorDataQoS(),
        [this](topstar_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });
    imustate_subscriber_ = this->create_subscription<topstar_hg::msg::IMUState>(
        HG_IMU_TORSO, rclcpp::SensorDataQoS(),
        [this](topstar_hg::msg::IMUState::SharedPtr message) {
          ImuStateHandler(message);
        });

    lowcmd_publisher_ =
        this->create_publisher<topstar_hg::msg::LowCmd>(HG_CMD_TOPIC, 10);

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { Control(); });
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { low_commandWriter(); });
    manual_thread_ = std::thread([this] { ensureManualMode(); });
  }

  ~H2AnkleSwingSender() override {
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

    MotorCommand motor_command_tmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      motor_command_tmp.tau_ff.at(i) = 0.0;
      motor_command_tmp.q_target.at(i) = 0.0;
      motor_command_tmp.dq_target.at(i) = 0.0;
      motor_command_tmp.kp.at(i) = Kp[i];
      motor_command_tmp.kd.at(i) = Kd[i];
    }

    if (ms) {
      time_ += control_dt_;
      if (time_ < duration_) {
        // [Stage 1]: set robot to zero posture
        for (int i = 0; i < H2_NUM_MOTOR; ++i) {
          double const ratio =
              clamp(static_cast<float>(time_ / duration_), 0.0, 1.0);
          motor_command_tmp.q_target.at(i) =
              static_cast<float>(1.0 - ratio) * ms->q.at(i);
        }
      } else if (time_ < duration_ * 2) {
        // [Stage 2]: swing ankle using PR mode
        mode_pr_ = Mode::PR;
        double const max_P = 0.42;  // pitch joint limit [rad]
        double const max_R = 0.1;   // roll joint limit [rad]
        double const t = time_ - duration_;
        double const L_P_des = max_P * std::sin(2.0 * PI * t);
        double const L_R_des = max_R * std::sin(2.0 * PI * t);
        double const R_P_des = max_P * std::sin(2.0 * PI * t);
        double const R_R_des = -max_R * std::sin(2.0 * PI * t);

        motor_command_tmp.q_target.at(LEFT_ANKLE_PITCH) =
            static_cast<float>(L_P_des);
        motor_command_tmp.q_target.at(LEFT_ANKLE_ROLL) =
            static_cast<float>(L_R_des);
        motor_command_tmp.q_target.at(RIGHT_ANKLE_PITCH) =
            static_cast<float>(R_P_des);
        motor_command_tmp.q_target.at(RIGHT_ANKLE_ROLL) =
            static_cast<float>(R_R_des);
      } else {
        // [Stage 3]: swing ankle using AB mode
        mode_pr_ = Mode::AB;
        double const max_A = 0.1;                // motor A = roll-side; roll joint limit [rad]
        double const max_B = PI * 10.0 / 180.0; // motor B = pitch-side; within pitch limit [rad]
        double const t = time_ - duration_ * 2;
        double const L_A_des = +max_A * std::sin(M_PI * t);
        double const L_B_des = +max_B * std::sin(M_PI * t + PI);
        double const R_A_des = -max_A * std::sin(M_PI * t);
        double const R_B_des = -max_B * std::sin(M_PI * t + PI);

        motor_command_tmp.q_target.at(LEFT_ANKLE_A) =
            static_cast<float>(L_A_des);
        motor_command_tmp.q_target.at(LEFT_ANKLE_B) =
            static_cast<float>(L_B_des);
        motor_command_tmp.q_target.at(RIGHT_ANKLE_A) =
            static_cast<float>(R_A_des);
        motor_command_tmp.q_target.at(RIGHT_ANKLE_B) =
            static_cast<float>(R_B_des);
      }

      motor_command_buffer_.SetData(motor_command_tmp);
    }
  }

  void low_commandWriter() {
    if (!manual_mode_ready_) {
      return;
    }

    topstar_hg::msg::LowCmd low_command;
    low_command.mode_pr = static_cast<uint8_t>(mode_pr_);
    low_command.mode_machine = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc =
        motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < H2_NUM_MOTOR; i++) {
        low_command.motor_cmd.at(i).mode = 1;
        low_command.motor_cmd.at(i).tau = mc->tau_ff.at(i);
        low_command.motor_cmd.at(i).q = mc->q_target.at(i);
        low_command.motor_cmd.at(i).dq = mc->dq_target.at(i);
        low_command.motor_cmd.at(i).kp = mc->kp.at(i);
        low_command.motor_cmd.at(i).kd = mc->kd.at(i);
      }

      get_crc(low_command);
      lowcmd_publisher_->publish(low_command);
    }
  }

  void LowStateHandler(topstar_hg::msg::LowState::SharedPtr message) {
    MotorState msTmp;
    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      msTmp.q.at(i) = message->motor_state[i].q;
      msTmp.dq.at(i) = message->motor_state[i].dq;
      if ((message->motor_state[i].motorstate != 0U) && i <= RIGHT_ANKLE_ROLL) {
        RCLCPP_INFO(this->get_logger(), "[ERROR] motor %d with code %d", i,
                    message->motor_state[i].motorstate);
      }
    }
    motor_state_buffer_.SetData(msTmp);

    // update gamepad
    topstar::common::Gamepad gamepad;
    topstar::common::REMOTE_DATA_RX rx;
    memcpy(rx.buff, message->wireless_remote.data(), 40);  // NOLINT
    gamepad.update(rx.RF_RX);

    ImuState imuTmp;
    imuTmp.omega = message->imu_state.gyroscope;
    imuTmp.rpy = message->imu_state.rpy;
    imu_state_buffer_.SetData(imuTmp);

    mode_machine_ = message->mode_machine;

    if (++counter_ % 500 == 0) {
      counter_ = 0;
      auto& rpy = message->imu_state.rpy;
      RCLCPP_INFO(this->get_logger(), "IMU.pelvis.rpy: %.2f %.2f %.2f\n",
                  rpy[0], rpy[1], rpy[2]);

      // RC
      RCLCPP_INFO(this->get_logger(), "gamepad.A.pressed: %d\n",
                  static_cast<int>(gamepad.A.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad.B.pressed: %d\n",
                  static_cast<int>(gamepad.B.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad.X.pressed: %d\n",
                  static_cast<int>(gamepad.X.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad.Y.pressed: %d\n",
                  static_cast<int>(gamepad.Y.pressed));

      auto& ms = message->motor_state;
      RCLCPP_INFO(this->get_logger(), "All %d Motors:", H2_NUM_MOTOR);
      std::ostringstream oss;
      oss.str("");
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << static_cast<int32_t>(ms[i].mode) << " ";
      }
      RCLCPP_INFO(this->get_logger(), "mode: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].q << " ";
      }
      RCLCPP_INFO(this->get_logger(), "pos: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].dq << " ";
      }
      RCLCPP_INFO(this->get_logger(), "vel: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < H2_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].tau_est << " ";
      }
      RCLCPP_INFO(this->get_logger(), "tau_est: %s", oss.str().c_str());
    }
  }

  void ImuStateHandler(topstar_hg::msg::IMUState::SharedPtr message) {
    auto& rpy = message->rpy;
    if (counter_ % 500 == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU.torso.rpy: %.2f %.2f %.2f", rpy[0],
                  rpy[1], rpy[2]);
    }
  }

  static double clamp(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  rclcpp::Publisher<topstar_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
  rclcpp::Subscription<topstar_hg::msg::LowState>::SharedPtr lowstate_subscriber_;
  rclcpp::Subscription<topstar_hg::msg::IMUState>::SharedPtr imustate_subscriber_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  std::thread manual_thread_;

  double time_{0.0};
  double control_dt_{0.002};
  double duration_{3.0};
  int32_t counter_{0};
  Mode mode_pr_{Mode::PR};
  std::atomic<uint8_t> mode_machine_;
  std::atomic<bool> manual_mode_ready_{false};
  std::atomic<bool> stop_manual_thread_{false};

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;
  topstar::robot::h2::LocoClient loco_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H2AnkleSwingSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
