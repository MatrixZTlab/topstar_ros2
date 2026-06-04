/**
 * This example demonstrates how to use ROS2 to receive low states of Topstar
 * H2 robot
 **/
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "topstar_hg/msg/bms_state.hpp"
#include "topstar_hg/msg/imu_state.hpp"
#include "topstar_hg/msg/low_state.hpp"
#include "topstar_hg/msg/motor_state.hpp"

constexpr bool INFO_IMU = true;    // Set 1 to info IMU states
constexpr bool INFO_MOTOR = true;  // Set 1 to info motor states
constexpr bool INFO_BMS = true;    // Set 1 to info BMS states
constexpr bool HIGH_FREQ = true;
// Set 1 to subscribe to low states with high frequencies (500Hz)
constexpr int LOWSTATE_LOG_INTERVAL_MS = 1000;
constexpr int BMS_LOG_INTERVAL_MS = 1000;

constexpr int H2_NUM_MOTOR = 29;

class LowStateSuber : public rclcpp::Node {
 public:
  LowStateSuber() : Node("low_state_suber") {
    // suber is set to subscribe "lowstate" or "lf/lowstate" (low frequencies)
    const auto* topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }
    suber_ = this->create_subscription<topstar_hg::msg::LowState>(
        topic_name, rclcpp::SensorDataQoS(),
        [this](const topstar_hg::msg::LowState::SharedPtr data) {
          topic_callback(data);
        });

    if (INFO_BMS) {
      bms_suber_ = this->create_subscription<topstar_hg::msg::BmsState>(
          "/bms/state", rclcpp::SensorDataQoS(),
          [this](const topstar_hg::msg::BmsState::SharedPtr data) {
            bms_topic_callback(data);
          });
    }
  }

 private:
  void topic_callback(const topstar_hg::msg::LowState::SharedPtr& data) {
    const auto now = std::chrono::steady_clock::now();
    const bool should_log =
        now - last_lowstate_log_time_ >=
        std::chrono::milliseconds(LOWSTATE_LOG_INTERVAL_MS);

    if (INFO_IMU) {
      imu_ = data->imu_state;

      if (should_log) {
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
    }

    if (INFO_MOTOR) {
      for (int i = 0; i < H2_NUM_MOTOR; i++) {
        motor_[i] = data->motor_state[i];
        if (should_log) {
          RCLCPP_INFO(this->get_logger(),
                      "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                      i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                      motor_[i].tau_est);
        }
      }
    }

    if (should_log) {
      last_lowstate_log_time_ = now;
    }
  }

  void bms_topic_callback(const topstar_hg::msg::BmsState::SharedPtr& data) {
    bms_ = *data;

    const auto now = std::chrono::steady_clock::now();
    if (now - last_bms_log_time_ >=
        std::chrono::milliseconds(BMS_LOG_INTERVAL_MS)) {
      log_bms_state();
      last_bms_log_time_ = now;
    }
  }

  void log_bms_state() {
    RCLCPP_INFO(
        this->get_logger(),
        "BMS -- soc: %u%%; pack_voltage: %.3f V; current: %.3f A; cycle: %u",
        bms_.soc, static_cast<double>(bms_.pack_voltage) / 1000.0,
        static_cast<double>(bms_.current) / 1000.0, bms_.cycle);
    RCLCPP_INFO(
        this->get_logger(),
        "BMS -- remain_cap: %u mAh; full_cap: %u mAh; temp: %d C / %d C",
        bms_.remain_cap, bms_.full_cap, bms_.temperature[0],
        bms_.temperature[1]);
    RCLCPP_INFO(
        this->get_logger(),
        "BMS -- status: [0x%02X 0x%02X 0x%02X]; cell0-3: [%u %u %u %u] mV",
        bms_.status[0], bms_.status[1], bms_.status[2], bms_.cell_vol[0],
        bms_.cell_vol[1], bms_.cell_vol[2], bms_.cell_vol[3]);
  }

  rclcpp::Subscription<topstar_hg::msg::LowState>::SharedPtr suber_;
  rclcpp::Subscription<topstar_hg::msg::BmsState>::SharedPtr bms_suber_;

  topstar_hg::msg::IMUState imu_;
  topstar_hg::msg::MotorState motor_[35];
  topstar_hg::msg::BmsState bms_;
  std::chrono::steady_clock::time_point last_lowstate_log_time_{};
  std::chrono::steady_clock::time_point last_bms_log_time_{};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowStateSuber>());
  rclcpp::shutdown();
  return 0;
}
