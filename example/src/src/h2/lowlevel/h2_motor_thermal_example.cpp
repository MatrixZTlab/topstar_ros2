/**
 * Read and print motor temperature and bus voltage for all 29 H2 joints.
 *
 * Temperature is populated by the RT thread via alternating PDO reads
 * (SE motors) or directly from PDO register 0x277D (LS motors).
 * Voltage comes from SE MOTOR_OR_Vbus (0x07) or LS PDO register 0x6079.
 *
 * Run:
 *   ros2 run topstar_ros2_h2_example h2_motor_thermal_example
 **/
#include <climits>
#include <iomanip>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "topstar_hg/msg/low_state.hpp"

constexpr int H2_NUM_MOTOR = 29;
constexpr double PRINT_INTERVAL_S = 1.0;

static const char* const JOINT_NAMES[H2_NUM_MOTOR] = {
    "L_HIP_PITCH",   "L_HIP_ROLL",    "L_HIP_YAW",    "L_KNEE",
    "L_ANKLE_PITCH", "L_ANKLE_ROLL",  "R_HIP_PITCH",  "R_HIP_ROLL",
    "R_HIP_YAW",     "R_KNEE",        "R_ANKLE_PITCH","R_ANKLE_ROLL",
    "WAIST_YAW",     "HEAD_YAW",      "HEAD_PITCH",
    "L_SHLDR_PITCH", "L_SHLDR_ROLL",  "L_SHLDR_YAW",  "L_ELBOW",
    "L_WRIST_YAW",   "L_WRIST_PITCH", "L_WRIST_ROLL",
    "R_SHLDR_PITCH", "R_SHLDR_ROLL",  "R_SHLDR_YAW",  "R_ELBOW",
    "R_WRIST_YAW",   "R_WRIST_PITCH", "R_WRIST_ROLL",
};

class MotorThermalMonitor : public rclcpp::Node {
 public:
  MotorThermalMonitor() : Node("motor_thermal_monitor") {
    max_temp_.fill(INT16_MIN);
    max_vol_.fill(0.0f);

    subscriber_ = this->create_subscription<topstar_hg::msg::LowState>(
        "lowstate", rclcpp::SensorDataQoS(),
        [this](const topstar_hg::msg::LowState::SharedPtr msg) {
          OnLowState(msg);
        });

    print_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(PRINT_INTERVAL_S * 1000)),
        [this] { PrintStats(); });

    RCLCPP_INFO(this->get_logger(),
                "Subscribing to 'lowstate' — printing every %.1f s",
                PRINT_INTERVAL_S);
  }

 private:
  void OnLowState(const topstar_hg::msg::LowState::SharedPtr& msg) {
    ++msg_count_;
    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      int16_t t = msg->motor_state[i].temperature[0];
      float   v = msg->motor_state[i].vol;
      last_temp_[i] = t;
      last_vol_[i]  = v;
      if (t > max_temp_[i]) max_temp_[i] = t;
      if (v > max_vol_[i])  max_vol_[i]  = v;
    }
    imu_ = msg->imu_state;
  }

  void PrintStats() {
    std::cout << "\n--- Motor Temperature & Voltage"
              << "  (msgs in last " << PRINT_INTERVAL_S << " s: "
              << msg_count_ << ") ---\n";
    std::cout << std::left  << std::setw(18) << "Joint"
              << std::right << std::setw(10) << "Temp(°C)"
              << std::setw(12) << "MaxTemp(°C)"
              << std::setw(10) << "Vol(V)"
              << std::setw(11) << "MaxVol(V)" << "\n";
    std::cout << std::string(61, '-') << "\n";

    for (int i = 0; i < H2_NUM_MOTOR; ++i) {
      std::cout << std::left  << std::setw(18) << JOINT_NAMES[i]
                << std::right << std::setw(10) << last_temp_[i]
                << std::setw(12) << max_temp_[i]
                << std::fixed << std::setprecision(1)
                << std::setw(10) << last_vol_[i]
                << std::setw(11) << max_vol_[i] << "\n";
    }

    std::cout << "\n--- IMU ---\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  RPY        (rad)  roll="  << std::setw(9) << imu_.rpy[0]
              << "  pitch=" << std::setw(9) << imu_.rpy[1]
              << "  yaw="   << std::setw(9) << imu_.rpy[2] << "\n";
    std::cout << "  Gyro      (rad/s) x="     << std::setw(9) << imu_.gyroscope[0]
              << "  y="     << std::setw(9) << imu_.gyroscope[1]
              << "  z="     << std::setw(9) << imu_.gyroscope[2] << "\n";
    std::cout << "  Accel     (m/s²)  x="     << std::setw(9) << imu_.accelerometer[0]
              << "  y="     << std::setw(9) << imu_.accelerometer[1]
              << "  z="     << std::setw(9) << imu_.accelerometer[2] << "\n";
    std::cout << "  Quaternion        w="      << std::setw(9) << imu_.quaternion[0]
              << "  x="     << std::setw(9) << imu_.quaternion[1]
              << "  y="     << std::setw(9) << imu_.quaternion[2]
              << "  z="     << std::setw(9) << imu_.quaternion[3] << "\n";
    std::cout << "  Temperature (°C)  " << imu_.temperature << "\n";
    std::cout << std::flush;

    msg_count_ = 0;
  }

  rclcpp::Subscription<topstar_hg::msg::LowState>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr print_timer_;
  uint32_t msg_count_{0};

  std::array<int16_t, H2_NUM_MOTOR> last_temp_{};
  std::array<float,   H2_NUM_MOTOR> last_vol_{};
  std::array<int16_t, H2_NUM_MOTOR> max_temp_;
  std::array<float,   H2_NUM_MOTOR> max_vol_;
  topstar_hg::msg::IMUState imu_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorThermalMonitor>());
  rclcpp::shutdown();
  return 0;
}
