/**
 * @file h2_ls_hand_example.cpp
 * @brief Example/test application for LS dexterous hand control via ROS2
 *
 * This is a test application to verify the LS hand ROS2 integration,
 * using position mode with ROS2 messages.
 *
 * Usage: ros2 run topstar_ros2_example h2_ls_hand_example <L/R>
 *   L - Left hand
 *   R - Right hand
 *
 * Interactive commands:
 *   r - Rotate fingers through range of motion
 *   g - Grip (one-shot close to fixed target)
 *   u - Ungrip (one-shot open to zero position)
 *   s - Stop all motors
 *   p - Print current state
 *   h - Print help
 *   q - Quit
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "topstar_hg/msg/hand_cmd.hpp"
#include "topstar_hg/msg/hand_state.hpp"

// LS Hand constants
#define HAND_DOF 6
#define HAND_SENSOR_COUNT 12

// Topic names matching topstar_bridge
// NOTE: ROS2 rmw_cyclonedds adds "rt/" prefix automatically
static const char* HAND_LEFT_CMD_TOPIC = "hand/left/cmd";
static const char* HAND_LEFT_STATE_TOPIC = "hand/left/state";
static const char* HAND_RIGHT_CMD_TOPIC = "hand/right/cmd";
static const char* HAND_RIGHT_STATE_TOPIC = "hand/right/state";

// State machine states
enum State { INIT, ROTATE, GRIP, UNGRIP, STOP, PRINT };

// LS Hand position limits (encoder counts / 100)
// These are approximate values - adjust based on actual hand configuration
static const int16_t max_position[HAND_DOF] = {10000, 10000, 10000,
                                               10000, 10000, 10000};
static const int16_t min_position[HAND_DOF] = {0, 0, 0, 0, 0, 0};

// topstar_bridge converts ROS HandCmd units to raw LS hand units:
// target_position_raw = target_position_ros * 100
// target_velocity_raw = target_velocity_ros * 10
static constexpr int32_t kBridgePositionScale = 100;
static constexpr int32_t kBridgeVelocityScale = 10;

static constexpr int16_t rawToRosPosition(int32_t raw_position) {
  return static_cast<int16_t>(raw_position / kBridgePositionScale);
}

static constexpr int16_t rawToRosVelocity(int32_t raw_velocity) {
  return static_cast<int16_t>(raw_velocity / kBridgeVelocityScale);
}

/**
 * @brief CRC32 calculation matching topstar_sdk2
 */
static uint32_t calculate_crc32(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }
      xbit >>= 1;
    }
  }
  return CRC32;
}

/**
 * @brief Convert state enum to string
 */
static const char* stateToString(State state) {
  switch (state) {
    case INIT:
      return "INIT";
    case ROTATE:
      return "ROTATE";
    case GRIP:
      return "GRIP";
    case UNGRIP:
      return "UNGRIP";
    case STOP:
      return "STOP";
    case PRINT:
      return "PRINT";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Get non-blocking keyboard input
 */
static char getNonBlockingInput() {
  struct termios oldt{};
  struct termios newt{};
  char ch = 0;
  int oldf = 0;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = static_cast<char>(getchar());

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  return ch;
}

class LSHandController : public rclcpp::Node {
 public:
  explicit LSHandController(const std::string& hand_side)
      : Node("ls_hand_controller"),
        hand_side_(hand_side),
        running_(false),
  current_state_(STOP),
  last_state_(INIT) {
    // Initialize command message - motor_cmd is a fixed-size array
    for (size_t i = 0; i < cmd_msg_.motor_cmd.size(); i++) {
      cmd_msg_.motor_cmd[i].target_position = 0;
      cmd_msg_.motor_cmd[i].target_velocity = 0;
      cmd_msg_.motor_cmd[i].max_current = 0;
      cmd_msg_.motor_cmd[i].mode = 0;
      cmd_msg_.motor_cmd[i].reserve = 0;
    }
    cmd_msg_.control_mode = 0;
    cmd_msg_.enable = 0;
    cmd_msg_.home = 0;
    cmd_msg_.reserve = 0;
    cmd_msg_.crc = 0;

    neutral_position_.fill(0);
    gripped = false;
  }

  ~LSHandController() override { stop(); }

  bool initialize() {
    // Set topic names based on hand side
    std::string cmd_topic_name;
    std::string state_topic_name;
    if (hand_side_ == "L") {
      cmd_topic_name = HAND_LEFT_CMD_TOPIC;
      state_topic_name = HAND_LEFT_STATE_TOPIC;
    } else {
      cmd_topic_name = HAND_RIGHT_CMD_TOPIC;
      state_topic_name = HAND_RIGHT_STATE_TOPIC;
    }

    // Use SensorDataQoS like the working h2_ankle_swing_example
    // Create publisher for commands
    cmd_publisher_ =
        this->create_publisher<topstar_hg::msg::HandCmd>(cmd_topic_name, 10);

    // Create subscriber for state - use SensorDataQoS for compatibility with
    // raw DDS bridge
    state_subscriber_ = this->create_subscription<topstar_hg::msg::HandState>(
        state_topic_name, rclcpp::SensorDataQoS(),
        [this](const topstar_hg::msg::HandState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          current_state_msg_ = *msg;
        });

    RCLCPP_INFO(this->get_logger(),
                "LS Hand Controller initialized for %s hand",
                hand_side_ == "L" ? "LEFT" : "RIGHT");
    RCLCPP_INFO(this->get_logger(), "  Publishing: %s", cmd_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Subscribing: %s",
                state_topic_name.c_str());

    printHelp();
    return true;
  }

  void start() {
    running_ = true;
    control_thread_ = std::thread([this] { controlLoop(); });
    input_thread_ = std::thread([this] { inputLoop(); });
  }

  void stop() {
    running_ = false;

    if (control_thread_.joinable()) {
      control_thread_.join();
    }
    if (input_thread_.joinable()) {
      input_thread_.join();
    }

    // Stop motors before cleanup
    stopMotors();
  }

 private:
  void controlLoop() {
    // The LS hand motion stack is non-real-time: each move command is executed
    // by a worker thread that may take up to ~640 ms (stuck-state recovery).
    // Publishing faster than the worker can consume commands causes the worker
    // to continuously restart, preventing any move from completing.
    // 10 Hz is sufficient for smooth sinusoidal motion updates.
    rclcpp::Rate rate(10);  // 10 Hz — matched to non-RT motion worker

    while (running_ && rclcpp::ok()) {
      // Spin to process callbacks
      rclcpp::spin_some(this->get_node_base_interface());

      State state = current_state_.load();

      if (state != last_state_) {
        std::cout << "\n--- Current State: " << stateToString(state)
                  << " ---\n";
        last_state_ = state;
      }

      switch (state) {
        case INIT:
          stopMotors();
          current_state_ = STOP;
          break;
        case ROTATE:
          if (!isHandReady()) {
            logNotReady("ROTATE");
            stopMotors();
            current_state_ = STOP;
            break;
          }
          rotateMotors();
          break;
        case GRIP:
          if (!isHandReady()) {
            logNotReady("GRIP");
            stopMotors();
            current_state_ = STOP;
            break;
          }
          if (!grip_command_sent_) {
            gripHand();
            grip_command_sent_ = true;
          }
          break;
        case UNGRIP:
          if (!isHandReady()) {
            logNotReady("UNGRIP");
            stopMotors();
            current_state_ = STOP;
            break;
          }
          if (!ungrip_command_sent_) {
            ungripHand();
            ungrip_command_sent_ = true;
          }
          break;
        case STOP:
          stopMotors();
          break;
        case PRINT:
          printState();
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          break;
      }

      rate.sleep();
    }
  }

  void inputLoop() {
    while (running_ && rclcpp::ok()) {
      char ch = getNonBlockingInput();
      if (ch != EOF && ch != 0) {
        switch (ch) {
          case 'q':
            std::cout << "\nExiting..." << std::endl;
            running_ = false;
            break;
          case 'r':
            std::cout << "\nSwitching to ROTATE mode" << std::endl;
            current_state_ = ROTATE;
            break;
          case 'g':
            std::cout << "\nSwitching to GRIP mode" << std::endl;
            grip_command_sent_ = false;
            current_state_ = GRIP;
            break;
          case 'u':
            std::cout << "\nSwitching to UNGRIP mode" << std::endl;
            ungrip_command_sent_ = false;
            current_state_ = UNGRIP;
            break;
          case 'p':
            std::cout << "\nSwitching to PRINT mode" << std::endl;
            current_state_ = PRINT;
            break;
          case 's':
            std::cout << "\nSwitching to STOP mode" << std::endl;
            current_state_ = STOP;
            break;
          case 'h':
            printHelp();
            break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void rotateMotors() {
    // The LS hand non-RT worker needs ~640 ms per move command.
    // Only issue a new target after kRotateDwellMs so the worker has time
    // to complete each move before the next target is queued.
    constexpr int64_t kRotateDwellMs = 1500;
    constexpr int16_t kRotateAmplitude = rawToRosPosition(4000);
    constexpr int16_t kRotateVelocity = rawToRosVelocity(10000);

    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - rotate_last_cmd_time_).count();

    if (elapsed_ms < kRotateDwellMs) {
      return;  // Worker still moving; do not overwrite target
    }

    captureNeutralPositionsIfNeeded();

    // Alternate between neutral (open) and neutral + amplitude (closed)
    int16_t target_offset = rotate_phase_ ? kRotateAmplitude : int16_t{0};
    rotate_phase_ = !rotate_phase_;
    rotate_last_cmd_time_ = now;

    cmd_msg_.enable = 1;
    cmd_msg_.control_mode = 0;  // Position mode
    cmd_msg_.home = 0;

    for (int i = 0; i < HAND_DOF; i++) {
      int32_t target32 = static_cast<int32_t>(neutral_position_[i]) + target_offset;
      cmd_msg_.motor_cmd[i].target_position = static_cast<int16_t>(
          std::clamp(target32,
                     static_cast<int32_t>(min_position[i]),
                     static_cast<int32_t>(max_position[i])));
      cmd_msg_.motor_cmd[i].target_velocity = kRotateVelocity;
      cmd_msg_.motor_cmd[i].max_current = 500;
      cmd_msg_.motor_cmd[i].mode = 0;
    }

    cmd_msg_.crc = calculate_crc32(reinterpret_cast<uint32_t*>(&cmd_msg_),
                                   sizeof(cmd_msg_) / sizeof(uint32_t) - 1);
    cmd_publisher_->publish(cmd_msg_);

    std::cout << "  ROTATE: " << (rotate_phase_ ? "open" : "closed")
              << " (ROS pos=" << target_offset
              << ", raw=" << (target_offset * kBridgePositionScale) << ")" << std::endl;
  }

  void gripHand() {
    // Match motor_gui raw command semantics through bridge scaling.
    constexpr int16_t kGripTarget = rawToRosPosition(2000);
    constexpr int16_t kGripVelocity = rawToRosVelocity(10000);
    constexpr int16_t kGripMaxCurrent = 800;
    sendOneShotPositionCommand(kGripTarget, kGripVelocity, kGripMaxCurrent);
  }

  void ungripHand() {
    // Return fingers to open reference position.
    constexpr int16_t kUngripTarget = rawToRosPosition(0);
    constexpr int16_t kUngripVelocity = rawToRosVelocity(10000);
    constexpr int16_t kUngripMaxCurrent = 800;
    sendOneShotPositionCommand(kUngripTarget, kUngripVelocity,
                               kUngripMaxCurrent);
  }

  void sendOneShotPositionCommand(int16_t target_position,
                                  int16_t target_velocity,
                                  int16_t max_current) {
    cmd_msg_.enable = 1;
    cmd_msg_.control_mode = 0;  // Position mode
    cmd_msg_.home = 0;

    for (int i = 0; i < HAND_DOF; i++) {
      cmd_msg_.motor_cmd[i].target_position = static_cast<int16_t>(
          std::clamp(static_cast<int32_t>(target_position),
                     static_cast<int32_t>(min_position[i]),
                     static_cast<int32_t>(max_position[i])));
      cmd_msg_.motor_cmd[i].target_velocity = target_velocity;
      cmd_msg_.motor_cmd[i].max_current = max_current;
      cmd_msg_.motor_cmd[i].mode = 0;
    }

    cmd_msg_.crc = calculate_crc32(reinterpret_cast<uint32_t*>(&cmd_msg_),
                                   sizeof(cmd_msg_) / sizeof(uint32_t) - 1);
    cmd_publisher_->publish(cmd_msg_);
  }

  void stopMotors() {
    // Disable motors
    cmd_msg_.enable = 0;
    cmd_msg_.control_mode = 0;
    cmd_msg_.home = 0;

    for (int i = 0; i < HAND_DOF; i++) {
      cmd_msg_.motor_cmd[i].target_position = 0;
      cmd_msg_.motor_cmd[i].target_velocity = 0;
      cmd_msg_.motor_cmd[i].max_current = 0;
      cmd_msg_.motor_cmd[i].mode = 0;
    }

    cmd_msg_.crc = calculate_crc32(reinterpret_cast<uint32_t*>(&cmd_msg_),
                                   sizeof(cmd_msg_) / sizeof(uint32_t) - 1);

    cmd_publisher_->publish(cmd_msg_);
  }

  void printState() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    std::cout << "\033[2J\033[H";  // Clear screen
    std::cout << "=== " << (hand_side_ == "L" ? "LEFT" : "RIGHT")
              << " Hand State ===" << std::endl;
    std::cout << "Hand ID: " << static_cast<int>(current_state_msg_.hand_id)
              << "  DOF: " << static_cast<int>(current_state_msg_.dof_count)
              << "  OP: " << static_cast<int>(current_state_msg_.operational)
              << "  Homed: " << static_cast<int>(current_state_msg_.homed)
              << std::endl;

    std::cout << "\nMotor States:" << std::endl;
    std::cout << "  Motor | Position | Velocity | Current | Status | Error"
              << std::endl;
    std::cout << "  ------|----------|----------|---------|--------|------"
              << std::endl;
    for (size_t i = 0;
         i < current_state_msg_.motor_state.size() && i < HAND_DOF; i++) {
      printf("    %zu   |   %5d  |   %5d  |  %5d  |   0x%02X |  0x%02X\n", i,
             current_state_msg_.motor_state[i].position,
             current_state_msg_.motor_state[i].velocity,
             current_state_msg_.motor_state[i].current,
             current_state_msg_.motor_state[i].status,
             current_state_msg_.motor_state[i].error);
    }

    std::cout << "\nSensor Data (Normal Force):" << std::endl;
    std::cout << "  ";
    for (size_t i = 0;
         i < current_state_msg_.normal_force.size() && i < HAND_SENSOR_COUNT;
         i++) {
      printf("[%2zu]%.1f ", i, current_state_msg_.normal_force[i]);
      if (i == 5) std::cout << "\n  ";
    }
    std::cout << std::endl;

    std::cout << "\nPower: " << current_state_msg_.power_v << "V, "
              << current_state_msg_.power_a << "A" << std::endl;

    if (current_state_msg_.error.size() >= 2) {
      std::cout << "Errors: 0x" << std::hex << current_state_msg_.error[0]
                << " 0x" << current_state_msg_.error[1] << std::dec
                << std::endl;
    }

    printHelp();
  }

  bool isHandReady() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_msg_.operational != 0 && current_state_msg_.homed != 0;
  }

  void captureNeutralPositionsIfNeeded() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (neutral_captured_) {
      return;
    }

    for (size_t i = 0; i < HAND_DOF && i < current_state_msg_.motor_state.size(); i++) {
      neutral_position_[i] = current_state_msg_.motor_state[i].position;
      std::cout << "Captured neutral position for motor " << i << ": "
                << neutral_position_[i] << std::endl;
    }
    neutral_captured_ = true;
  }

  void logNotReady(const char* requested_mode) {
    auto now = std::chrono::steady_clock::now();
    if (now - last_not_ready_log_ < std::chrono::seconds(1)) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_WARN(
        this->get_logger(),
        "Ignoring %s: hand not ready (operational=%u, homed=%u). Staying in STOP.",
        requested_mode, static_cast<unsigned int>(current_state_msg_.operational),
        static_cast<unsigned int>(current_state_msg_.homed));
    last_not_ready_log_ = now;
  }

  static void printHelp() {
    std::cout << "\nCommands:" << std::endl;
    std::cout << "  r - Rotate (slow open/close sweep, 1.5 s dwell)" << std::endl;
    std::cout << "  g - Grip once (raw target 2000)" << std::endl;
    std::cout << "  u - Ungrip once (raw target 0)" << std::endl;
    std::cout << "  p - Print state" << std::endl;
    std::cout << "  s - Stop motors" << std::endl;
    std::cout << "  h - Help" << std::endl;
    std::cout << "  q - Quit" << std::endl;
  }

 public:
  bool isRunning() const { return running_.load() && rclcpp::ok(); }

 private:
  // Member variables
  std::string hand_side_;
  std::atomic<bool> running_;
  std::atomic<State> current_state_;
  State last_state_;
  bool gripped;

  // ROS2 entities
  rclcpp::Publisher<topstar_hg::msg::HandCmd>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<topstar_hg::msg::HandState>::SharedPtr state_subscriber_;

  // Message buffers
  topstar_hg::msg::HandCmd cmd_msg_;
  topstar_hg::msg::HandState current_state_msg_;
  std::array<int16_t, HAND_DOF> neutral_position_;
  bool neutral_captured_ = false;
  bool grip_command_sent_ = false;
  bool ungrip_command_sent_ = false;
  std::mutex state_mutex_;
  std::chrono::steady_clock::time_point last_not_ready_log_ =
      std::chrono::steady_clock::now() - std::chrono::seconds(5);
  // Rotate state
  bool rotate_phase_ = false;  // false = open (neutral), true = closed
  std::chrono::steady_clock::time_point rotate_last_cmd_time_ =
      std::chrono::steady_clock::time_point{};  // epoch → triggers immediately

  // Threads
  std::thread control_thread_;
  std::thread input_thread_;
};

int main(int argc, char** argv) {
  std::cout << "=== H2 LS Hand ROS2 Example ===" << std::endl;
  std::cout << "ROS2-based hand control (position mode)" << std::endl;

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <L/R>" << std::endl;
    std::cerr << "  L - Left hand" << std::endl;
    std::cerr << "  R - Right hand" << std::endl;
    return 1;
  }

  std::string hand_side = argv[1];
  if (hand_side != "L" && hand_side != "R") {
    std::cerr << "Invalid hand side. Please specify 'L' or 'R'." << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto controller = std::make_shared<LSHandController>(hand_side);

  if (!controller->initialize()) {
    std::cerr << "Failed to initialize controller" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  controller->start();

  // Wait for user to quit (controller sets running_ to false on 'q')
  while (controller->isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  controller->stop();
  rclcpp::shutdown();
  std::cout << "Goodbye!" << std::endl;

  return 0;
}
