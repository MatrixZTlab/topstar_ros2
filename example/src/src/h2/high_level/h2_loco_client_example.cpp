#include <chrono>
#include <h2/h2_loco_client.hpp>
#include <iostream>
#include <map>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "common/ut_errror.hpp"
#include "rclcpp/rclcpp.hpp"

class TopstarH2ControlNode : public rclcpp::Node {
 public:
  explicit TopstarH2ControlNode(
      const std::vector<std::pair<std::string, std::string>>& args)
      : Node("topstar_h2_control_node"), args_(args), client_(this) {
    thread_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      ProcessCommands();
    });
  }

  ~TopstarH2ControlNode() override {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  bool handleActionError(int32_t error_code) {
    if (error_code == 0) {
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "Execute action failed, error code: %d",
                 error_code);
    UT_PRINT_ERR(error_code,
                 topstar::robot::h2::UT_ROBOT_LOCO_ERR_LOCOSTATE_NOT_AVAILABLE);
    UT_PRINT_ERR(error_code,
                 topstar::robot::h2::UT_ROBOT_LOCO_ERR_INVALID_FSM_ID);
    UT_PRINT_ERR(error_code,
                 topstar::robot::h2::UT_ROBOT_LOCO_ERR_INVALID_TASK_ID);
    UT_PRINT_ERR(error_code, UT_ROBOT_TASK_TIMEOUT);
    return false;
  }

 private:
  std::thread thread_;

  static bool looksLikeJsonObject(const std::string& s) {
    for (char c : s) {
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        continue;
      }
      return c == '{';
    }
    return false;
  }

  void ProcessCommands() {
    int pending_raw_api = -1;
    std::string pending_raw_param;

    auto flush_raw_call_if_ready = [this, &pending_raw_api,
                                    &pending_raw_param]() -> bool {
      if (pending_raw_api < 0 || pending_raw_param.empty()) {
        return true;
      }
      auto ret = client_.CallRaw(pending_raw_api, pending_raw_param);
      if (!handleActionError(ret)) {
        return false;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Raw API command sent: api_id=%d param=%s",
                  pending_raw_api, pending_raw_param.c_str());
      pending_raw_api = -1;
      pending_raw_param.clear();
      return true;
    };

    for (const auto& arg_pair : args_) {
      RCLCPP_INFO(this->get_logger(),
                  "Processing command: [%s] with param: [%s]...",
                  arg_pair.first.c_str(), arg_pair.second.c_str());

      if (arg_pair.first == "get_fsm_id") {
        int fsm_id = 0;
        auto ret = client_.GetFsmId(fsm_id);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "current fsm_id: %d", fsm_id);
      }

      if (arg_pair.first == "get_fsm_mode") {
        int fsm_mode = 0;
        auto ret = client_.GetFsmMode(fsm_mode);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "current fsm_mode: %d", fsm_mode);
      }

      if (arg_pair.first == "get_balance_mode") {
        int balance_mode = 0;
        auto ret = client_.GetBalanceMode(balance_mode);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "current balance_mode: %d", balance_mode);
      }

      if (arg_pair.first == "get_swing_height") {
        float swing_height = NAN;
        auto ret = client_.GetSwingHeight(swing_height);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "current swing_height: %f", swing_height);
      }

      if (arg_pair.first == "get_stand_height") {
        float stand_height = NAN;
        auto ret = client_.GetStandHeight(stand_height);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "current stand_height: %f", stand_height);
      }

      if (arg_pair.first == "get_phase") {
        std::vector<float> phase;
        auto ret = client_.GetPhase(phase);
        if (!handleActionError(ret)) continue;
        std::ostringstream oss;
        for (size_t i = 0; i < phase.size(); ++i) {
          if (i > 0) oss << ", ";
          oss << phase[i];
        }
        RCLCPP_INFO(this->get_logger(), "current phase: [%s]", oss.str().c_str());
      }

      if (arg_pair.first == "set_fsm_id") {
        int fsm_id = std::stoi(arg_pair.second);
        auto ret = client_.SetFsmId(fsm_id);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set fsm_id to %d", fsm_id);
      }

      if (arg_pair.first == "set_balance_mode") {
        int balance_mode = std::stoi(arg_pair.second);
        auto ret = client_.SetBalanceMode(balance_mode);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set balance_mode to %d", balance_mode);
      }

      if (arg_pair.first == "set_swing_height") {
        float swing_height = std::stof(arg_pair.second);
        auto ret = client_.SetSwingHeight(swing_height);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set swing_height to %f", swing_height);
      }

      if (arg_pair.first == "set_stand_height") {
        float stand_height = std::stof(arg_pair.second);
        auto ret = client_.SetStandHeight(stand_height);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set stand_height to %f", stand_height);
      }

      if (arg_pair.first == "set_velocity") {
        std::vector<float> param = stringToFloatVector(arg_pair.second);
        auto param_size = param.size();
        float vx = NAN, vy = NAN, omega = NAN, duration = NAN;
        if (param_size == 3) {
          vx = param.at(0); vy = param.at(1); omega = param.at(2); duration = 1.F;
        } else if (param_size == 4) {
          vx = param.at(0); vy = param.at(1); omega = param.at(2); duration = param.at(3);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Invalid param size for SetVelocity: %zu", param_size);
          continue;
        }
        auto ret = client_.SetVelocity(vx, vy, omega, duration);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set velocity to %s", arg_pair.second.c_str());
      }

      if (arg_pair.first == "damp") {
        auto ret = client_.Damp();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "Damp command sent");
      }

      if (arg_pair.first == "start") {
        auto ret = client_.Start();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "Start command sent");
      }

      if (arg_pair.first == "squat") {
        auto ret = client_.Squat();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "Squat command sent");
      }

      if (arg_pair.first == "sit") {
        auto ret = client_.Sit();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "Sit command sent");
      }

      if (arg_pair.first == "stand_up") {
        auto ret = client_.StandUp();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "StandUp command sent");
      }

      if (arg_pair.first == "zero_torque") {
        auto ret = client_.ZeroTorque();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "ZeroTorque command sent");
      }

      if (arg_pair.first == "stop_move") {
        auto ret = client_.StopMove();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "StopMove command sent");
      }

      if (arg_pair.first == "move") {
        std::vector<float> param = stringToFloatVector(arg_pair.second);
        auto param_size = param.size();
        if (param_size != 3) {
          RCLCPP_ERROR(this->get_logger(), "Invalid param size for Move: %zu", param_size);
          continue;
        }
        auto ret = client_.Move(param.at(0), param.at(1), param.at(2));
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "Move command sent: %s", arg_pair.second.c_str());
      }

      if (arg_pair.first == "set_arm_task") {
        int task_id = std::stoi(arg_pair.second);
        auto ret = client_.SetArmTask(task_id);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set arm task to %d", task_id);
      }

      if (arg_pair.first == "stop_arm_task") {
        auto ret = client_.SetArmTask(0);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "stop arm task command sent");
      }

      if (arg_pair.first == "set_speed_mode") {
        int speed_mode = std::stoi(arg_pair.second);
        auto ret = client_.SetSpeedMode(speed_mode);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "set speed mode to %d", speed_mode);
      }

      if (arg_pair.first == "balance_stand") {
        auto ret = client_.BalanceStand();
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "BalanceStand command sent");
      }

      if (arg_pair.first == "continuous_gait") {
        bool enable = arg_pair.second == "1" || arg_pair.second == "true";
        auto ret = client_.ContinuousGait(enable);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "ContinuousGait command sent: %s",
                    enable ? "true" : "false");
      }

      if (arg_pair.first == "switch_move_mode") {
        bool enable = arg_pair.second == "1" || arg_pair.second == "true";
        auto ret = client_.SwitchMoveMode(enable);
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "SwitchMoveMode command sent: %s",
                    enable ? "true" : "false");
      }

      if (arg_pair.first == "shake_hand") {
        int32_t ret = 0;
        if (looksLikeJsonObject(arg_pair.second)) {
          ret = client_.ShakeHandRaw(arg_pair.second);
        } else {
          ret = client_.ShakeHand(arg_pair.second.empty() ? 0 : std::stoi(arg_pair.second));
        }
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "ShakeHand command sent");
      }

      if (arg_pair.first == "wave_hand") {
        int32_t ret = 0;
        if (looksLikeJsonObject(arg_pair.second)) {
          ret = client_.WaveHandRaw(arg_pair.second);
        } else {
          bool with_turn = arg_pair.second == "1" || arg_pair.second == "true";
          ret = client_.WaveHand(with_turn);
        }
        if (!handleActionError(ret)) continue;
        RCLCPP_INFO(this->get_logger(), "WaveHand command sent");
      }

      if (arg_pair.first == "raw_api") {
        pending_raw_api = std::stoi(arg_pair.second);
        if (!flush_raw_call_if_ready()) continue;
      }

      if (arg_pair.first == "raw_param" || arg_pair.first == "raw_json") {
        pending_raw_param = arg_pair.second;
        if (!flush_raw_call_if_ready()) continue;
      }

      RCLCPP_INFO(this->get_logger(), "Done processing command: %s",
                  arg_pair.first.c_str());
    }

    if (pending_raw_api >= 0 || !pending_raw_param.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring incomplete raw command (requires both --raw_api and --raw_param)");
    }
    rclcpp::shutdown();
  }

  static std::vector<float> stringToFloatVector(const std::string& str) {
    std::vector<float> result;
    std::stringstream ss(str);
    float num = NAN;
    while (ss >> num) {
      result.push_back(num);
      ss.ignore();
    }
    return result;
  }

  std::vector<std::pair<std::string, std::string>> args_;
  topstar::robot::h2::LocoClient client_;
};

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);

  std::vector<std::pair<std::string, std::string>> args;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.substr(0, 2) == "--") {
      size_t pos = arg.find('=');
      std::string key, value;
      if (pos != std::string::npos) {
        key = arg.substr(2, pos - 2);
        value = arg.substr(pos + 1);
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.length() - 2);
        }
      } else {
        key = arg.substr(2);
        value = "";
      }
      args.emplace_back(key, value);
    }
  }

  if (args.size() == 0) {
    std::cout << "usage: h2_loco_client_example [--get_fsm_id | --set_fsm_id=N | --damp | --stand_up | --squat | --sit | --zero_torque | --move=\"vx vy omega\" | --set_arm_task=N | --stop_arm_task | --shake_hand[=N|JSON] | --wave_hand[=B|JSON] | --raw_api=ID --raw_param='{\"data\":...}' | ...]"
              << std::endl;
    return 0;
  }

  auto node = std::make_shared<TopstarH2ControlNode>(args);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
