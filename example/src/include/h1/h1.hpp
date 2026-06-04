// H1 joint indices and default PD gains for the wheeled humanoid.
// H1 has 18 upper-body DOF; the wheel base is driven separately via Twist.
// Mirrors simulate_python/h1_joint_defs.py — keep both in sync.

#pragma once

#include <array>

constexpr int H1_NUM_JOINTS  = 18;
constexpr int H1_MOTOR_SLOTS = 18;  // motor_cmd[0:18] in LowCmd/LowState

enum class H1JointIndex : int {
    // Core
    TORSO_LIFT          = 0,
    TORSO_PITCH         = 1,
    HEAD_YAW            = 2,
    HEAD_PITCH          = 3,

    // Right arm (7 DOF)
    RIGHT_SHOULDER_BASE = 4,
    RIGHT_SHOULDER      = 5,
    RIGHT_ELBOW_YAW     = 6,
    RIGHT_ELBOW         = 7,
    RIGHT_WRIST_YAW     = 8,
    RIGHT_WRIST_PITCH   = 9,
    RIGHT_WRIST_ROLL    = 10,

    // Left arm (7 DOF)
    LEFT_SHOULDER_BASE  = 11,
    LEFT_SHOULDER       = 12,
    LEFT_ELBOW_YAW      = 13,
    LEFT_ELBOW          = 14,
    LEFT_WRIST_YAW      = 15,
    LEFT_WRIST_PITCH    = 16,
    LEFT_WRIST_ROLL     = 17,
};

// Default position-control gains for each motor slot.
// Order: torso_lift, torso_pitch, head_yaw, head_pitch, right arm ×7, left arm ×7.
constexpr std::array<float, H1_NUM_JOINTS> H1_DEFAULT_KP = {
    5000.0f, 3000.0f,  500.0f,  500.0f,   // core
     800.0f,  800.0f,  800.0f,  800.0f,  600.0f, 600.0f, 600.0f,  // right arm
     800.0f,  800.0f,  800.0f,  800.0f,  600.0f, 600.0f, 600.0f,  // left arm
};

constexpr std::array<float, H1_NUM_JOINTS> H1_DEFAULT_KD = {
    100.0f,  80.0f,  20.0f,  20.0f,
     30.0f,  30.0f,  30.0f,  30.0f,  20.0f, 20.0f, 20.0f,
     30.0f,  30.0f,  30.0f,  30.0f,  20.0f, 20.0f, 20.0f,
};
