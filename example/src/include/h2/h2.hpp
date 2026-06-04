// H2 joint indices for arm SDK example (arm joints only)
// H2 has 29 DOF total; arms are joints 15-28

constexpr float PI_2 = 1.57079632F;

enum class H2ArmJointIndex : int {
  // Left leg
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_ROLL = 5,

  // Right leg
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_ROLL = 11,

  // Waist
  WAIST_YAW = 12,

  // Head
  HEAD_YAW = 13,
  HEAD_PITCH = 14,

  // Left arm
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_YAW = 19,
  LEFT_WRIST_PITCH = 20,
  LEFT_WRIST_ROLL = 21,

  // Right arm
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_YAW = 26,
  RIGHT_WRIST_PITCH = 27,
  RIGHT_WRIST_ROLL = 28,
};
