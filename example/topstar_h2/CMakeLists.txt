cmake_minimum_required(VERSION 3.5)
project(topstar_ros2_h2_example)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Reuse existing H2 sources/assets from the original package tree.
set(TOPSTAR_LEGACY_EXAMPLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)

include_directories(
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/include
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/include/common
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/include/nlohmann
)
link_directories(${TOPSTAR_LEGACY_EXAMPLE_DIR}/src)

set(
  DEPENDENCY_LIST
  topstar_hg
  topstar_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

find_package(ament_cmake REQUIRED)
find_package(topstar_hg REQUIRED)
find_package(topstar_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

find_package(CycloneDDS 0.10.5 QUIET)
if(CycloneDDS_FOUND)
  message(STATUS "Found CycloneDDS ${CycloneDDS_VERSION}")
else()
  message(WARNING "CycloneDDS 0.10.5 not found - found version may differ")
endif()

add_executable(h2_low_level_example
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/lowlevel/h2_low_level_example.cpp
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/common/motor_crc_hg.cpp
)
add_executable(h2_ankle_swing_example
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/lowlevel/h2_ankle_swing_example.cpp
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/common/motor_crc_hg.cpp
)
add_executable(h2_joint_oscillation_example
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/lowlevel/h2_joint_oscillation_example.cpp
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/common/motor_crc_hg.cpp
)
add_executable(read_low_state_hg ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/read_low_state_hg.cpp)

add_executable(h2_arm_sdk_dds_example
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/high_level/h2_arm_sdk_dds_example.cpp
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/common/motor_crc_hg.cpp
)
add_executable(h2_arm_action_example
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/high_level/h2_arm_action_example.cpp
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/common/motor_crc_hg.cpp
)
add_executable(h2_loco_client_example
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/high_level/h2_loco_client_example.cpp
  ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/common/motor_crc_hg.cpp
)
add_executable(h2_ls_hand_example ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/h2/ls_hand/h2_ls_hand_example.cpp)

target_compile_features(h2_low_level_example PRIVATE cxx_std_20)
target_compile_features(h2_ankle_swing_example PRIVATE cxx_std_20)
target_compile_features(h2_joint_oscillation_example PRIVATE cxx_std_20)
target_compile_features(h2_arm_sdk_dds_example PRIVATE cxx_std_20)
target_compile_features(h2_arm_action_example PRIVATE cxx_std_20)
target_compile_features(h2_loco_client_example PRIVATE cxx_std_20)
target_compile_features(h2_ls_hand_example PRIVATE cxx_std_20)

ament_target_dependencies(h2_low_level_example ${DEPENDENCY_LIST})
ament_target_dependencies(h2_ankle_swing_example ${DEPENDENCY_LIST})
ament_target_dependencies(h2_joint_oscillation_example ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state_hg ${DEPENDENCY_LIST})
ament_target_dependencies(h2_arm_sdk_dds_example ${DEPENDENCY_LIST})
ament_target_dependencies(h2_arm_action_example ${DEPENDENCY_LIST})
ament_target_dependencies(h2_loco_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(h2_ls_hand_example ${DEPENDENCY_LIST})

install(TARGETS
  h2_low_level_example
  h2_ankle_swing_example
  h2_joint_oscillation_example
  read_low_state_hg
  h2_arm_sdk_dds_example
  h2_arm_action_example
  h2_loco_client_example
  h2_ls_hand_example
  DESTINATION lib/${PROJECT_NAME}
)

set(TOPSTAR_H2_SRC_DIR $ENV{HOME}/topstar_h2/src)
if(CycloneDDS_FOUND AND EXISTS "${TOPSTAR_H2_SRC_DIR}/topstar_hg.c")
  add_executable(mujoco_ros2_bridge
    ${TOPSTAR_LEGACY_EXAMPLE_DIR}/src/mujoco_ros2_bridge.cpp
    ${TOPSTAR_H2_SRC_DIR}/topstar_hg.c
  )
  target_compile_features(mujoco_ros2_bridge PRIVATE cxx_std_17)
  target_include_directories(mujoco_ros2_bridge PRIVATE ${TOPSTAR_H2_SRC_DIR})
  ament_target_dependencies(mujoco_ros2_bridge ${DEPENDENCY_LIST})
  target_link_libraries(mujoco_ros2_bridge CycloneDDS::ddsc Threads::Threads)
  install(TARGETS mujoco_ros2_bridge DESTINATION lib/${PROJECT_NAME})
  message(STATUS "mujoco_ros2_bridge: enabled (CycloneDDS ${CycloneDDS_VERSION})")

else()
  if(NOT CycloneDDS_FOUND)
    message(WARNING "mujoco_ros2_bridge: disabled (CycloneDDS not found)")
  else()
    message(WARNING "mujoco_ros2_bridge: disabled (topstar_h2 src not found at ${TOPSTAR_H2_SRC_DIR})")
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()