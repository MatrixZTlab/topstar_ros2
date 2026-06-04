cmake_minimum_required(VERSION 3.5)
project(topstar_ros2_example)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── mujoco_ros2_bridge (C++ DDS relay) ───────────────────────────────────────
# Optional: only built when ~/topstar_mujoco/simulate/src/topstar_hg.c exists
# and CycloneDDS is available. Depends on topstar_mujoco, NOT topstar_h2.
set(_TOPSTAR_MUJOCO_SRC "$ENV{HOME}/topstar_mujoco/simulate/src")
if(EXISTS "${_TOPSTAR_MUJOCO_SRC}/topstar_hg.c")
  list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu")
  find_package(CycloneDDS QUIET)
  if(CycloneDDS_FOUND)
    add_executable(mujoco_ros2_bridge
      src/mujoco_ros2_bridge.cpp
      ${_TOPSTAR_MUJOCO_SRC}/topstar_hg.c
    )
    target_include_directories(mujoco_ros2_bridge PRIVATE
      ${_TOPSTAR_MUJOCO_SRC}
    )
    target_link_libraries(mujoco_ros2_bridge CycloneDDS::ddsc)
    install(TARGETS mujoco_ros2_bridge DESTINATION lib/${PROJECT_NAME})
    message(STATUS "mujoco_ros2_bridge: enabled (topstar_mujoco + CycloneDDS found)")
  else()
    message(STATUS "mujoco_ros2_bridge: skipped (CycloneDDS not found)")
  endif()
else()
  message(STATUS "mujoco_ros2_bridge: skipped (topstar_mujoco not found at ${_TOPSTAR_MUJOCO_SRC})")
endif()
# ── H1 Python nodes ───────────────────────────────────────────────────────────
# Install the topstar_h1 Python package (joint_defs, ros2 node, drive example)
ament_python_install_package(topstar_h1 PACKAGE_DIR src/h1/topstar_h1)

# Install H1 executable entry points
install(PROGRAMS
  src/h1/topstar_h1/h1_ros2_node.py
  RENAME h1_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/h1/topstar_h1/h1_drive_example.py
  RENAME h1_drive_example
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/h1/topstar_h1/h1_send_velocity.py
  RENAME h1_send_velocity
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/h1/topstar_h1/h1_upper_body_jog.py
  RENAME h1_upper_body_jog
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/h1/topstar_h1/x5_monitor.py
  RENAME x5_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Remove legacy H2 binaries that used to be installed by this package.
# They can remain in install/ after the package split and cause ros2 run to
# resolve stale executables from topstar_ros2_example.
install(CODE [[
  set(_topstar_pkg_name "topstar_ros2_example")
  set(_topstar_legacy_h2_bins
    "h2_low_level_example"
    "h2_ankle_swing_example"
    "h2_joint_oscillation_example"
    "read_low_state_hg"
    "h2_arm_sdk_dds_example"
    "h2_arm_action_example"
    "h2_loco_client_example"
    "h2_ls_hand_example"
  )

  foreach(_bin IN LISTS _topstar_legacy_h2_bins)
    set(_legacy_path "${CMAKE_INSTALL_PREFIX}/lib/${_topstar_pkg_name}/${_bin}")
    if(EXISTS "${_legacy_path}" OR IS_SYMLINK "${_legacy_path}")
      file(REMOVE "${_legacy_path}")
      message(STATUS "Removed legacy executable from ${_topstar_pkg_name}: ${_legacy_path}")
    endif()
  endforeach()
]])

# ── Launch files ──────────────────────────────────────────────────────────────
install(DIRECTORY
  src/h1/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Config files ─────────────────────────────────────────────────────────────
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── URDF / mesh assets ────────────────────────────────────────────────────────
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
