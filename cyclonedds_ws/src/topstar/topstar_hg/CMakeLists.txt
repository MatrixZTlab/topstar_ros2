cmake_minimum_required(VERSION 3.5)
project(topstar_hg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)

# Require CycloneDDS 0.10.5
find_package(CycloneDDS 0.10.5 QUIET)
if(CycloneDDS_FOUND)
  message(STATUS "Found CycloneDDS ${CycloneDDS_VERSION}")
else()
  message(WARNING "CycloneDDS 0.10.5 not found - found version may differ")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/IMUState.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/PressSensorState.msg"
  "msg/HandMotorCmd.msg"
  "msg/HandMotorState.msg"
  "msg/HandCmd.msg"
  "msg/HandState.msg"
  "msg/GripperCmd.msg"
  "msg/GripperState.msg"
  "srv/GetArmFK.srv"
  "srv/GetArmIK.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_generate_dds_interfaces(
  ${rosidl_generate_interfaces_TARGET}__dds_connext_idl
  IDL_TUPLES ${rosidl_generate_interfaces_IDL_TUPLES}
  OUTPUT_SUBFOLDERS "dds_connext"
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}__dds_connext_idl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
