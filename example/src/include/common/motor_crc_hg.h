/*****************************************************************
 Copyright (c) 2020, Topstar Robotics.Co.Ltd. All rights reserved.
 Modified for Topstar H2 robot
******************************************************************/

#ifndef _MOTOR_CRC_H_
#define _MOTOR_CRC_H_

#include <stdint.h>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "topstar_hg/msg/low_cmd.hpp"
#include "topstar_hg/msg/motor_cmd.hpp"

typedef struct {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float Kp;
  float Kd;
  uint32_t reserve = 0;
} MotorCmd;

typedef struct {
  uint8_t modePr;
  uint8_t modeMachine;
  std::array<MotorCmd, 35> motorCmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
} LowCmd;

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
void get_crc(topstar_hg::msg::LowCmd &msg);

#endif
