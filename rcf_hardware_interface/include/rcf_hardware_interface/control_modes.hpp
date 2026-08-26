/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <cstdint>

namespace rcf_hardware_interface {

enum class ControlMode : std::int8_t {
  BEGIN = -1,
  MODE_EFFORT = 0,
  MODE_VELOCITY = 1,
  MODE_POSITION = 2,
  MODE_PVT = 3,
  NOMODE = 4,
  EMERGENCY_STOP = 5,
  SWITCHING = 6,
  ERROR = 7,
  CUSTOM_MODE_1 = 8,
  CUSTOM_MODE_2 = 9,
  CUSTOM_MODE_3 = 10,
  UNKNOWN = 11,
};

constexpr double to_interface_value(ControlMode mode) { return static_cast<double>(static_cast<std::int8_t>(mode)); }

constexpr bool is_control_mode_value(double value, ControlMode mode) { return value == to_interface_value(mode); }

inline ControlMode to_control_mode(double value) {
  const auto int_value = static_cast<int>(value);
  switch (int_value) {
    case -1:
      return ControlMode::BEGIN;
    case 0:
      return ControlMode::MODE_EFFORT;
    case 1:
      return ControlMode::MODE_VELOCITY;
    case 2:
      return ControlMode::MODE_POSITION;
    case 3:
      return ControlMode::MODE_PVT;
    case 4:
      return ControlMode::NOMODE;
    case 5:
      return ControlMode::EMERGENCY_STOP;
    case 6:
      return ControlMode::SWITCHING;
    case 7:
      return ControlMode::ERROR;
    case 8:
      return ControlMode::CUSTOM_MODE_1;
    case 9:
      return ControlMode::CUSTOM_MODE_2;
    case 10:
      return ControlMode::CUSTOM_MODE_3;
    default:
      return ControlMode::UNKNOWN;
  }
}

}  // namespace rcf_hardware_interface
