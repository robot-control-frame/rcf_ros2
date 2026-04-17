/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * All rights reserved.
 *******************************************************************************/

#pragma once

namespace rcf_hardware_interface {
struct ControlModes {
  static constexpr double BEGIN = -1.0;
  static constexpr double MODE_EFFORT = 0.0;
  static constexpr double MODE_VELOCITY = 1.0;
  static constexpr double MODE_POSITION = 2.0;
  static constexpr double MODE_PVT = 3.0;
  static constexpr double NOMODE = 4.0;
  static constexpr double EMERGENCY_STOP = 5.0;
  static constexpr double SWITCHING = 6.0;
  static constexpr double ERROR = 7.0;
  static constexpr double CUSTOM_MODE_1 = 8.0;
  static constexpr double CUSTOM_MODE_2 = 9.0;
  static constexpr double CUSTOM_MODE_3 = 10.0;
};
}  // namespace rcf_hardware_interface
