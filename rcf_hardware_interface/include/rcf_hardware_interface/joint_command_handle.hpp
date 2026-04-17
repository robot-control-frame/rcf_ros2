/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <limits>
#include <string>
#include <vector>

#include "control_modes.hpp"

namespace rcf_hardware_interface {

class JointCommandHandle {
 public:
  /**
   * \param joint_name Joint name
   */
  explicit JointCommandHandle(const std::string& joint_name) : joint_name_(joint_name) {}

  void state_interface_configuration(std::vector<std::string>& state_interfaces) const {
    state_interfaces.push_back(joint_name_ + "/position");
    state_interfaces.push_back(joint_name_ + "/velocity");
    state_interfaces.push_back(joint_name_ + "/effort");
  }
  void command_interface_configuration(std::vector<std::string>& command_interfaces) const {
    command_interfaces.push_back(joint_name_ + "/control_mode");
    command_interfaces.push_back(joint_name_ + "/cmd_position");
    command_interfaces.push_back(joint_name_ + "/cmd_velocity");
    command_interfaces.push_back(joint_name_ + "/cmd_effort");
    command_interfaces.push_back(joint_name_ + "/v1");
    command_interfaces.push_back(joint_name_ + "/v2");
    command_interfaces.push_back(joint_name_ + "/v3");
  }
  /// Mode value must use constants from \c ControlMode (e.g. \c ControlMode::MODE_EFFORT).
  void setCommand(double mode, double cmd_position, double cmd_velocity, double cmd_effort, double v1, double v2,
                  double v3) {
    control_mode_ = mode;
    cmd_position_ = cmd_position;
    cmd_velocity_ = cmd_velocity;
    cmd_effort_ = cmd_effort;
    v1_ = v1;
    v2_ = v2;
    v3_ = v3;
  }

  double getPosition() const { return position_; }
  double getVelocity() const { return velocity_; }
  double getEffort() const { return effort_; }
  double getControlMode() const { return control_mode_; }
  double getPosCommand() const { return cmd_position_; }
  double getVelCommand() const { return cmd_velocity_; }
  double getEffortCommand() const { return cmd_effort_; }
  double getV1() const { return v1_; }
  double getV2() const { return v2_; }
  double getV3() const { return v3_; }

 private:
  std::string joint_name_;
  double position_{std::numeric_limits<double>::quiet_NaN()};
  double velocity_{std::numeric_limits<double>::quiet_NaN()};
  double effort_{std::numeric_limits<double>::quiet_NaN()};
  double control_mode_{ControlModes::BEGIN};
  double cmd_position_{std::numeric_limits<double>::quiet_NaN()};
  double cmd_velocity_{std::numeric_limits<double>::quiet_NaN()};
  double cmd_effort_{std::numeric_limits<double>::quiet_NaN()};
  double v1_{std::numeric_limits<double>::quiet_NaN()};
  double v2_{std::numeric_limits<double>::quiet_NaN()};
  double v3_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace rcf_hardware_interface
