/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * All rights reserved.
 *******************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
namespace rcf_hardware_interface {

class JointStateHandle : public semantic_components::SemanticComponentInterface<sensor_msgs::msg::JointState> {
 public:
  explicit JointStateHandle(const std::string& name)
      : SemanticComponentInterface(name,
                                   {{name + "/" + "position"}, {name + "/" + "velocity"}, {name + "/" + "effort"}}) {}

  void state_interface_configuration(std::vector<std::string>& state_interfaces) const {
    state_interfaces.push_back(name + "/position");
    state_interfaces.push_back(name + "/velocity");
    state_interfaces.push_back(name + "/effort");
  }

  std::array<double, 3> get_joint_state() const {
    update_data_from_interfaces();
    std::array<double, 3> joint_state;
    std::copy(data_.begin(), data_.begin() + 3, joint_state.begin());
    return joint_state;
  }

 protected:
  void update_data_from_interfaces() const {
    for (auto i = 0u; i < data_.size(); ++i) {
      const auto data = state_interfaces_[i].get().get_optional();
      if (data.has_value()) {
        data_[i] = data.value();
      }
    }
  }
  std::string name;
  mutable std::array<double, 3> data_{{0.0, 0.0, 0.0}};
};

}  // namespace rcf_hardware_interface
