/*******************************************************************************
 * Copyright (c) 2026/08/27, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#include "rcf_common/transmission_parse.hpp"

namespace rcf_common {

TransmissionParse::TransmissionParse() {}

TransmissionParse::~TransmissionParse() {}

bool TransmissionParse::parse(const YAML::Node& transmissions_config) {
  try {
    auto transmissions_data = transmissions_config["transmissions"];
    for (auto trans : transmissions_data) {
      YAML::Node trans_config = trans.second;
      YAML::Node joint_config = trans_config["joint"];
      if (!joint_config.IsMap()) {
        throw std::runtime_error("joint config must be a map");
      }
      for (auto joint : joint_config) {
        auto joint_name = joint.first.as<std::string>();
        joint_limits::JointLimits limit;
        YAML::Node joint_limit_config = joint.second["limit"];
        if (joint_limit_config.IsNull()) {
          RCLCPP_ERROR(logger_, "The joint %s does not contain limiting parameters.", joint_name.c_str());
        } else {
          auto lower = joint_limit_config["lower"];
          auto upper = joint_limit_config["upper"];
          auto velocity = joint_limit_config["velocity"];
          auto effort = joint_limit_config["effort"];
          if (lower.IsNull() or upper.IsNull()) {
            RCLCPP_ERROR(logger_, "The joint %s does not contain position limiting  parameters.", joint_name.c_str());
          } else {
            limit.has_position_limits = true;
            limit.min_position = lower.as<double>() + std::numeric_limits<double>::epsilon();
            limit.max_position = upper.as<double>() - std::numeric_limits<double>::epsilon();
          }

          if (velocity.IsNull()) {
            RCLCPP_ERROR(logger_, "The joint %s does not contain velocity limiting  parameters.", joint_name.c_str());
          } else {
            limit.has_velocity_limits = true;
            limit.max_velocity = velocity.as<double>();
          }

          if (effort.IsNull()) {
            RCLCPP_ERROR(logger_, "The joint %s does not contain effort limiting  parameters.", joint_name.c_str());
          } else {
            limit.has_effort_limits = true;
            limit.max_effort = effort.as<double>();
          }
        }
        joint_limits_map_.insert({joint_name, limit});
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  }

  return true;
}

joint_limits::JointLimits TransmissionParse::get_joint_limits(const std::string& joint_name) {
  if (joint_limits_map_.find(joint_name) == joint_limits_map_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("TransmissionParse"), "Joint %s not found in joint_limits_map_",
                 joint_name.c_str());
    throw std::runtime_error("Joint " + joint_name + " not found in joint_limits_map_");
  } else {
    return joint_limits_map_[joint_name];
  }
}

}  // namespace rcf_common
