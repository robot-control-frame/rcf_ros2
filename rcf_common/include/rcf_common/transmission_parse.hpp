#pragma once
#include <yaml-cpp/yaml.h>

#include <joint_limits/joint_limits.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
namespace rcf_common {

class TransmissionParse {
 public:
  TransmissionParse();
  ~TransmissionParse();

  bool parse(const YAML::Node& transmissions_config);

  joint_limits::JointLimits get_joint_limits(const std::string& joint_name);

 private:
  std::unordered_map<std::string, joint_limits::JointLimits> joint_limits_map_;

  rclcpp::Logger logger_{rclcpp::get_logger("TransmissionParse")};
};
}  // namespace rcf_common
