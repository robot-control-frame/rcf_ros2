/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "rcf_transmission_interface/rcf_simple_transmission_loader.hpp"

#include <fmt/format.h>

#include <stdexcept>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcf_transmission_interface/rcf_simple_transmission.hpp"
#include "rclcpp/logging.hpp"

namespace rcf_transmission_interface {
hardware_interface::TransmissionInfo RcfSimpleTransmissionLoader::create_transmission_info(const std::string name,
                                                                                           YAML::Node& config) {
  try {
    YAML::Node trans_joint_config = config["joint"];
    if (!trans_joint_config.IsMap() || trans_joint_config.size() != 1) {
      throw std::runtime_error("joint config must contain exactly one joint entry");
    }
    const auto joint_entry = trans_joint_config.begin();
    const auto joint_name = joint_entry->first.as<std::string>();
    YAML::Node joint_config = joint_entry->second;
    const auto joint_offset = joint_config["offset"].as<double>(0.0);

    YAML::Node trans_act_config = config["actuator"];
    if (!trans_act_config.IsMap() || trans_act_config.size() != 1) {
      throw std::runtime_error("actuator config must contain exactly one actuator entry");
    }
    const auto actuator_entry = trans_act_config.begin();
    const auto actuator_name = actuator_entry->first.as<std::string>();
    YAML::Node actuator_config = actuator_entry->second;
    const auto mechanical_reduction = actuator_config["mechanical_reduction"].as<double>();
    hardware_interface::TransmissionInfo transmission_info;
    transmission_info.name = name;
    transmission_info.type = "rcf_transmission_interface/RcfSimpleTransmission";

    hardware_interface::JointInfo joint_info;
    joint_info.name = joint_name;
    joint_info.offset = joint_offset;
    transmission_info.joints.push_back(joint_info);

    hardware_interface::ActuatorInfo act_info;
    act_info.name = actuator_name;
    act_info.mechanical_reduction = mechanical_reduction;
    transmission_info.actuators.push_back(act_info);
    return transmission_info;
  } catch (const std::exception& ex) {
    throw std::runtime_error(fmt::format("Failed to create transmission info '{}': {}", name, ex.what()));
  }
}

std::shared_ptr<transmission_interface::Transmission> RcfSimpleTransmissionLoader::load(
    const hardware_interface::TransmissionInfo& transmission_info) {
  try {
    const auto mechanical_reduction = transmission_info.actuators.at(0).mechanical_reduction;
    const auto offset = transmission_info.joints.at(0).offset;
    std::shared_ptr<transmission_interface::Transmission> transmission(
        new RcfSimpleTransmission(mechanical_reduction, offset));
    return transmission;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rcf_simple_transmission_loader"), "Failed to construct transmission '%s' because",
                 ex.what());
    return std::shared_ptr<transmission_interface::Transmission>();
  }
}

}  // namespace rcf_transmission_interface

PLUGINLIB_EXPORT_CLASS(rcf_transmission_interface::RcfSimpleTransmissionLoader,
                       rcf_transmission_interface::RcfTransmissionLoaderBase)
