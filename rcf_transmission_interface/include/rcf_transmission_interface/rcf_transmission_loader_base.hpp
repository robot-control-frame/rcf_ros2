/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <yaml-cpp/yaml.h>

#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>

namespace rcf_transmission_interface {

class RcfTransmissionLoaderBase : public transmission_interface::TransmissionLoader {
 public:
  virtual hardware_interface::TransmissionInfo create_transmission_info(const std::string name, YAML::Node& config) = 0;
};

}  // namespace rcf_transmission_interface
