/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <memory>

#include "rcf_transmission_interface/rcf_transmission_loader_base.hpp"
namespace rcf_transmission_interface {

class RcfSimpleTransmissionLoader : public RcfTransmissionLoaderBase {
 public:
  hardware_interface::TransmissionInfo create_transmission_info(const std::string name, YAML::Node& config) override;
  std::shared_ptr<transmission_interface::Transmission> load(
      const hardware_interface::TransmissionInfo& transmission_info) override;
};

}  // namespace rcf_transmission_interface
