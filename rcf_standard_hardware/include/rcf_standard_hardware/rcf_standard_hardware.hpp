
#pragma once

#include <memory>
#include <string>
#include <vector>

// rcf
#include "rcf_standard_driver_interface/rcf_standard_driver_base.hpp"
// ros
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
// ros2_control
#include <pluginlib/class_loader.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace rcf_standard_hardware {
class RcfStandardHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RcfStandardHardware);

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

 private:
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::string transmissions_config_path_;
  std::string rcf_std_hardware_config_path_;
  bool hw_debug_{false};
  std::shared_ptr<rcf_standard_driver_interface::RcfStandardDriverBase> driver_;
};

}  // namespace rcf_standard_hardware
