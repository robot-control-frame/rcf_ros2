
#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
// rcf
#include "rcf_hardware_interface/control_modes.hpp"
#include "rcf_standard_driver_interface/rcf_standard_driver_base.hpp"
#include "rcf_standard_hardware/type.hpp"

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
#include "transmission_interface/transmission.hpp"

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
  rclcpp::Logger get_logger() const override { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const override { return clock_; }

 private:
  // set hardware interface
  bool setup_motors(YAML::Node& hw_config);
  bool setup_imus(YAML::Node& hw_config);
  bool setup_transmission(YAML::Node& transmissions_config);
  bool setup_ft_sensors(YAML::Node& hw_config);
  bool setup_check_timeout(YAML::Node& hw_config);

  void init_dynamic_actuator_state_msg();
  // motor
  std::vector<std::shared_ptr<ActData>> act_data_vector;
  std::unordered_map<uint16_t, size_t> id2act_index_;
  std::vector<std::shared_ptr<JointData>> joint_data_vector;
  // imu
  std::vector<std::shared_ptr<ImuData>> imus_data_vec_;
  std::unordered_map<uint16_t, size_t> id2imu_index_;
  // ft
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensor_data_vec_;
  std::unordered_map<uint16_t, size_t> id2ft_index_;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::string transmissions_config_path_;
  std::string rcf_std_hardware_config_path_;
  bool hw_debug_{false};
  std::shared_ptr<rcf_standard_driver_interface::RcfStandardDriverBase> driver_;

  // ros2_control
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;

  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::DynamicJointState>> dynamic_actuator_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>
      realtime_dynamic_actuator_state_publisher_;
  control_msgs::msg::DynamicJointState dynamic_actuator_state_msg_;

  bool servo_init_{true};
  bool use_jump_command_protection_{true};
};

}  // namespace rcf_standard_hardware
