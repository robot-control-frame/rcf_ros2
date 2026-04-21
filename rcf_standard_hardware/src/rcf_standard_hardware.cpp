#include "rcf_standard_hardware/rcf_standard_hardware.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace rcf_standard_hardware {
hardware_interface::CallbackReturn RcfStandardHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("RcfStandardHardware"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::on_init");
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  YAML::Node rcf_std_hardware_config, transmissions_config;
  try {
    transmissions_config_path_ = info_.hardware_parameters["transmissions_config_path"];
    RCLCPP_INFO(get_logger(), "transmissions_config_path: %s", transmissions_config_path_.c_str());
    rcf_std_hardware_config_path_ = info_.hardware_parameters["rcf_std_hardware_config_path"];
    RCLCPP_INFO(get_logger(), "rcf_std_hardware_config_path: %s", rcf_std_hardware_config_path_.c_str());

    // rcf_std_hardware_config = YAML::LoadFile(rcf_std_hardware_config_path_);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception while reading the YAML file: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  pluginlib::ClassLoader<rcf_standard_driver_interface::RcfStandardDriverBase> driver_loader(
      "rcf_standard_driver_interface", "rcf_standard_driver_interface::RcfStandardDriverBase");

  try {
    driver_ = driver_loader.createSharedInstance("rcf_fake_driver/RcfFakeDriver");
    std::vector<std::string> driver_config;
    driver_config.push_back(transmissions_config_path_);

    driver_->init(driver_config);
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(get_logger(), "Failed to create driver: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // if (!setup_imus(rcf_std_hardware_config)) {
  //   RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup IMU FAILURE");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RcfStandardHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::on_configure.");

  RCLCPP_INFO(get_logger(), "Configuration successful");
  driver_->starting();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RcfStandardHardware::export_state_interfaces() {
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RcfStandardHardware::export_command_interfaces() {
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  return command_interfaces;
}

hardware_interface::CallbackReturn RcfStandardHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::on_activate.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RcfStandardHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::on_deactivate.");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RcfStandardHardware::read(const rclcpp::Time& time,
                                                          const rclcpp::Duration& /*period*/) {
  RCLCPP_INFO(this->get_logger(), "RcfStandardHardware::read");
  driver_->get_servo_state();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RcfStandardHardware::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/) {
  RCLCPP_INFO(this->get_logger(), "RcfStandardHardware::write");
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  return hardware_interface::return_type::OK;
}

bool RcfStandardHardware::setup_imus(YAML::Node& hw_config) {
  try {
    auto imus_config = hw_config["standard_hardware"]["imus"];

    size_t imu_index = 0;
    for (auto imu : imus_config) {
      std::string name = imu.first.as<std::string>();
      YAML::Node imu_config = imu.second;
      uint16_t id = imu_config["id"].as<uint16_t>();
      auto data = std::make_unique<rcf_standard_hardware::ImuData>();
      data->name = name;
      RCLCPP_INFO(get_logger(), "RcfStandardHardware setup imu: %s, id: %d", name.c_str(), id);
      data->id = id;
      // hardware_interface::ImuSensorHandle imu_handle(
      //     data->name, data->name, data->orientation, data->orientation_covariance, data->angular_vel,
      //     data->angular_velocity_covariance, data->linear_accel, data->linear_acceleration_covariance, data->rpy);
      // imu_sensor_interface_.registerHandle(imu_handle);

      imus_data_vec_.push_back(std::move(data));
      id2imu_index_.insert(std::make_pair(id, imu_index));
      imu_index += 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  return true;
}

}  // namespace rcf_standard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rcf_standard_hardware::RcfStandardHardware, hardware_interface::SystemInterface)
