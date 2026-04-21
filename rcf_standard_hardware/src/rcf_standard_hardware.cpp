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

    rcf_std_hardware_config = YAML::LoadFile(rcf_std_hardware_config_path_);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception while reading the YAML file: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  pluginlib::ClassLoader<rcf_standard_driver_interface::RcfStandardDriverBase> driver_loader(
      "rcf_standard_driver_interface", "rcf_standard_driver_interface::RcfStandardDriverBase");

  try {
    driver_ = driver_loader.createSharedInstance("rcf_fake_driver/RcfFakeDriver");
    std::vector<std::string> driver_config;
    driver_config.push_back(rcf_std_hardware_config_path_);

    driver_->init(driver_config);
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(get_logger(), "Failed to create driver: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!setup_imus(rcf_std_hardware_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup IMU FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!setup_ft_sensors(rcf_std_hardware_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup FT SENSORS FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  for (const auto& imu : imus_data_vec_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.x", &imu->orientation[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.y", &imu->orientation[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.z", &imu->orientation[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.w", &imu->orientation[3]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "angular_velocity.x", &imu->angular_vel[0]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "angular_velocity.y", &imu->angular_vel[1]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "angular_velocity.z", &imu->angular_vel[2]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "linear_acceleration.x", &imu->linear_accel[0]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "linear_acceleration.y", &imu->linear_accel[1]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "linear_acceleration.z", &imu->linear_accel[2]));
  }

  for (const auto& ft : ft_sensor_data_vec_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "force.x", &ft->force[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "force.y", &ft->force[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "force.z", &ft->force[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "torque.x", &ft->torque[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "torque.y", &ft->torque[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "torque.z", &ft->torque[2]));
  }
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

  auto imu_states = driver_->get_imu_state();
  RCLCPP_INFO(get_logger(), "RcfStandardHardware read imu states size: %zu", imu_states.size());
  for (auto& item : imu_states) {
    auto iter = id2imu_index_.find(item.id);
    if (iter != id2imu_index_.end()) {
      auto& imu_data = imus_data_vec_[id2imu_index_[item.id]];
      imu_data->stamp = item.stamp;
      imu_data->angular_vel[0] = item.angular_velocity[0];
      imu_data->angular_vel[1] = item.angular_velocity[1];
      imu_data->angular_vel[2] = item.angular_velocity[2];
      imu_data->linear_accel[0] = item.linear_acceleration[0];
      imu_data->linear_accel[1] = item.linear_acceleration[1];
      imu_data->linear_accel[2] = item.linear_acceleration[2];
      imu_data->orientation[0] = item.orientation[0];
      imu_data->orientation[1] = item.orientation[1];
      imu_data->orientation[2] = item.orientation[2];
      imu_data->orientation[3] = item.orientation[3];
      RCLCPP_INFO(get_logger(), "RcfStandardHardware read imu:********** %s, id: %d", imu_data->name.c_str(),
                  imu_data->id);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown IMU device ID: %d", item.id);
    }
  }

  // read force torque sensors
  auto ft_status = driver_->get_ft_state();
  for (auto& item : ft_status) {
    auto iter = id2ft_index_.find(item.id);
    if (iter != id2ft_index_.end()) {
      auto& ft_data = ft_sensor_data_vec_[id2ft_index_[item.id]];
      ft_data->stamp = item.stamp;
      ft_data->force[0] = item.force[0];
      ft_data->force[1] = item.force[1];
      ft_data->force[2] = item.force[2];
      ft_data->torque[0] = item.torque[0];
      ft_data->torque[1] = item.torque[1];
      ft_data->torque[2] = item.torque[2];

    } else {
      RCLCPP_ERROR(get_logger(), "Unknown FT device ID: %d", item.id);
    }
  }

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
    auto imus_config = hw_config["rcf_std_hardware"]["imus"];

    size_t imu_index = 0;
    for (auto imu : imus_config) {
      std::string name = imu.first.as<std::string>();
      YAML::Node imu_config = imu.second;
      uint16_t id = imu_config["id"].as<uint16_t>();
      auto data = std::make_unique<rcf_standard_hardware::ImuData>();
      data->name = name;
      RCLCPP_INFO(get_logger(), "RcfStandardHardware setup imu: %s, id: %d", name.c_str(), id);
      data->id = id;
      imus_data_vec_.push_back(std::move(data));
      id2imu_index_.insert(std::make_pair(id, imu_index));
      imu_index += 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  RCLCPP_INFO(get_logger(), "Setup imus sensors success");
  return true;
}

bool RcfStandardHardware::setup_ft_sensors(YAML::Node& hw_config) {
  try {
    auto fts_config = hw_config["rcf_std_hardware"]["force_torque_sensors"];
    size_t ft_index = 0;
    for (auto ft : fts_config) {
      std::string name = ft.first.as<std::string>();
      YAML::Node ft_config = ft.second;
      uint16_t id = ft_config["id"].as<uint16_t>();
      auto data = std::make_unique<ForceTorqueData>();
      data->name = name;
      data->id = id;
      ft_sensor_data_vec_.push_back(std::move(data));
      id2ft_index_.insert(std::make_pair(id, ft_index));
      ft_index += 1;
    }
    RCLCPP_INFO(get_logger(), "Setup ft sensors success");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  return true;
}

}  // namespace rcf_standard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rcf_standard_hardware::RcfStandardHardware, hardware_interface::SystemInterface)
