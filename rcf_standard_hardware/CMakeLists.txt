cmake_minimum_required(VERSION 3.16)
project(rcf_standard_hardware LANGUAGES CXX)

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS hardware_interface pluginlib rclcpp
                                 rclcpp_lifecycle
                                 rcf_hardware_interface
                                 rcf_standard_driver_interface
                                 transmission_interface)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(rcf_standard_hardware SHARED src/rcf_standard_hardware.cpp)
target_compile_features(rcf_standard_hardware PUBLIC cxx_std_17)
target_include_directories(
  rcf_standard_hardware PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include/rcf_standard_hardware>)
ament_target_dependencies(rcf_standard_hardware PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface
                                         plugin.xml)

# INSTALL
install(DIRECTORY include/ DESTINATION include/rcf_standard_hardware)
install(
  TARGETS rcf_standard_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
