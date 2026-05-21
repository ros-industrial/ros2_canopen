cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control_test_assets)

add_library(
  canopen_ros2_control
  SHARED
  src/canopen_system.cpp
  src/cia402_system.cpp
  src/robot_system.cpp
)



target_compile_features(canopen_ros2_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(canopen_ros2_control PUBLIC -fPIC -Wl,--no-undefined)

target_include_directories(canopen_ros2_control PUBLIC include)
target_link_libraries(canopen_ros2_control PUBLIC
  canopen_402_driver::cia402_driver
  canopen_402_driver::lely_motion_controller_bridge
  canopen_402_driver::lifecycle_cia402_driver
  canopen_402_driver::node_canopen_cia402_driver
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  canopen_proxy_driver::lifecycle_proxy_driver
  canopen_proxy_driver::node_canopen_proxy_driver
  canopen_proxy_driver::proxy_driver
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)

# prevent pluginlib from using boost
target_compile_definitions(canopen_ros2_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface canopen_ros2_control.xml)

install(
  TARGETS
  canopen_ros2_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  #find_package(ament_cmake_gmock REQUIRED)
  #find_package(ros2_control_test_assets REQUIRED)

  #ament_add_gmock(test_canopen_system test/test_canopen_system.cpp)
  #target_include_directories(test_canopen_system PRIVATE include)
  #target_link_libraries(test_canopen_system ...
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_ros2_control
)
ament_export_dependencies(
  canopen_402_driver
  canopen_core
  canopen_proxy_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

ament_package()
