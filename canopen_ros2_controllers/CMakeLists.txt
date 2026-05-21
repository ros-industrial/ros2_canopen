cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_controllers)

find_package(ament_cmake REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/canopen_proxy_controller.cpp
  src/cia402_device_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${canopen_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  canopen_402_driver::cia402_driver
  canopen_402_driver::lely_motion_controller_bridge
  canopen_402_driver::lifecycle_cia402_driver
  canopen_402_driver::node_canopen_cia402_driver
  canopen_proxy_driver::lifecycle_proxy_driver
  canopen_proxy_driver::node_canopen_proxy_driver
  canopen_proxy_driver::proxy_driver
  controller_interface::controller_interface
  controller_manager::controller_manager
  controller_manager::controller_manager_parameters
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  realtime_tools::thread_priority
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "CANOPEN_PROXY_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface canopen_ros2_controllers.xml)

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(FALSE)
  # find_package(ament_cmake_gmock REQUIRED)

#   ament_add_gmock(test_load_canopen_proxy_controller test/test_load_canopen_proxy_controller.cpp)
#   target_include_directories(test_load_canopen_proxy_controller PRIVATE include)
#   target_link_libraries(test_load_canopen_proxy_controller ...

#   ament_add_gmock(test_load_cia402_device_controller test/test_load_cia402_device_controller.cpp)
#   target_include_directories(test_load_cia402_device_controller PRIVATE include)
#   target_link_libraries(test_load_cia402_device_controller ...

#   ament_add_gmock(test_load_cia402_robot_controller test/test_load_cia402_robot_controller.cpp)
#   target_include_directories(test_load_cia402_robot_controller PRIVATE include)
#   target_link_libraries(test_load_cia402_robot_controller ...

#   ament_add_gmock(test_canopen_proxy_controller test/test_canopen_proxy_controller.cpp)
#   target_include_directories(test_canopen_proxy_controller PRIVATE include)
#   target_link_libraries(test_canopen_proxy_controller ...
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  canopen_402_driver
  canopen_interfaces
  canopen_proxy_driver
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
)

ament_package()
