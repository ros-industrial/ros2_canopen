ament_add_gtest(test_node_canopen_base_driver_ros
test_node_canopen_base_driver_ros.cpp
)
target_include_directories(test_node_canopen_base_driver_ros PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_node_canopen_base_driver_ros
  ${canopen_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  Boost::thread
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  diagnostic_updater::diagnostic_updater
  node_canopen_base_driver
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)




ament_add_gtest(test_base_driver_component
test_base_driver_component.cpp
)
target_include_directories(test_base_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_base_driver_component
  ${canopen_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  Boost::thread
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/master.dcf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
