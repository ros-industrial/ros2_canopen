ament_add_gtest(test_node_interface
test_node_interface.cpp
)
target_include_directories(test_node_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_node_interface
  ${canopen_interfaces_TARGETS}
  canopen_base_driver::base_driver
  canopen_base_driver::lely_driver_bridge
  canopen_base_driver::lifecycle_base_driver
  canopen_base_driver::node_canopen_base_driver
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  node_canopen_proxy_driver
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)




ament_add_gtest(test_driver_component
test_driver_component.cpp
)
target_include_directories(test_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_driver_component
  ${canopen_interfaces_TARGETS}
  canopen_base_driver::base_driver
  canopen_base_driver::lely_driver_bridge
  canopen_base_driver::lifecycle_base_driver
  canopen_base_driver::node_canopen_base_driver
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/master.dcf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
