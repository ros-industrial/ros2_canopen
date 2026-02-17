ament_add_gtest(test_node_canopen_basic_driver
  test_node_canopen_basic_master.cpp
)
target_include_directories(test_node_canopen_basic_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_node_canopen_basic_driver
  ${canopen_interfaces_TARGETS}
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  lely_master_bridge
  master_driver
  node_canopen_basic_master
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(test_node_canopen_basic_driver_ros
test_node_canopen_basic_master_ros.cpp
)
target_include_directories(test_node_canopen_basic_driver_ros PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_node_canopen_basic_driver_ros
  ${canopen_interfaces_TARGETS}
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  lely_master_bridge
  master_driver
  node_canopen_basic_master
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gtest(test_master_driver_component
test_master_driver_component.cpp
)
target_include_directories(test_master_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_master_driver_component
  ${canopen_interfaces_TARGETS}
  canopen_core::device_container
  canopen_core::node_canopen_driver
  canopen_core::node_canopen_master
  lely_master_bridge
  lifecycle_master_driver
  master_driver
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  rclcpp_lifecycle::rclcpp_lifecycle
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/master.dcf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
