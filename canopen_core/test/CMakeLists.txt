ament_add_gmock(test_node_canopen_driver
    test_node_canopen_driver.cpp
)
target_include_directories(test_node_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_node_canopen_driver
    ${canopen_interfaces_TARGETS}
    node_canopen_driver
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
)

ament_add_gmock(test_node_canopen_master
  test_node_canopen_master.cpp
)
target_include_directories(test_node_canopen_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_node_canopen_master
    ${canopen_interfaces_TARGETS}
    node_canopen_master
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
)

# Temporary fixing ci build stability
if(DEVICE_CONTAINER_TESTING)
ament_add_gmock(test_device_container
  test_device_container.cpp
)
target_include_directories(test_device_container PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_device_container
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    device_container
    node_canopen_driver
    node_canopen_master
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
)
endif()

file(COPY bus_configs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

ament_add_gmock(test_canopen_driver
test_canopen_driver.cpp
)
target_include_directories(test_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_canopen_driver
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    node_canopen_driver
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
)

ament_add_gmock(test_lifecycle_canopen_driver
test_lifecycle_canopen_driver.cpp
)
target_include_directories(test_lifecycle_canopen_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_lifecycle_canopen_driver
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    node_canopen_driver
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
)

ament_add_gmock(test_canopen_master
test_canopen_master.cpp
)
target_include_directories(test_canopen_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_canopen_master
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    node_canopen_master
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
)

ament_add_gmock(test_lifecycle_canopen_master
test_lifecycle_canopen_master.cpp
)
target_include_directories(test_lifecycle_canopen_master PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_lifecycle_canopen_master
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    node_canopen_master
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
    yaml-cpp
)

ament_add_gmock(test_lifecycle_manager
test_lifecycle_manager.cpp
)
target_include_directories(test_lifecycle_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_lifecycle_manager
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    device_container
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
)

ament_add_gmock(test_errors
test_errors.cpp
)
target_include_directories(test_errors PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_errors
    ${canopen_interfaces_TARGETS}
    ${lifecycle_msgs_TARGETS}
    device_container
    node_canopen_driver
    node_canopen_master
    rclcpp::rclcpp
    rclcpp_components::component
    rclcpp_components::component_manager
    rclcpp_lifecycle::rclcpp_lifecycle
)
