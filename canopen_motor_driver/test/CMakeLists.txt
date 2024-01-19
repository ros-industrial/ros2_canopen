ament_add_gmock(test_canopen_402
test_canopen_402.cpp
)
ament_target_dependencies(test_canopen_402
  rclcpp
  lely_core_libraries
)
target_include_directories(test_canopen_402 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_canopen_402
    canopen_motor
)

ament_add_gmock(test_canopen_modes
test_canopen_modes.cpp
)
ament_target_dependencies(test_canopen_modes
  rclcpp
  lely_core_libraries
)
target_include_directories(test_canopen_modes PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)
target_link_libraries(test_canopen_modes
    canopen_motor
)
