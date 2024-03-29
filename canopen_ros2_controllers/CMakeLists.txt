cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_controllers)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
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

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

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
  find_package(ament_cmake_gmock REQUIRED)

#   ament_add_gmock(test_load_canopen_proxy_controller test/test_load_canopen_proxy_controller.cpp)
#   target_include_directories(test_load_canopen_proxy_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_canopen_proxy_controller
#     controller_manager
#     hardware_interface
#     canopen_interfaces
#   )

#   ament_add_gmock(test_load_cia402_device_controller test/test_load_cia402_device_controller.cpp)
#   target_include_directories(test_load_cia402_device_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_cia402_device_controller
#     controller_manager
#     hardware_interface
#     canopen_interfaces
#   )

#   ament_add_gmock(test_load_cia402_robot_controller test/test_load_cia402_robot_controller.cpp)
#   target_include_directories(test_load_cia402_robot_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_cia402_robot_controller
#     controller_manager
#     hardware_interface
#     canopen_interfaces
#   )

#  ament_add_gmock(test_canopen_proxy_controller test/test_canopen_proxy_controller.cpp)
#   target_include_directories(test_canopen_proxy_controller PRIVATE include)
#   target_link_libraries(test_canopen_proxy_controller ${PROJECT_NAME})
#   ament_target_dependencies(
#     test_canopen_proxy_controller
#     controller_interface
#     hardware_interface
#     canopen_interfaces
#  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
