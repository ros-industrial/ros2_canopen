#include "canopen_ros2_control/robot_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using namespace canopen_ros2_control;

auto robot_system_logger = rclcpp::get_logger("robot_system_interface");

hardware_interface::CallbackReturn Cia402RobotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    robot_system_logger, "bus_config: '%s'", info_.hardware_parameters["bus_config"].c_str());
  RCLCPP_INFO(
    robot_system_logger, "master_config: '%s'", info_.hardware_parameters["master_config"].c_str());
  RCLCPP_INFO(
    robot_system_logger, "can_interface_name: '%s'",
    info_.hardware_parameters["can_interface_name"].c_str());
  RCLCPP_INFO(
    robot_system_logger, "master_bin: '%s'", info_.hardware_parameters["master_bin"].c_str());

  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Cia402RobotSystem::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  spin_thread_ = std::make_unique<std::thread>(&Cia402RobotSystem::spin, this);
  init_thread_ = std::make_unique<std::thread>(&Cia402RobotSystem::initDeviceContainer, this);

  if (init_thread_->joinable())
  {
    init_thread_->join();
  }
  else
  {
    RCLCPP_ERROR(robot_system_logger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Cia402RobotSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Cia402RobotSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Cia402RobotSystem::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Cia402RobotSystem::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Cia402RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Iterate over joints in xacro
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // If no node_id in xacro, skip.
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding motor canopen interfaces
      continue;
    }

    // Read node_id
    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // actual position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &motor_data_[node_id].actual_position));

    // actual speed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[node_id].actual_velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Cia402RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Iterate over joints in xacro
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // Skip joing if node_id missing
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        robot_system_logger, "Joint %s does not have node_id specified.",
        info_.joints[i].name.c_str());
      // skip adding canopen interfaces
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // DATA INTERFACES
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &motor_data_[node_id].target_position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &motor_data_[node_id].target_velocity));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motor_data_[node_id].target_torque));

    // COMMAND INTERFACES

    // Switch command
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "operation_mode", &motor_data_[node_id].operation_mode.mode));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "operation_mode_feedback", &motor_data_[node_id].operation_mode.resp));

    // Init command
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "init", &motor_data_[node_id].init.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "init_feedback", &motor_data_[node_id].init.response));

    // Recover command
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "recover", &motor_data_[node_id].recover.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "recover_feedback", &motor_data_[node_id].recover.response));

    // Halt command
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "halt", &motor_data_[node_id].halt.command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "halt_feedback", &motor_data_[node_id].halt.response));
  }
  return command_interfaces;
}

hardware_interface::return_type Cia402RobotSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto drivers = device_container_->get_registered_drivers();

  // Iterate over joints
  for (auto it = motor_data_.begin(); it != motor_data_.end(); ++it)
  {
    auto motion_controller_driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers[it->first]);
    // get position
    motor_data_[it->first].actual_position = motion_controller_driver->get_position();
    // get speed
    motor_data_[it->first].actual_velocity = motion_controller_driver->get_speed();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Cia402RobotSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto drivers = device_container_->get_registered_drivers();

  for (auto it = motor_data_.begin(); it != motor_data_.end(); ++it)
  {
    auto motion_controller_driver =
      std::static_pointer_cast<ros2_canopen::Cia402Driver>(drivers[it->first]);

    // init
    if (motor_data_[it->first].init.command_available())
    {
      motor_data_[it->first].init.set_response(motion_controller_driver->init_motor());
      RCLCPP_INFO(robot_system_logger, "Initialised node %d", it->first);
    }

    // recover
    if (motor_data_[it->first].recover.command_available())
    {
      motor_data_[it->first].recover.set_response(motion_controller_driver->recover_motor());
    }

    // halt
    if (motor_data_[it->first].halt.command_available())
    {
      motor_data_[it->first].halt.set_response(motion_controller_driver->halt_motor());
    }

    // mode switching
    if (motor_data_[it->first].operation_mode.mode_changed())
    {
      RCLCPP_INFO(robot_system_logger, "Switching mode for node %d", it->first);
      auto mode = motor_data_[it->first].operation_mode.get_mode();
      motor_data_[it->first].operation_mode.set_response(
        motion_controller_driver->set_operation_mode(mode));
      RCLCPP_INFO(robot_system_logger, "Switched mode for node %d", it->first);
    }

    const uint16_t & mode = motion_controller_driver->get_mode();

    // Check operation mode and set correct target.
    switch (mode)
    {
      case MotorBase::No_Mode:
        break;
      case MotorBase::Profiled_Position:
      case MotorBase::Cyclic_Synchronous_Position:
      case MotorBase::Interpolated_Position:
        motion_controller_driver->set_target(motor_data_[it->first].target_position);
        break;
      case MotorBase::Profiled_Velocity:
      case MotorBase::Cyclic_Synchronous_Velocity:
        motion_controller_driver->set_target(motor_data_[it->first].target_velocity);
        break;
      case MotorBase::Profiled_Torque:
        motion_controller_driver->set_target(motor_data_[it->first].target_torque);
        break;
      default:
        RCLCPP_INFO(rclcpp::get_logger("robot_system_interface"), "Mode not supported");
    }
  }

  return hardware_interface::return_type::OK;
}

void Cia402RobotSystem::initDeviceContainer()
{
  std::string tmp_master_bin = (info_.hardware_parameters["master_bin"] == "\"\"")
                                 ? ""
                                 : info_.hardware_parameters["master_bin"];

  device_container_->init(
    info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
    info_.hardware_parameters["bus_config"], tmp_master_bin);
  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(
    rclcpp::get_logger("robot_system_interface"), "Number of registered drivers: '%zu'",
    device_container_->count_drivers());
  for (auto it = drivers.begin(); it != drivers.end(); it++)
  {
    uint16_t node_id = it->first;
    std::string driver_type = device_container_->get_driver_type(node_id);
    RCLCPP_INFO(
      robot_system_logger, "Found driver type '%s' at node_id '%u'", driver_type.c_str(), node_id);

    // Check if driver at id is of type ros2_canopen::Cia402Driver
    if (driver_type.compare("ros2_canopen::Cia402Driver") == 0)
    {
      cia402_drivers_.emplace_back(
        std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second));
    }
    else
    {
      RCLCPP_ERROR(
        robot_system_logger, "Driver at node_id '%u' is not of type ros2_canopen::Cia402Driver",
        node_id);
    }
  }
  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

void Cia402RobotSystem::spin()
{
  executor_->spin();
  executor_->remove_node(device_container_);
  RCLCPP_INFO(device_container_->get_logger(), "Stopped spinning Cia402RobotSystem ROS2 executor");
}

void Cia402RobotSystem::clean()
{
  printf("Cancel exectutor...");
  executor_->cancel();
  printf("Join spin thread...");
  spin_thread_->join();

  printf("Reset variables...");
  device_container_.reset();
  executor_.reset();

  init_thread_->join();
  init_thread_.reset();

  executor_.reset();
  spin_thread_.reset();
  motor_data_.clear();
  cia402_drivers_.clear();
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::Cia402RobotSystem, hardware_interface::SystemInterface)
