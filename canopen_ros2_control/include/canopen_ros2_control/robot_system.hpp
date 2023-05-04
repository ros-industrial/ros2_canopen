#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_core/device_container.hpp"
#include "canopen_ros2_control/robot_system_data.hpp"
#include "canopen_ros2_control/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
namespace canopen_ros2_control
{
class Cia402RobotSystem : public hardware_interface::SystemInterface
{
  struct Cia402MotorNodeData
  {
    // FROM MOTOR
    double actual_position = std::numeric_limits<double>::quiet_NaN();
    double actual_velocity = std::numeric_limits<double>::quiet_NaN();

    // TO MOTOR
    double target_position = std::numeric_limits<double>::quiet_NaN();
    double target_velocity = std::numeric_limits<double>::quiet_NaN();
    double target_torque = std::numeric_limits<double>::quiet_NaN();

    // COMMANDS
    MotorTriggerCommand init;
    MotorTriggerCommand recover;
    MotorTriggerCommand halt;

    MotorModeSwitchCommand operation_mode;
  };

public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  Cia402RobotSystem() : hardware_interface::SystemInterface() {}
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~Cia402RobotSystem() = default;

  /**
   * @brief Initialize the hardware interface
   *
   * Fetch the hardware info stored in the urdf.
   * Specifically these values:
   * - bus_config
   * - master_config
   * - can_interface_name
   * - master_bin
   *
   * @param info
   * @return hardware_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the hardware interface
   *
   * Create the device container and the executor.
   * Start the threads for running the canopen stack.
   *
   * @param info
   * @return hardware_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export the state interfaces for this system.
   *
   * The state interface of each cia402 device contains the following:
   * - Position
   * - Velocity
   *
   * @return std::vector<hardware_interface::StateInterface>
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export the command interfaces for this system.
   *
   * The command interface of each cia402 device contains the following:
   * - Position
   * - Velocity
   * - Effort
   * - Operation Mode
   * - Init Command
   * - Halt Command
   * - Recover Command
   *
   * @return std::vector<hardware_interface::CommandInterface>
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Cleanup the hardware interface
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Shutdown the hardware interface
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the hardware interface
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface
   *
   * @param previous_state
   * @return hardware_interface::CallbackReturn
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read the state from the hardware interface
   *
   * @return hardware_interface::return_type
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write the command to the hardware interface
   *
   * @return hardware_interface::return_type
   */
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::map<uint, Cia402MotorNodeData> motor_data_;
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> init_thread_;
  std::vector<std::shared_ptr<ros2_canopen::Cia402Driver>> cia402_drivers_;

private:
  /**
   * @brief Spins the ROS executor
   *
   */
  void spin();
  void clean();

  /**
   * @brief Initialize the device container
   *
   */
  void initDeviceContainer();
};
}  // namespace canopen_ros2_control
