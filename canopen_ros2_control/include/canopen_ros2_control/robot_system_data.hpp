#include <cmath>
#include <cstdint>
#include <limits>

namespace canopen_ros2_control
{
/**
 * @brief Struct for storing the data necessary for a triggering command.
 *
 * @details
 * A trigger command from a controller will write to the command double.
 * The result of the trigger operation will be written to the response double.
 * A result of 1.0 means the operation was successful.
 * A result of 0.0 means the operation failed.
 * Only a trigger command of 1.0 will be accepted and trigger the operation.
 * The command_available function can be used to check if a command is available,
 * undefined commands (other than 1.0) will be ignored.
 * The set_response function should be used to set the response value.
 * It will then clear the command.
 *
 */
struct MotorTriggerCommand
{
  double command = std::numeric_limits<double>::quiet_NaN();
  double response = std::numeric_limits<double>::quiet_NaN();

  bool command_available()
  {
    if (command == 1.0)
    {
      return true;
    }
    command = std::numeric_limits<double>::quiet_NaN();
    return false;
  }

  void set_response(bool response)
  {
    command = std::numeric_limits<double>::quiet_NaN();
    this->response = response ? 1.0 : 0.0;
  }
};

/**
 * @brief Struct for motor mode switch commands.
 *
 * The motor mode switch command expects a mode in double format.
 * See canopen_cia402_driver/base.hpp
 *
 * No_Mode = 0.0
 * Profiled_Position = 1.0
 * Velocity = 2.0
 * Profiled_Velocity = 3.0
 * Profiled_Torque = 4.0
 * Reserved = 5.0
 * Homing = 6.0
 * Interpolated_Position = 7.0
 * Cyclic_Synchronous_Position = 8.0
 * Cyclic_Synchronous_Velocity = 9.0
 * Cyclic_Synchronous_Torque = 10.0
 *
 * The mode change function enables checking if the mode has changed
 * and a switch is necessary. The get_mode function returns the
 * mode in integer format. The set_mode function sets the response
 * value (results of the actual mode setting function).
 *
 */
struct MotorModeSwitchCommand
{
  double mode{std::numeric_limits<double>::quiet_NaN()};
  double resp{std::numeric_limits<double>::quiet_NaN()};
  double old_mode{std::numeric_limits<double>::quiet_NaN()};

  /**
   * @brief Check if the mode has changed.
   * @return true
   * @return false
   */
  bool mode_changed()
  {
    if (!std::isnan(mode) && mode != old_mode)
    {
      old_mode = mode;
      return true;
    }
    return false;
  }

  /**
   * @brief Get the mode value.
   * @return int8_t
   */
  uint16_t get_mode() { return static_cast<uint16_t>(mode); }

  /**
   * @brief Set the response value.
   * @param response
   */
  void set_response(bool response)
  {
    if (response)
    {
      resp = 1.0;
      old_mode = mode;
    }
    else
    {
      resp = 0.0;
    }
  }
};

}  // namespace canopen_ros2_control
