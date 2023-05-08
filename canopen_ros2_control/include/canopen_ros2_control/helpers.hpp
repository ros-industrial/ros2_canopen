#ifndef CANOPEN_ROS2_CONTROL__HELPER_HPP_
#define CANOPEN_ROS2_CONTROL__HELPER_HPP_

#include <cmath>
#include <cstdint>
#include <functional>
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
struct TriggerCommand
{
  double command = std::numeric_limits<double>::quiet_NaN();
  double response = std::numeric_limits<double>::quiet_NaN();
  std::function<bool()> trigger_function;

  bool try_trigger()
  {
    if (command == 1.0)
    {
      response = trigger_function() ? 1.0 : 0.0;
      command = std::numeric_limits<double>::quiet_NaN();
      return true;
    }
    command = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
};

}  // namespace canopen_ros2_control
#endif
