# ROS2 CANopen

[![Build Status](https://github.com/ros-industrial/ros2_canopen/workflows/rolling/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)
[![Documentation Status](https://github.com/ros-industrial/ros2_canopen/workflows/Documentation/badge.svg?branch=master)](https://github.com/ros-industrial/ros2_canopen/actions)


## Documentation
The documentation is generated using sphinx and doxygen.
The current manual can be found [here](https://ros-industrial.github.io/ros2_canopen/manual/).
The current api reference can be found [here](https://ros-industrial.github.io/ros2_canopen/api/).

## Status
Currently under development. Not for production use.

**Available Features:**
* Device Manager (using rclcpp::components)
* MasterDriver (Service Interface)
* ProxyDriver (Service Interface)
* Cia402Driver (Service Interface)
* Generic ros2_control Interface (implementing `hardware_interface::SystemInterface`) - check https://control.ros.org for more details

## Contributing
This repository uses `pre-commit` for code formatting.
This program has to be setup locally and installed inside the repository.
For this execute in the repository folder following commands:
```
sudo apt install -y pre-commit
pre-commit install
```
The checks are automatically executed before each commit.
This helps you to always commit well formatted code.
To run all the checks manually use `pre-commit run -a` command.
For the other options check `pre-commit --help`.

In a case of an "emergency" you can avoid execution of pre-commit hooks by adding `-n` flag to `git commit` command - this is NOT recommended to do if you don't know what are you doing!
