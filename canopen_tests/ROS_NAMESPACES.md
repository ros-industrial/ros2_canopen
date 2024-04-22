The `cia402_namespaced_system.launch.py` example demonstrates how to run a
CIA402 system in a namespace to allow multiple CANOpen-based robots in the
same ROS domain. This example is necessary because the controller names
are picked up from `ros2_controllers.yaml` and we must define their names
in that file with a `__namespace__/controller_name:` key, then use
ReplaceString from `nav2_common` to dynamically add the namespace to the
controller definition.

For a proxy system, no example is included; it's sufficient to include, e.g.
the `proxy_setup.launch.py` launch description in another launch file and
push a namespace onto it with PushROSNamespace in the ordinary way.
