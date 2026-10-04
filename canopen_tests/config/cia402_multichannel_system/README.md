# Multi-Channel CiA 402 System Configuration

This configuration demonstrates the multi-channel/multi-axes feature as described in CiA 302-7 and CiA 402-2 specifications.

## Overview

Multi-channel devices allow controlling multiple independent axes over a single CANopen Node-ID. This is useful for:
- Multi-axis motor controllers (e.g., dual-axis servo drives)
- Coordinated motion control systems
- Reducing network complexity by consolidating multiple axes into one node

## CiA 402-2 Specification

According to CiA 402-2, multi-axis devices use offset object dictionary indices:

- **Channel 0** (standard): Indices 0x6000-0x67FF
- **Channel 1**: Indices 0x6800-0x6FFF (offset +0x800)
- **Channel 2**: Indices 0x7000-0x77FF (offset +0x1000)
- **Channel N**: Indices 0x6000 + (N * 0x800)

### Common Object Examples

| Object | Channel 0 | Channel 1 | Channel 2 |
|--------|-----------|-----------|-----------|
| Controlword | 0x6040 | 0x6840 | 0x7040 |
| Statusword | 0x6041 | 0x6841 | 0x7041 |
| Modes of operation | 0x6060 | 0x6860 | 0x7060 |
| Position actual value | 0x6064 | 0x6864 | 0x7064 |
| Velocity actual value | 0x606C | 0x686C | 0x706C |
| Target position | 0x607A | 0x687A | 0x707A |
| Target velocity | 0x60FF | 0x68FF | 0x70FF |

## Configuration Structure

### Bus Configuration (bus.yml)

The bus configuration defines:
1. PDO mappings for each channel with appropriate index offsets
2. SDO initialization values for each channel
3. A single node_id that hosts multiple channels

### Hardware Description (URDF)

When using ros2_control, the hardware interface must specify the channel for each joint:

```xml
<ros2_control name="cia402_multichannel_system" type="system">
  <hardware>
    <plugin>canopen_ros2_control/Cia402System</plugin>
    <param name="bus_config">$(find canopen_tests)/config/cia402_multichannel_system/bus.yml</param>
    <param name="master_config">$(find canopen_tests)/config/master.dcf</param>
    <param name="can_interface_name">vcan0</param>
    <param name="master_bin"></param>
  </hardware>

  <!-- First axis (channel 0) -->
  <joint name="node_2_channel_0">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="node_id">2</param>
    <param name="channel">0</param>  <!-- Channel specification -->
  </joint>

  <!-- Second axis (channel 1) -->
  <joint name="node_2_channel_1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="node_id">2</param>
    <param name="channel">1</param>  <!-- Channel specification -->
  </joint>
</ros2_control>
```

### Key Parameters

- **node_id**: The CANopen node ID of the device
- **channel**: The channel/axis number within that device (0, 1, 2, ...)
  - Defaults to 0 if not specified (backward compatible)
  - Used to calculate the object dictionary index offset

## Backward Compatibility

The multi-channel feature is fully backward compatible:

- If the `channel` parameter is not specified, it defaults to 0
- Channel 0 uses standard object indices (no offset)
- Existing single-axis configurations work without modification

## Usage Example

1. Configure the bus with appropriate PDO mappings for all channels
2. In your URDF, specify joints with their `node_id` and `channel`
3. Use standard ros2_control interfaces to control each axis independently
4. Each channel operates as an independent joint in the system

## Hardware Requirements

Your CANopen device must:
- Support CiA 402-2 multi-axis specification
- Implement object dictionary entries at the correct offset indices
- Have appropriate EDS/DCF file defining all channel objects

## Testing

To test this configuration with simulated devices:

```bash
# Set up virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0

# Launch the multi-channel system
ros2 launch canopen_tests cia402_multichannel_system_setup.launch.py
```

## References

- CiA 302-7: Framework for programmable CANopen devices - Part 7: Multi-axis positioning controller
- CiA 402-2: CANopen device profile for drives and motion control - Part 2: Multiple axis communication parameter
