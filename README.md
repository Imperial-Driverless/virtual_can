# virtual_can
Package emulating a CAN device using data from simulations to test VCU-Car connection

If it's the first time you're cloning this, run `git submodule update --init --recursive` to clone the `FS-AI-API` submodule.

### Dependencies

- All ROS dependencies found in the package manifest as usual.
- `id_msgs` installed
- Ability to communicate through CAN
- If you want to emulate a vehicle, you will need to run the `vcan.sh` (once every reboot) to enable the vcan0 interface

# Nodes

## virtual_can

This node emulates a CAN device from the ADS-DV. It transmits topics from the simulator over CAN for `ros_can` to publish them as ROS topics. It uses the `FS-AI_API` submodule.

### Subscribers

| Topic                      |
Type                                                |
Purpose                                                                                      |

|-------------------------------------------------- |
--------------------------------- |
-------------------------------------------------------------------------------------------- |
| `/imu/data`                |
sensor_msgs/msg/Imu                                 |
IMU data from the simulator                                                                                   |
| `/vcu_drive_feedback`      |
id_msgs/msg/VCUDriveFeedback |
Processed wheel speeds using one of the VCU feedback processors

### Parameters

| Topic                       | Type   | Default | Purpose                                                                                    |
| --------------------------- | ------ | ------- | ------------------------------------------------------------------------------------------ |
| `can_interface`             | string | can0    | Sets the CAN interface in FS-AI library.                                                   |
| `can_debug`                 | int    | 0       | Starts the FS-AI library in debug mode. Set to something other than 0 to enable            |
| `can_simulate`              | int    | 0       | Starts the FS-AI library with simulated CAN data. Set to something other than 0 to enable  |
| `debug_logging`             | bool   | false   | Enable debug logging.                                                                      |

# Launch files

| Filename                 | Purpose                                                                             |
| ------------------- | ----------------------------------------------------------------------------------- |
| `virtual_can.launch.py` | Starts the `virtual_can` node in normal working operation configured for the ADS-DV car                 |

### TODO
- Implement state machine of the vehicle + services to modify it
- Test on lightweight_lidar_only_simulator
- Implement EBS service