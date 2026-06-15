# thermoeye_ros2_driver

ROS2 driver for ThermoEye thermal cameras based on TmSDK.

## Features

* Thermal camera initialization through TmSDK
* Real-time thermal image acquisition
* ROS2 image publishing
* Compatible with ROS2 Humble
* OpenCV integration

## Environment

* Ubuntu 22.04
* ROS2 Humble
* OpenCV 4.x
* ThermoEye TmSDK

## Build

```bash
cd ~/vision2601_ws
colcon build --packages-select thermal_cam_driver
source install/setup.bash
```

## Run

```bash
ros2 run thermal_cam_driver thermal_node
```

## Published Topics

| Topic              | Type              | Description          |
| ------------------ | ----------------- | -------------------- |
| /thermal/image_raw | sensor_msgs/Image | Thermal image stream |

## Hardware

* ThermoEye Thermal Camera

## Dependencies

* ROS2 Humble
* OpenCV
* cv_bridge
* image_transport
* ThermoEye TmSDK

## Notice

This repository provides a ROS2 integration layer for ThermoEye thermal cameras.

The original camera SDK (TmSDK) is developed and distributed by ThermoEye. This project does not redistribute or claim ownership of the SDK.
