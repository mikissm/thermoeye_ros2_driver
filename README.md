# thermoeye_ros2_driver

ROS2 driver for ThermoEye thermal cameras based on TmSDK.

## Features

* Thermal camera initialization via TmSDK
* Real-time thermal image acquisition
* ROS2 image publishing
* OpenCV integration
* Tested on ROS2 Humble (Ubuntu 22.04)

## Environment

* Ubuntu 22.04
* ROS2 Humble
* OpenCV 4.x
* TmSDK 2.1.0

## Prerequisites

Install TmSDK separately before building this package.

Official TmSDK release:

https://github.com/ThermoEye/TmSDK/releases/tag/2.1.0

Example:

```bash
sudo dpkg -i tmsdk_2.1.0_amd64.deb
```

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

| Topic              | Type                  | Description          |
| ------------------ | --------------------- | -------------------- |
| /thermal/image_raw | sensor_msgs/msg/Image | Thermal image stream |

## Package Structure

```text
thermal_cam_driver
├── include/
├── src/
├── CMakeLists.txt
└── package.xml
```

## Hardware

* ThermoEye thermal camera

## Dependencies

* ROS2 Humble
* OpenCV
* cv_bridge
* sensor_msgs
* TmSDK

## Disclaimer

This repository contains only the ROS2 integration layer.

TmSDK is developed and distributed by ThermoEye and is not included in this repository. Users must obtain and install the SDK separately.
