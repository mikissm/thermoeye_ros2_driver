#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "thermal_cam_driver/thermal_camera.hpp"

#include <std_msgs/msg/float32.hpp>

class ThermalNode : public rclcpp::Node
{
public:
    ThermalNode();

private:
    void timerCallback();

    rclcpp::TimerBase::SharedPtr timer_;

    image_transport::Publisher image_color_pub_;
    image_transport::Publisher temp_map_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    ThermalCamera thermal_camera_;
    bool camera_available_;
};
