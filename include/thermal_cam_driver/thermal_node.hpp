#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include "thermal_cam_driver/thermal_camera.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class ThermalNode : public rclcpp::Node
{
public:
    ThermalNode();
    void stop();

private:
    bool startup_calibration_ = false;
    bool disable_auto_ffc_ = false;
    bool measurement_enabled_ = false;
    uint64_t received_frames_ = 0;
    uint64_t empty_polls_ = 0;
    bool calibration_confirmable_ = false;
    std::chrono::steady_clock::time_point last_preview_{};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_, confirm_service_;
    image_transport::Publisher preview_pub_;
    void timerCallback();
    void publishFfcState(const std::string& state, bool valid);
    double ffc_interval_s_ = 0;
    double ffc_recovery_s_ = 2;
    bool ffc_failed_ = false;
    bool ffc_recovering_ = false;
    std::chrono::steady_clock::time_point next_ffc_, resume_at_;
    std::string last_ffc_stamp_ = "never";
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ffc_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    image_transport::Publisher image_color_pub_;
    image_transport::Publisher temp_map_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    ThermalCamera thermal_camera_;
    bool camera_available_;
};
