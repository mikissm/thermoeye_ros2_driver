#include "thermal_cam_driver/thermal_node.hpp"
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

ThermalNode::ThermalNode()
: Node("thermal_node")
{
    thermal_camera_.initialize();

    image_color_pub_ =
        image_transport::create_publisher(
            this,
            "/thermal/image_color"
        );

    temp_map_pub_ =
        image_transport::create_publisher(
            this,
            "/thermal/temp_map"
        );

    max_temp_pub_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/thermal/max_temp",
            rclcpp::QoS(10)
        );

    min_temp_pub_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/thermal/min_temp",
            rclcpp::QoS(10)
        );

    camera_info_pub_ =
        this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/thermal/camera_info",
            rclcpp::QoS(10)
        );

    timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ThermalNode::timerCallback, this)
        );

    RCLCPP_INFO(
        this->get_logger(),
        "thermal_cam_driver started"
    );
}

void ThermalNode::timerCallback()
{
    cv::Mat color;
    cv::Mat temp_map;

    float min_temp = 0.0f;
    float max_temp = 0.0f;

    if (!thermal_camera_.getFrame(color, temp_map, min_temp, max_temp))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to get frame"
        );

        return;
    }

    // Color image
    auto color_msg =
        cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            color
        ).toImageMsg();

    color_msg->header.stamp = this->now();
    color_msg->header.frame_id = "thermal_camera";

    image_color_pub_.publish(*color_msg);

    // Temperature map (32FC1)
    auto temp_msg =
        cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "32FC1",
            temp_map
        ).toImageMsg();

    temp_msg->header.stamp = color_msg->header.stamp;
    temp_msg->header.frame_id = "thermal_camera";

    temp_map_pub_.publish(*temp_msg);

    // CameraInfo
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.width = 160;
    camera_info.height = 120;
    camera_info.header.stamp = color_msg->header.stamp;
    camera_info.header.frame_id = "thermal_camera";
    camera_info_pub_->publish(camera_info);

    std_msgs::msg::Float32 max_msg;
    max_msg.data = max_temp;
    max_temp_pub_->publish(max_msg);

    std_msgs::msg::Float32 min_msg;
    min_msg.data = min_temp;
    min_temp_pub_->publish(min_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ThermalNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}