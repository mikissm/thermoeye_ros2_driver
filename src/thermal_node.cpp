#include "thermal_cam_driver/thermal_node.hpp"
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <iomanip>
#include <thread>

ThermalNode::ThermalNode()
: Node("thermal_node")
{
    const std::string gain_mode = declare_parameter<std::string>("gain_mode", "low");
    if (gain_mode != "low" && gain_mode != "high" && gain_mode != "unchanged")
        throw std::invalid_argument("gain_mode must be low, high or unchanged");
    const auto device_port = declare_parameter<std::string>("device_port", "");
    camera_available_ = thermal_camera_.initialize(device_port);
    if (!camera_available_)
        throw std::runtime_error("Thermal camera initialization failed");
    const double poll_rate = declare_parameter("poll_rate_hz", 9.0);
    if (!std::isfinite(poll_rate) || poll_rate <= 0.0 || poll_rate > 60.0)
        throw std::invalid_argument("poll_rate_hz must be finite and in (0, 60]");
    ffc_interval_s_ = declare_parameter("ffc_interval_s", 0.0);
    ffc_recovery_s_ = declare_parameter("ffc_recovery_s", 2.0);
    startup_calibration_ = declare_parameter("startup_calibration", false);
    disable_auto_ffc_ = declare_parameter("disable_auto_ffc", false);
    measurement_enabled_ = !startup_calibration_;
    if (startup_calibration_ && ffc_interval_s_ != 0)
        throw std::invalid_argument("startup_calibration requires ffc_interval_s=0");
    if (!std::isfinite(ffc_interval_s_) || ffc_interval_s_ < 0 ||
        !std::isfinite(ffc_recovery_s_) || ffc_recovery_s_ < 0.5)
        throw std::invalid_argument("FFC interval must be >=0, recovery must be >=0.5 seconds");
    if (ffc_interval_s_ > 0 && !disable_auto_ffc_)
        throw std::invalid_argument("Scheduled FFC requires disable_auto_ffc=true");
    if (disable_auto_ffc_ && !thermal_camera_.enableScheduledFfc())
        throw std::runtime_error("Cannot establish manual FFC mode; periodic FFC disabled safely");
    if (!thermal_camera_.configureGain(gain_mode))
        throw std::runtime_error("Gain configuration/readback failed; measurements not published");
    next_ffc_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(ffc_interval_s_));
    ffc_status_pub_ = create_publisher<std_msgs::msg::String>(
        "/thermal/ffc/status", rclcpp::QoS(1).transient_local());
    publishFfcState("waiting_for_frame", false);
    preview_pub_ = image_transport::create_publisher(this, "/thermal/calibration/preview");
    calibrate_service_ = create_service<std_srvs::srv::Trigger>("/thermal/calibration/run",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            if (!disable_auto_ffc_) {
                response->success = false;
                response->message = "Enable disable_auto_ffc before using manual calibration";
                return;
            }
            measurement_enabled_ = false;
            calibration_confirmable_ = false;
            last_ffc_stamp_ = std::to_string(now().nanoseconds());
            publishFfcState("correcting", false);
            try { response->success = thermal_camera_.runFfc(); }
            catch (const std::exception& e) { response->message = e.what(); }
            ffc_failed_ = !response->success;
            ffc_recovering_ = response->success;
            resume_at_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(ffc_recovery_s_));
            if (response->message.empty()) response->message = response->success ? "Inspect preview before confirming" : "FFC failed; measurement blocked";
            publishFfcState(response->success ? "recovering" : "failed", false);
        });
    confirm_service_ = create_service<std_srvs::srv::Trigger>("/thermal/calibration/confirm",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            response->success = (!disable_auto_ffc_ || calibration_confirmable_) && !ffc_failed_ && !ffc_recovering_ &&
                std::chrono::steady_clock::now() - last_preview_ < std::chrono::milliseconds(500);
            measurement_enabled_ = response->success;
            response->message = response->success ? "Measurement enabled" : "Wait for required calibration and a fresh frame";
        });

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
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / poll_rate)),
            std::bind(&ThermalNode::timerCallback, this)
        );

    RCLCPP_INFO(
        this->get_logger(),
        "thermal_cam_driver started"
    );
}

void ThermalNode::publishFfcState(const std::string& state, bool valid)
{
    std_msgs::msg::String msg;
    msg.data = "{\"state\":\"" + state + "\",\"valid\":" +
        (valid ? "true" : "false") + ",\"last_attempt_ros_ns\":\"" +
        last_ffc_stamp_ + "\",\"interval_s\":" + std::to_string(ffc_interval_s_) + "}";
    ffc_status_pub_->publish(msg);
}

void ThermalNode::timerCallback()
{
    if (!camera_available_)
        return;
    const auto steady_now = std::chrono::steady_clock::now();
    if (ffc_interval_s_ > 0 && steady_now >= next_ffc_) {
        publishFfcState("correcting", false);
        last_ffc_stamp_ = std::to_string(now().nanoseconds());
        bool success = false;
        try { success = thermal_camera_.runFfc(); }
        catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "FFC exception: %s", error.what());
        }
        ffc_failed_ = !success;
        ffc_recovering_ = success;
        const auto finished = std::chrono::steady_clock::now();
        next_ffc_ = finished + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(ffc_interval_s_));
        resume_at_ = finished + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(ffc_recovery_s_));
        publishFfcState(success ? "recovering" : "failed", false);
        if (success) RCLCPP_INFO(get_logger(), "FFC succeeded; publication paused for %.1fs", ffc_recovery_s_);
        else RCLCPP_ERROR(get_logger(), "FFC failed; temperature publication blocked until successful retry");
        return;
    }
    if (ffc_failed_) return;

    cv::Mat color;
    cv::Mat temp_map;

    float min_temp = 0.0f;
    float max_temp = 0.0f;

    if (!thermal_camera_.getFrame(color, temp_map, min_temp, max_temp))
    {
        ++empty_polls_;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(), 2000,
            "Failed to get frame"
        );
        publishFfcState("frame_unavailable", false);
        calibration_confirmable_ = false;
        return;
    }
    if (++received_frames_ == 1)
        RCLCPP_INFO(get_logger(), "First frame received: %dx%d", color.cols, color.rows);
    // Drain SDK frames during recovery; do not relabel them as fresh measurements.
    if (ffc_recovering_) {
        if (steady_now < resume_at_) return;
        ffc_recovering_ = false;
        calibration_confirmable_ = true;
        return;  // discard the first frame at the recovery boundary as well
    }
    if (!disable_auto_ffc_ || last_ffc_stamp_ != "never") calibration_confirmable_ = true;
    publishFfcState(measurement_enabled_ ? "streaming" : (calibration_confirmable_ ? "awaiting_confirmation" : "awaiting_calibration"), measurement_enabled_);
    if (!measurement_enabled_) {
        auto preview = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color).toImageMsg();
        preview->header.stamp = now();
        preview->header.frame_id = "thermal_camera";
        preview_pub_.publish(*preview);
        last_preview_ = std::chrono::steady_clock::now();
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
    camera_info.width = color.cols;
    camera_info.height = color.rows;
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

void ThermalNode::stop()
{
    if (timer_) timer_->cancel();
    camera_available_ = false;
    measurement_enabled_ = false;
    publishFfcState("closing", false);
    RCLCPP_INFO(get_logger(), "Acquisition stopped: frames=%llu empty_polls=%llu",
                static_cast<unsigned long long>(received_frames_),
                static_cast<unsigned long long>(empty_polls_));
    if (!thermal_camera_.close()) {
        publishFfcState("close_failed", false);
        throw std::runtime_error("SDK cleanup reported an error; inspect CLOSE log");
    }
    publishFfcState("disconnected", false);
}

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
}

int main(int argc, char* argv[])
{
    // Scanner does not Open a device or write any sensor setting.
    if (argc == 2 && std::string(argv[1]) == "--list-cameras") {
        try {
            for (const auto& info : TmSDK::TmLocalCamera::GetCameraList())
                std::cout << "CAMERA " << std::quoted(info.Name) << ' '
                          << std::quoted(info.ComPort) << ' ' << info.Index << std::endl;
            return 0;
        } catch (const std::exception& error) {
            std::cerr << error.what() << std::endl;
            return 1;
        }
    }
    // Keep ROS alive until SDK teardown finishes. Signals only request a stop;
    // they never destroy a camera or ROS context inside a signal handler.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    try
    {
        auto node = std::make_shared<ThermalNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        while (!stop_requested && rclcpp::ok()) {
            executor.spin_once(std::chrono::milliseconds(50));
        }
        executor.remove_node(node);
        node->stop();
        node.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "thermal_node failed: " << error.what() << std::endl;
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
