#pragma once

#include <opencv2/opencv.hpp>
#include "TmSDK/TmLocalCamera.hxx"
#include "TmSDK/TmFrame.hxx"

class ThermalCamera
{
public:
    ThermalCamera();
    ~ThermalCamera();

    bool initialize(const std::string& device_port = "");
    bool close() noexcept;
    bool configureGain(const std::string& mode);
    bool enableScheduledFfc();
    bool runFfc();
    bool getFrame(
        cv::Mat& color,
        cv::Mat& temp_map,
        float& min_temp,
        float& max_temp);

private:
    TmSDK::TmLocalCamera* camera_;
    bool initialized_;
    int previous_ffc_mode_ = -1;
    bool restore_ffc_mode_ = false;
    std::unique_ptr<TmSDK::TmLocalCamInfo> selected_info_;
};
