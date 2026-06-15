#pragma once

#include <opencv2/opencv.hpp>
#include "TmSDK/TmLocalCamera.hxx"
#include "TmSDK/TmFrame.hxx"

class ThermalCamera
{
public:
    ThermalCamera();
    ~ThermalCamera();

    bool initialize();
    bool getFrame(
        cv::Mat& color,
        cv::Mat& temp_map,
        float& min_temp,
        float& max_temp);

private:
    TmSDK::TmLocalCamera* camera_;
    bool initialized_;
};