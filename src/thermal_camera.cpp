#include "thermal_cam_driver/thermal_camera.hpp"
#include <iostream>

using namespace TmSDK;

ThermalCamera::ThermalCamera()
: camera_(nullptr),
  initialized_(false)
{
}

ThermalCamera::~ThermalCamera()
{
    if (camera_)
    {
        try
        {
            camera_->Close();
        }
        catch (...)
        {
            std::cout << "[WARN] Camera Close failed" << std::endl;
        }

        delete camera_;
        camera_ = nullptr;
    }
}

bool ThermalCamera::initialize()
{
    std::cout << "[INFO] GetCameraList..." << std::endl;

    auto list = TmLocalCamera::GetCameraList();

    std::cout << "[INFO] Camera count = " << list.size() << std::endl;

    if (list.empty())
    {
        std::cout << "[ERROR] No camera found" << std::endl;
        return false;
    }

    camera_ = new TmLocalCamera();

    if (!camera_)
    {
        std::cout << "[ERROR] Allocation failed" << std::endl;
        return false;
    }

    std::cout << "[INFO] Opening camera..." << std::endl;

    bool ok = false;

    try
    {
        ok = camera_->Open(&list[0]);
    }
    catch (const std::exception& e)
    {
        std::cout << "[EXCEPTION] Open(): " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cout << "[FATAL] Open crashed" << std::endl;
        return false;
    }

    if (!ok)
    {
        std::cout << "[ERROR] Open failed" << std::endl;
        return false;
    }

    try
    {
        camera_->SetColorMap(TmSDK::ColormapTypes::Inferno);
    }
    catch (...)
    {
        std::cout << "[WARN] SetColorMap failed" << std::endl;
    }

    initialized_ = true;

    std::cout << "[INFO] Camera initialized OK" << std::endl;

    return true;
}

bool ThermalCamera::getFrame(
    cv::Mat& color,
    cv::Mat& temp_map,
    float& min_temp,
    float& max_temp)
{
    if (!camera_ || !initialized_)
        return false;

    TmFrame tm_frame(TmSDK::ColormapTypes::Inferno);

    bool ok = false;

    try
    {
        ok = camera_->QueryFrame(&tm_frame, 0, 0);
    }
    catch (...)
    {
        std::cout << "[ERROR] QueryFrame crash" << std::endl;
        return false;
    }

    if (!ok)
    {
        std::cout << "[WARN] QueryFrame failed" << std::endl;
        return false;
    }

    // Temperature statistics
    // Find min/max raw values and convert to temperatures
    double minVal = 0.0, avgVal = 0.0, maxVal = 0.0;
    TmSDK::Point minLoc, maxLoc;

    bool stat_ok = false; // MinMaxLoc status

    try
    {
        // Check return value of MinMaxLoc and catch exceptions
        stat_ok = tm_frame.MinMaxLoc(minVal, avgVal, maxVal, minLoc, maxLoc);
    }
    catch (...)
    {
        std::cout << "[WARN] MinMaxLoc failed" << std::endl;
        stat_ok = false;
    }

    if (stat_ok)
    {
        try
        {
            // Convert raw thermal values to Celsius
            min_temp = static_cast<float>(camera_->GetTemperature(minVal));
        }
        catch (...)
        {
            std::cout << "[WARN] GetTemperature(min) failed" << std::endl;
            min_temp = 0.0f;
        }

        try
        {
            // Convert raw thermal values to Celsius
            max_temp = static_cast<float>(camera_->GetTemperature(maxVal));
        }
        catch (...)
        {
            std::cout << "[WARN] GetTemperature(max) failed" << std::endl;
            max_temp = 0.0f;
        }
    }
    else
    {
        // MinMaxLoc failed or returned false — set sentinel temps
        min_temp = 0.0f;
        max_temp = 0.0f;
    }

    uint8_t* bitmap = nullptr;

    try
    {
        bitmap = tm_frame.ToBitmap(TmSDK::ColorOrder::COLOR_RGB);
    }
    catch (...)
    {
        std::cout << "[ERROR] ToBitmap crash" << std::endl;
        return false;
    }

    if (!bitmap)
        return false;

    int w = tm_frame.Width();
    int h = tm_frame.Height();

    if (w <= 0 || h <= 0)
        return false;

    // Fill color image (preserve existing behavior)
    cv::Mat rgb(h, w, CV_8UC3, bitmap);
    cv::cvtColor(rgb, color, cv::COLOR_RGB2BGR);

    // Create temperature map (float32, Celsius per pixel)
    try
    {
        temp_map = cv::Mat(h, w, CV_32FC1);

        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                double raw = 0.0;

                try
                {
                    raw = tm_frame.GetPixel(x, y);
                }
                catch (...)
                {
                    raw = 0.0;
                }

                float c = 0.0f;

                try
                {
                    c = static_cast<float>(camera_->GetTemperature(raw));
                }
                catch (...)
                {
                    c = 0.0f;
                }

                temp_map.at<float>(y, x) = c;
            }
        }
    }
    catch (...)
    {
        std::cout << "[ERROR] Creating temp_map failed" << std::endl;
        return false;
    }

    return true;
}