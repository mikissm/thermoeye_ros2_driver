#include "thermal_cam_driver/thermal_camera.hpp"
#include <iostream>
#include <algorithm>
#include <stdexcept>

using namespace TmSDK;

ThermalCamera::ThermalCamera()
: camera_(nullptr),
  initialized_(false)
{
}

ThermalCamera::~ThermalCamera()
{
    close();
}

bool ThermalCamera::close() noexcept
{
    if (!camera_) return true;
    bool success = true;
    initialized_ = false;
    const auto started = std::chrono::steady_clock::now();
    // Match the viewer disconnect sequence. Polling has no registered callbacks,
    // but ending acquisition first also handles SDK-owned dispatch resources.
    auto step = [&success](const char* name, auto action) {
        std::cout << "[CLOSE] " << name << " begin" << std::endl;
        try { action(); }
        catch (...) {
            success = false;
            std::cerr << "[CLOSE] " << name << " failed" << std::endl;
        }
        std::cout << "[CLOSE] " << name << " end" << std::endl;
    };
    step("EndAcquisition", [&] { camera_->EndAcquisition(); });
    step("UnregisterEventHandler", [&] { camera_->UnregisterEventHandler(); });
    step("UnregisterConnectionEventHandler", [&] { camera_->UnregisterConnectionEventHandler(); });
    if (restore_ffc_mode_) {
        step("RestoreFfcMode", [&] {
            auto* control = camera_->GetTmControl();
            if (!control || !control->SetFlatFieldCorrectionMode(previous_ffc_mode_) ||
                    control->GetFlatFieldCorrectionMode() != previous_ffc_mode_)
                throw std::runtime_error("FFC mode restoration/readback failed");
        });
        restore_ffc_mode_ = false;
    }
    step("Close", [&] { camera_->Close(); });
    step("Delete", [&] { delete camera_; });
    camera_ = nullptr;
    selected_info_.reset();
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::cout << "[CLOSE] complete success=" << success << " elapsed_s=" << elapsed << std::endl;
    return success;
}

bool ThermalCamera::enableScheduledFfc()
{
    auto* control = camera_ ? camera_->GetTmControl() : nullptr;
    if (!control) return false;
    previous_ffc_mode_ = control->GetFlatFieldCorrectionMode();
    std::cout << "[INFO] Original FFC mode=" << previous_ffc_mode_
              << ", gain=" << control->GetGainModeState() << std::endl;
    if (previous_ffc_mode_ != 0 && previous_ffc_mode_ != 1) return false;
    if (previous_ffc_mode_ == 1) {
        if (!control->SetFlatFieldCorrectionMode(0)) return false;
        // Restore the original mode during orderly teardown, including failures
        // after the setter succeeds but before setup completes.
        restore_ffc_mode_ = true;
        if (control->GetFlatFieldCorrectionMode() != 0) return false;
    }
    return true;
}

bool ThermalCamera::runFfc()
{
    auto* control = camera_ ? camera_->GetTmControl() : nullptr;
    return control && control->RunFlatFieldCorrection();
}

bool ThermalCamera::configureGain(const std::string& mode)
{
    auto* control = camera_ ? camera_->GetTmControl() : nullptr;
    if (!control) return false;
    const int before = control->GetGainModeState();
    if (mode == "unchanged") {
        std::cout << "[INFO] Gain unchanged, readback=" << before << std::endl;
        return true;
    }
    const int target = mode == "low" ? 1 : mode == "high" ? 0 : -1;
    if (target < 0) return false;
    if (before != target && !control->SetGainModeState(target)) return false;
    const int after = control->GetGainModeState();
    std::cout << "[INFO] Gain requested=" << mode << " before=" << before
              << " readback=" << after << std::endl;
    return after == target;
}

bool ThermalCamera::initialize(const std::string& device_port)
{
    std::cout << "[INFO] GetCameraList..." << std::endl;

    auto list = TmLocalCamera::GetCameraList();

    std::cout << "[INFO] Camera count = " << list.size() << std::endl;

    if (list.empty())
    {
        std::cout << "[ERROR] No camera found" << std::endl;
        return false;
    }

    auto selected = list.begin();
    if (!device_port.empty()) {
        selected = std::find_if(list.begin(), list.end(), [&](const auto& info) {
            return info.ComPort == device_port;
        });
        if (selected == list.end()) {
            std::cerr << "[ERROR] Selected camera disappeared: " << device_port << std::endl;
            return false;
        }
    }
    selected_info_ = std::make_unique<TmLocalCamInfo>(*selected);
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
        std::cout << "[OPEN] " << selected_info_->Name << " port="
                  << selected_info_->ComPort << " index=" << selected_info_->Index << std::endl;
        ok = camera_->Open(selected_info_.get());
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
    camera_->SetTempUnit(TmSDK::TempUnit::CELSIUS);

    // Read-only startup evidence for afterimage investigation; do not change gain.
    if (auto* control = camera_->GetTmControl()) {
        try {
            std::cout << "[INFO] Sensor model=" << control->GetProductModelName()
                      << " firmware=" << control->GetFirmwareVersion()
                      << " gain=" << control->GetGainModeState()
                      << " FFC mode=" << control->GetFlatFieldCorrectionMode() << std::endl;
            double interval = 0, threshold = 0;
            if (control->GetFlatFieldCorrectionParameters(interval, threshold))
                std::cout << "[INFO] Device FFC interval=" << interval
                          << " trigger threshold=" << threshold << std::endl;
            else std::cout << "[WARN] Device FFC parameters unavailable" << std::endl;
        } catch (const std::exception& error) {
            std::cout << "[WARN] Sensor status query failed: " << error.what() << std::endl;
        }
    }

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
