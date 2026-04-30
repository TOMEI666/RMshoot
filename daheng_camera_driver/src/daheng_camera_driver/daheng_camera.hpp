#pragma once
#include <GxIAPI.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class DahengCamera {
public:
    DahengCamera(rclcpp::Node* node);
    ~DahengCamera();
    
    bool open();
    void close();
    bool start_acquisition();
    bool stop_acquisition();
    bool capture(cv::Mat& image);
    bool set_exposure(double exposure_us);
    bool set_gain(double gain_db);
    
private:
    rclcpp::Node* node_;
    GX_DEV_HANDLE device_handle_ = nullptr;
    bool is_opened_ = false;
    bool is_streaming_ = false;
};