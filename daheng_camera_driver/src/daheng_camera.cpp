#include "daheng_camera_driver/daheng_camera.hpp"

DahengCamera::DahengCamera(rclcpp::Node* node) : node_(node) {
    GXInitLib(); // 初始化SDK库[2](@ref)
}

DahengCamera::~DahengCamera() {
    close();
    GXCloseLib();
}

bool DahengCamera::open() {
    if(is_opened_) return true;
    
    uint32_t device_num = 0;
    GXUpdateDeviceList(&device_num, 1000);
    if(device_num < 1) {
        RCLCPP_ERROR(node_->get_logger(), "未检测到大恒相机设备");
        return false;
    }

    GX_OPEN_PARAM open_param;
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = "1"; // 打开第一个相机
    
    GX_STATUS status = GXOpenDevice(&open_param, &device_handle_);
    if(status != GX_STATUS_SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "相机打开失败: 0x%x", status);
        return false;
    }
    
    // 设置采集模式为连续采集
    GXSetEnum(device_handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    
    is_opened_ = true;
    return true;
}

void DahengCamera::close() {
    if(is_streaming_) stop_acquisition();
    if(is_opened_) {
        GXCloseDevice(device_handle_);
        device_handle_ = nullptr;
        is_opened_ = false;
    }
}

bool DahengCamera::start_acquisition() {
    if(!is_opened_) return false;
    
    GX_STATUS status = GXStreamOn(device_handle_);
    if(status != GX_STATUS_SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "开始采集失败: 0x%x", status);
        return false;
    }
    is_streaming_ = true;
    return true;
}

bool DahengCamera::stop_acquisition() {
    if(!is_streaming_) return true;
    
    GX_STATUS status = GXStreamOff(device_handle_);
    if(status != GX_STATUS_SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "停止采集失败: 0x%x", status);
        return false;
    }
    is_streaming_ = false;
    return true;
}

bool DahengCamera::capture(cv::Mat& image) {
    if(!is_streaming_) return false;
    
    GX_FRAME_DATA frame_data;
    frame_data.nPixelSize = 0;
    frame_data.pImgBuf = nullptr;
    
    GX_STATUS status = GXGetImage(device_handle_, &frame_data, 1000);
    if(status != GX_STATUS_SUCCESS) {
        RCLCPP_WARN(node_->get_logger(), "获取图像失败: 0x%x", status);
        return false;
    }
    
    // 转换图像格式为OpenCV Mat[1,7](@ref)
    if(frame_data.nPixelFormat == GX_PIXEL_FORMAT_BAYER_GR8) {
        cv::Mat raw(frame_data.nHeight, frame_data.nWidth, CV_8UC1, frame_data.pImgBuf);
        cv::cvtColor(raw, image, cv::COLOR_BayerGR2RGB);
    } 
    else if(frame_data.nPixelFormat == GX_PIXEL_FORMAT_MONO8) {
        image = cv::Mat(frame_data.nHeight, frame_data.nWidth, CV_8UC1, frame_data.pImgBuf);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "不支持的图像格式: %d", frame_data.nPixelFormat);
        return false;
    }
    return true;
}

bool DahengCamera::set_exposure(double exposure_us) {
    if(!is_opened_) return false;
    return GXSetFloat(device_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_us) == GX_STATUS_SUCCESS;
}

bool DahengCamera::set_gain(double gain_db) {
    if(!is_opened_) return false;
    return GXSetFloat(device_handle_, GX_FLOAT_GAIN, gain_db) == GX_STATUS_SUCCESS;
}