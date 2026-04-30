#include "daheng.hpp"

#include <libusb-1.0/libusb.h>
#include <unordered_map>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
Daheng::Daheng(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("Daheng's daemon thread stopped.");
  }};
}

Daheng::~Daheng()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("Daheng destructed.");
}

void Daheng::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void Daheng::set_exposure(double exposure_ms)
{
  exposure_us_ = exposure_ms * 1e3;  // 转换为微秒
  if (handle_) {
    // 停止当前采集以安全地改变曝光设置
    bool was_capturing = capturing_;
    if (was_capturing) {
      tools::logger()->info("[Daheng] Temporarily stopping capture to change exposure...");
      capture_stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 设置新的曝光值
    GX_STATUS status = GXSetFloatValue(handle_, "ExposureTime", exposure_us_);
    if (status == GX_STATUS_SUCCESS) {
      tools::logger()->info("[Daheng] Exposure set to {:.1f} ms ({:.0f} us) - SUCCESS", exposure_ms, exposure_us_);

      // 验证设置是否生效
      GX_FLOAT_VALUE exposure_value;
      GX_STATUS read_status = GXGetFloatValue(handle_, "ExposureTime", &exposure_value);
      if (read_status == GX_STATUS_SUCCESS) {
        tools::logger()->info("[Daheng] Exposure verification: set={:.0f} us, read={:.0f} us (range: {:.0f}-{:.0f} us)",
                             exposure_us_, exposure_value.dCurValue,
                             exposure_value.dMin, exposure_value.dMax);

        // 检查设置值是否在有效范围内
        if (exposure_us_ < exposure_value.dMin || exposure_us_ > exposure_value.dMax) {
          tools::logger()->warn("[Daheng] Exposure value {:.0f} us is out of range [{:.0f}, {:.0f}] us",
                               exposure_us_, exposure_value.dMin, exposure_value.dMax);
        }
      } else {
        tools::logger()->error("[Daheng] Failed to read exposure value! Status: {:#x}", read_status);
      }

      // 重新启动采集
      if (was_capturing) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        capture_start();
        tools::logger()->info("[Daheng] Capture restarted with new exposure settings");
      }

    } else {
      tools::logger()->error("[Daheng] Failed to set exposure! Status: {:#x}", status);

      // 如果设置失败，尝试重新启动采集
      if (was_capturing) {
        capture_start();
      }
    }
  } else {
    tools::logger()->warn("[Daheng] Camera handle not available, cannot set exposure");
  }
}

void Daheng::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  GX_STATUS status;

  // Initialize library
  status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXInitLib failed: {:#x}", status);
    return;
  }

  // Update device list
  uint32_t device_num = 0;
  status = GXUpdateAllDeviceList(&device_num, 1000);
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXUpdateAllDeviceList failed: {:#x}", status);
    GXCloseLib();
    return;
  }

  if (device_num == 0) {
    tools::logger()->warn("Not found camera!");
    GXCloseLib();
    return;
  }

  // Open device by index
  GX_OPEN_PARAM open_param;
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;
  open_param.openMode = GX_OPEN_INDEX;
  open_param.pszContent = "1";
  
  status = GXOpenDevice(&open_param, &handle_);
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXOpenDevice failed: {:#x}", status);
    GXCloseLib();
    return;
  }

  // Set camera parameters
  set_enum_value("BalanceWhiteAuto", 2);  // Continuous
  set_enum_value("ExposureAuto", 0);      // Off
  set_enum_value("GainAuto", 0);          // Off

  // 检查曝光范围
  GX_FLOAT_VALUE exposure_range;
  GX_STATUS range_status = GXGetFloatValue(handle_, "ExposureTime", &exposure_range);
  if (range_status == GX_STATUS_SUCCESS) {
    tools::logger()->info("[Daheng] Camera exposure range: {:.0f}-{:.0f} us, step: {:.0f} us",
                         exposure_range.dMin, exposure_range.dMax, exposure_range.dInc);
  }

  // 设置曝光前检查范围并调整
  GX_FLOAT_VALUE exposure_check;
  GX_STATUS check_status = GXGetFloatValue(handle_, "ExposureTime", &exposure_check);
  if (check_status == GX_STATUS_SUCCESS) {
    if (exposure_us_ < exposure_check.dMin) {
      exposure_us_ = exposure_check.dMin;
      tools::logger()->warn("[Daheng] Exposure {:.0f} us too low, adjusted to min {:.0f} us",
                           exposure_us_ * 1000, exposure_check.dMin);
    } else if (exposure_us_ > exposure_check.dMax) {
      exposure_us_ = exposure_check.dMax;
      tools::logger()->warn("[Daheng] Exposure {:.0f} us too high, adjusted to max {:.0f} us",
                           exposure_us_ * 1000, exposure_check.dMax);
    }
  }

  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);

  // Start acquisition
  status = GXStreamOn(handle_);
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXStreamOn failed: {:#x}", status);
    GXCloseDevice(handle_);
    GXCloseLib();
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng's capture thread started.");

    capturing_ = true;

    GX_FRAME_DATA frame_data;
    frame_data.pImgBuf = nullptr;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      GX_STATUS status;
      uint32_t timeout_ms = 100;

      // Allocate buffer if needed
      if (frame_data.pImgBuf == nullptr) {
        // Get payload size
        GX_INT_VALUE payload_size;
        status = GXGetIntValue(handle_, "PayloadSize", &payload_size);
        if (status == GX_STATUS_SUCCESS) {
          frame_data.pImgBuf = malloc(payload_size.nCurValue);
          frame_data.nImgSize = payload_size.nCurValue;
        } else {
          tools::logger()->warn("Failed to get PayloadSize: {:#x}", status);
          break;
        }
      }

      status = GXGetImage(handle_, &frame_data, timeout_ms);
      if (status != GX_STATUS_SUCCESS) {
        if (status != GX_STATUS_TIMEOUT) {
          tools::logger()->warn("GXGetImage failed: {:#x}", status);
          break;
        }
        continue;
      }

      auto timestamp = std::chrono::steady_clock::now();
      
      // Get frame info
      auto width = frame_data.nWidth;
      auto height = frame_data.nHeight;
      auto pixel_format = frame_data.nPixelFormat;
      
      cv::Mat img(cv::Size(width, height), CV_8U, frame_data.pImgBuf);
      
      // Convert pixel format if needed
      cv::Mat dst_image;
      const static std::unordered_map<int32_t, cv::ColorConversionCodes> type_map = {
        {GX_PIXEL_FORMAT_BAYER_GR8, cv::COLOR_BayerGR2RGB},
        {GX_PIXEL_FORMAT_BAYER_RG8, cv::COLOR_BayerRG2RGB},
        {GX_PIXEL_FORMAT_BAYER_GB8, cv::COLOR_BayerGB2RGB},
        {GX_PIXEL_FORMAT_BAYER_BG8, cv::COLOR_BayerBG2RGB}};
      
      if (type_map.find(pixel_format) != type_map.end()) {
        cv::cvtColor(img, dst_image, type_map.at(pixel_format));
        img = dst_image;
      }

      queue_.push({img, timestamp});
    }

    // Free allocated buffer
    if (frame_data.pImgBuf != nullptr) {
      free(frame_data.pImgBuf);
    }

    capturing_ = false;
    tools::logger()->info("Daheng's capture thread stopped.");
  }};
}

void Daheng::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  GX_STATUS status;

  status = GXStreamOff(handle_);
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXStreamOff failed: {:#x}", status);
  }

  status = GXCloseDevice(handle_);
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXCloseDevice failed: {:#x}", status);
  }

  status = GXCloseLib();
  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXCloseLib failed: {:#x}", status);
  }
}

void Daheng::set_float_value(const std::string & name, double value)
{
  GX_STATUS status = GXSetFloatValue(handle_, name.c_str(), value);

  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXSetFloatValue(\"{}\", {}) failed: {:#x}", name, value, status);
  }
}

void Daheng::set_enum_value(const std::string & name, int64_t value)
{
  GX_STATUS status = GXSetEnumValue(handle_, name.c_str(), value);

  if (status != GX_STATUS_SUCCESS) {
    tools::logger()->warn("GXSetEnumValue(\"{}\", {}) failed: {:#x}", name, value, status);
  }
}

void Daheng::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void Daheng::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
