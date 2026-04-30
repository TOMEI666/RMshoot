// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under
// Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
// ros2
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "armor_detector/armor_detector_node.hpp"
#include "armor_detector/types.hpp"
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/url_resolver.hpp"

namespace fyt::auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options)
    : Node("armor_detector", options) {
  FYT_REGISTER_LOGGER("armor_detector", "~/fyt2024-log", INFO);
  FYT_INFO("armor_detector", "Starting ArmorDetectorNode!");
  // Detector
  detector_ = initDetector();

  // Armors Publisher
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
      "armor_detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.y = 0.15;
  armor_marker_.scale.z = 0.12;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "armor_detector/marker", 10);

  // Debug Publishers
  debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    createDebugPublishers();
  }

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this,
                std::placeholders::_1));

  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
      "armor_detector/set_mode",
      std::bind(&ArmorDetectorNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  // Detect armors
  auto armors = detectArmors(img_msg);

  // Init message
  armors_msg_.header = img_msg->header;
  armors_msg_.armors.clear();

  // Publishing marker
  if (debug_) {
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;
    armor_marker_.header = text_marker_.header = armors_msg_.header;
    // Fill the markers
    for (const auto &armor : armors_msg_.armors) {
      armor_marker_.pose = armor.pose;
      armor_marker_.id++;
      text_marker_.pose.position = armor.pose.position;
      text_marker_.id++;
      text_marker_.pose.position.y -= 0.1;
      text_marker_.text = armor.number;
      marker_array_.markers.emplace_back(armor_marker_);
      marker_array_.markers.emplace_back(text_marker_);
    }
    publishMarkers();
  }

  // Publishing detected armors
  armors_pub_->publish(armors_msg_);
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  Detector::LightParams l_params = {
      .min_ratio = declare_parameter("light.min_ratio", 0.08),
      .max_ratio = declare_parameter("light.max_ratio", 0.4),
      .max_angle = declare_parameter("light.max_angle", 40.0),
      .color_diff_thresh =
          static_cast<int>(declare_parameter("light.color_diff_thresh", 25))};

  Detector::ArmorParams a_params = {
      .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
      .min_small_center_distance =
          declare_parameter("armor.min_small_center_distance", 0.8),
      .max_small_center_distance =
          declare_parameter("armor.max_small_center_distance", 3.2),
      .min_large_center_distance =
          declare_parameter("armor.min_large_center_distance", 3.2),
      .max_large_center_distance =
          declare_parameter("armor.max_large_center_distance", 5.0),
      .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, EnemyColor::BLUE,
                                             l_params, a_params);

  // Set dynamic parameter callback
  on_set_parameters_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));

  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) {
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  auto armors = detector_->detect(img);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;

  // Publish debug info
  if (debug_) {
    binary_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img)
            .toImageMsg());

    // Sort lights and armors data by x coordinate
    std::sort(detector_->debug_lights.data.begin(),
              detector_->debug_lights.data.end(),
              [](const auto &l1, const auto &l2) {
                return l1.center_x < l2.center_x;
              });
    std::sort(detector_->debug_armors.data.begin(),
              detector_->debug_armors.data.end(),
              [](const auto &a1, const auto &a2) {
                return a1.center_x < a2.center_x;
              });

    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    detector_->drawResults(img);

    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency
               << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

rcl_interfaces::msg::SetParametersResult
ArmorDetectorNode::onSetParameters(std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "binary_thres") {
      detector_->binary_thres = param.as_int();
    } else if (param.get_name() == "light.min_ratio") {
      detector_->light_params.min_ratio = param.as_double();
    } else if (param.get_name() == "light.max_ratio") {
      detector_->light_params.max_ratio = param.as_double();
    } else if (param.get_name() == "light.max_angle") {
      detector_->light_params.max_angle = param.as_double();
    } else if (param.get_name() == "light.color_diff_thresh") {
      detector_->light_params.color_diff_thresh = param.as_int();
    } else if (param.get_name() == "armor.min_light_ratio") {
      detector_->armor_params.min_light_ratio = param.as_double();
    } else if (param.get_name() == "armor.min_small_center_distance") {
      detector_->armor_params.min_small_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_small_center_distance") {
      detector_->armor_params.max_small_center_distance = param.as_double();
    } else if (param.get_name() == "armor.min_large_center_distance") {
      detector_->armor_params.min_large_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_large_center_distance") {
      detector_->armor_params.max_large_center_distance = param.as_double();
    } else if (param.get_name() == "armor.max_angle") {
      detector_->armor_params.max_angle = param.as_double();
    }
  }
  return result;
}

void ArmorDetectorNode::createDebugPublishers() noexcept {
  lights_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugLights>(
      "armor_detector/debug_lights", 10);
  armors_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugArmors>(
      "armor_detector/debug_armors", 10);
  this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
  this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
  binary_img_pub_ =
      image_transport::create_publisher(this, "armor_detector/binary_img");
  result_img_pub_ =
      image_transport::create_publisher(this, "armor_detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers() noexcept {
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers() noexcept {
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action =
      armors_msg_.armors.empty() ? Marker::DELETEALL : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  response->message = "0";
  
  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("armor_detector", "Invalid mode: {}", request->mode);
    return;
  }

  auto createImageSub = [this]() {
    if (img_sub_ == nullptr) {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          "image_raw", rclcpp::SensorDataQoS(),
          std::bind(&ArmorDetectorNode::imageCallback, this,
                    std::placeholders::_1));
    }
  };

  switch (mode) {
  case VisionMode::AUTO_AIM_RED: {
    detector_->detect_color = EnemyColor::RED;
    createImageSub();
    break;
  }
  case VisionMode::AUTO_AIM_BLUE: {
    detector_->detect_color = EnemyColor::BLUE;
    createImageSub();
    break;
  }
  default: {
    img_sub_.reset();
  }
  }

  FYT_WARN("armor_detector", "Set mode to {}", mode_name);
}

} // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorDetectorNode)  // 修改这一行
