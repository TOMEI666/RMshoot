#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path, io::CommunicationMode::CAN);  // 使用CAN通信
  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  cv::Mat debug_img;
  bool show_detection = true;
  bool show_tracking = true;
  bool show_prediction = true;
  bool show_info = true;
  double scale_factor = 0.7;

  // 曝光控制变量
  double current_exposure = 100.0;    // 默认100ms，在相机范围内
  double exposure_step = 50.0;        // 每次调整50ms，更精细的控制

  // 预测可视化变量
  bool show_prediction_window = false;
  const int PREDICTION_HISTORY_SIZE = 200;  // 存储最近200帧的预测数据

  // 预测历史数据存储
  struct PredictionData {
    double time;
    Eigen::VectorXd state;  // 11维状态向量
    Eigen::Vector4d aim_xyza;  // 瞄准点坐标
    bool valid = false;
  };
  std::deque<PredictionData> prediction_history;

  // 预测可视化绘制函数
  void draw_prediction_window(const std::deque<PredictionData> & history)
  {
    if (history.empty()) return;

    const int WINDOW_WIDTH = 1200;
    const int WINDOW_HEIGHT = 800;
    const int MARGIN = 60;
    const int PLOT_WIDTH = (WINDOW_WIDTH - 3 * MARGIN) / 2;
    const int PLOT_HEIGHT = (WINDOW_HEIGHT - 3 * MARGIN) / 2;

    cv::Mat prediction_img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(30, 30, 30));

    // 找到数据范围
    double time_min = history.front().time;
    double time_max = history.back().time;
    double x_min = 1e9, x_max = -1e9;
    double y_min = 1e9, y_max = -1e9;
    double z_min = 1e9, z_max = -1e9;
    double vx_min = 1e9, vx_max = -1e9;
    double vy_min = 1e9, vy_max = -1e9;
    double vz_min = 1e9, vz_max = -1e9;
    double w_min = 1e9, w_max = -1e9;

    for (const auto & data : history) {
      if (!data.valid) continue;
      const auto & state = data.state;
      x_min = std::min(x_min, state[0]); x_max = std::max(x_max, state[0]);
      y_min = std::min(y_min, state[2]); y_max = std::max(y_max, state[2]);
      z_min = std::min(z_min, state[4]); z_max = std::max(z_max, state[4]);
      vx_min = std::min(vx_min, state[1]); vx_max = std::max(vx_max, state[1]);
      vy_min = std::min(vy_min, state[3]); vy_max = std::max(vy_max, state[3]);
      vz_min = std::min(vz_min, state[5]); vz_max = std::max(vz_max, state[5]);
      w_min = std::min(w_min, state[7]); w_max = std::max(w_max, state[7]);
    }

    // 添加一些边距
    double x_range = x_max - x_min; if (x_range < 0.1) x_range = 0.1;
    double y_range = y_max - y_min; if (y_range < 0.1) y_range = 0.1;
    double z_range = z_max - z_min; if (z_range < 0.1) z_range = 0.1;
    double vx_range = vx_max - vx_min; if (vx_range < 0.1) vx_range = 0.1;
    double vy_range = vy_max - vy_min; if (vy_range < 0.1) vy_range = 0.1;
    double vz_range = vz_max - vz_min; if (vz_range < 0.1) vz_range = 0.1;
    double w_range = w_max - w_min; if (w_range < 0.1) w_range = 0.1;

    x_min -= x_range * 0.1; x_max += x_range * 0.1;
    y_min -= y_range * 0.1; y_max += y_range * 0.1;
    z_min -= z_range * 0.1; z_max += z_range * 0.1;
    vx_min -= vx_range * 0.1; vx_max += vx_range * 0.1;
    vy_min -= vy_range * 0.1; vy_max += vy_range * 0.1;
    vz_min -= vz_range * 0.1; vz_max += vz_range * 0.1;
    w_min -= w_range * 0.1; w_max += w_range * 0.1;

    // 绘制四个子图的边框和标题
    auto draw_plot_area = [&](int x, int y, int w, int h, const std::string & title) {
      cv::rectangle(prediction_img, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(100, 100, 100), 1);
      cv::putText(prediction_img, title, cv::Point(x + 10, y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    };

    // 左上：位置轨迹 (X-Y)
    draw_plot_area(MARGIN, MARGIN, PLOT_WIDTH, PLOT_HEIGHT, "Position Trajectory (X-Y)");
    // 右上：高度和Z速度
    draw_plot_area(MARGIN * 2 + PLOT_WIDTH, MARGIN, PLOT_WIDTH, PLOT_HEIGHT, "Height & Z-Velocity");
    // 左下：XY速度
    draw_plot_area(MARGIN, MARGIN * 2 + PLOT_HEIGHT, PLOT_WIDTH, PLOT_HEIGHT, "Velocity (X-Y)");
    // 右下：角速度和瞄准点
    draw_plot_area(MARGIN * 2 + PLOT_WIDTH, MARGIN * 2 + PLOT_HEIGHT, PLOT_WIDTH, PLOT_HEIGHT, "Angular Velocity & Aim");

    // 坐标转换函数
    auto time_to_pixel = [&](double t) { return MARGIN + (t - time_min) / (time_max - time_min) * PLOT_WIDTH; };
    auto value_to_pixel_y = [&](double v, double min_v, double max_v, int plot_y) {
      return plot_y + PLOT_HEIGHT - MARGIN - (v - min_v) / (max_v - min_v) * (PLOT_HEIGHT - 2 * MARGIN);
    };

    // 绘制网格和坐标轴标签
    auto draw_grid = [&](int plot_x, int plot_y, double min_val, double max_val, bool is_time_x = true) {
      // 垂直网格线
      for (int i = 0; i <= 5; ++i) {
        int x = plot_x + MARGIN + i * (PLOT_WIDTH - 2 * MARGIN) / 5;
        cv::line(prediction_img, cv::Point(x, plot_y + MARGIN), cv::Point(x, plot_y + PLOT_HEIGHT - MARGIN), cv::Scalar(60, 60, 60), 1);
      }
      // 水平网格线
      for (int i = 0; i <= 4; ++i) {
        int y = plot_y + MARGIN + i * (PLOT_HEIGHT - 2 * MARGIN) / 4;
        cv::line(prediction_img, cv::Point(plot_x + MARGIN, y), cv::Point(plot_x + PLOT_WIDTH - MARGIN, y), cv::Scalar(60, 60, 60), 1);
      }
    };

    draw_grid(MARGIN, MARGIN, 0, 0);  // 位置轨迹
    draw_grid(MARGIN * 2 + PLOT_WIDTH, MARGIN, 0, 0);  // 高度
    draw_grid(MARGIN, MARGIN * 2 + PLOT_HEIGHT, 0, 0);  // XY速度
    draw_grid(MARGIN * 2 + PLOT_WIDTH, MARGIN * 2 + PLOT_HEIGHT, 0, 0);  // 角速度

    // 绘制数据曲线
    for (size_t i = 1; i < history.size(); ++i) {
      const auto & prev = history[i-1];
      const auto & curr = history[i];
      if (!prev.valid || !curr.valid) continue;

      double prev_t = prev.time;
      double curr_t = curr.time;
      const auto & prev_state = prev.state;
      const auto & curr_state = curr.state;

      // 位置轨迹 (X-Y) - 左上
      {
        int plot_x = MARGIN, plot_y = MARGIN;
        cv::Point prev_pt(value_to_pixel_y(prev_state[0], x_min, x_max, plot_y),
                         value_to_pixel_y(prev_state[2], y_min, y_max, plot_y));
        cv::Point curr_pt(value_to_pixel_y(curr_state[0], x_min, x_max, plot_y),
                         value_to_pixel_y(curr_state[2], y_min, y_max, plot_y));
        cv::line(prediction_img, prev_pt, curr_pt, cv::Scalar(0, 255, 0), 2);
      }

      // 高度和Z速度 - 右上
      {
        int plot_x = MARGIN * 2 + PLOT_WIDTH, plot_y = MARGIN;
        // Z位置 (绿色)
        cv::Point prev_z(value_to_pixel_y(prev_t, time_min, time_max, plot_y),
                        value_to_pixel_y(prev_state[4], z_min, z_max, plot_y));
        cv::Point curr_z(value_to_pixel_y(curr_t, time_min, time_max, plot_y),
                        value_to_pixel_y(curr_state[4], z_min, z_max, plot_y));
        cv::line(prediction_img, prev_z, curr_z, cv::Scalar(0, 255, 0), 2);
        // Z速度 (红色)
        cv::Point prev_vz(value_to_pixel_y(prev_t, time_min, time_max, plot_y),
                         value_to_pixel_y(prev_state[5], vz_min, vz_max, plot_y));
        cv::Point curr_vz(value_to_pixel_y(curr_t, time_min, time_max, plot_y),
                         value_to_pixel_y(curr_state[5], vz_min, vz_max, plot_y));
        cv::line(prediction_img, prev_vz, curr_vz, cv::Scalar(0, 0, 255), 1);
      }

      // XY速度 - 左下
      {
        int plot_x = MARGIN, plot_y = MARGIN * 2 + PLOT_HEIGHT;
        // X速度 (蓝色)
        cv::Point prev_vx(value_to_pixel_y(prev_t, time_min, time_max, plot_y),
                         value_to_pixel_y(prev_state[1], vx_min, vx_max, plot_y));
        cv::Point curr_vx(value_to_pixel_y(curr_t, time_min, time_max, plot_y),
                         value_to_pixel_y(curr_state[1], vx_min, vx_max, plot_y));
        cv::line(prediction_img, prev_vx, curr_vx, cv::Scalar(255, 0, 0), 2);
        // Y速度 (黄色)
        cv::Point prev_vy(value_to_pixel_y(prev_t, time_min, time_max, plot_y),
                         value_to_pixel_y(prev_state[3], vy_min, vy_max, plot_y));
        cv::Point curr_vy(value_to_pixel_y(curr_t, time_min, time_max, plot_y),
                         value_to_pixel_y(curr_state[3], vy_min, vy_max, plot_y));
        cv::line(prediction_img, prev_vy, curr_vy, cv::Scalar(0, 255, 255), 2);
      }

      // 角速度 - 右下
      {
        int plot_x = MARGIN * 2 + PLOT_WIDTH, plot_y = MARGIN * 2 + PLOT_HEIGHT;
        cv::Point prev_w(value_to_pixel_y(prev_t, time_min, time_max, plot_y),
                        value_to_pixel_y(prev_state[7], w_min, w_max, plot_y));
        cv::Point curr_w(value_to_pixel_y(curr_t, time_min, time_max, plot_y),
                        value_to_pixel_y(curr_state[7], w_min, w_max, plot_y));
        cv::line(prediction_img, prev_w, curr_w, cv::Scalar(255, 0, 255), 2);
      }
    }

    // 绘制当前状态信息
    if (!history.empty() && history.back().valid) {
      const auto & current = history.back();
      const auto & state = current.state;

      cv::putText(prediction_img, fmt::format("Current State:"),
                 cv::Point(10, WINDOW_HEIGHT - 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
      cv::putText(prediction_img, fmt::format("X: {:.2f} Y: {:.2f} Z: {:.2f}", state[0], state[2], state[4]),
                 cv::Point(10, WINDOW_HEIGHT - 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      cv::putText(prediction_img, fmt::format("VX: {:.2f} VY: {:.2f} VZ: {:.2f}", state[1], state[3], state[5]),
                 cv::Point(10, WINDOW_HEIGHT - 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
      cv::putText(prediction_img, fmt::format("Angle: {:.2f} W: {:.2f}", state[6] * 180 / M_PI, state[7]),
                 cv::Point(10, WINDOW_HEIGHT - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
      cv::putText(prediction_img, fmt::format("Radius: {:.2f}", state[8]),
                 cv::Point(10, WINDOW_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    }

    // 绘制图例
    cv::putText(prediction_img, "Legend:", cv::Point(WINDOW_WIDTH - 200, WINDOW_HEIGHT - 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::line(prediction_img, cv::Point(WINDOW_WIDTH - 180, WINDOW_HEIGHT - 85), cv::Point(WINDOW_WIDTH - 160, WINDOW_HEIGHT - 85), cv::Scalar(0, 255, 0), 2);
    cv::putText(prediction_img, "Position", cv::Point(WINDOW_WIDTH - 150, WINDOW_HEIGHT - 80), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    cv::line(prediction_img, cv::Point(WINDOW_WIDTH - 180, WINDOW_HEIGHT - 65), cv::Point(WINDOW_WIDTH - 160, WINDOW_HEIGHT - 65), cv::Scalar(255, 0, 0), 2);
    cv::putText(prediction_img, "X-Velocity", cv::Point(WINDOW_WIDTH - 150, WINDOW_HEIGHT - 60), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    cv::line(prediction_img, cv::Point(WINDOW_WIDTH - 180, WINDOW_HEIGHT - 45), cv::Point(WINDOW_WIDTH - 160, WINDOW_HEIGHT - 45), cv::Scalar(0, 255, 255), 2);
    cv::putText(prediction_img, "Y-Velocity", cv::Point(WINDOW_WIDTH - 150, WINDOW_HEIGHT - 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    cv::line(prediction_img, cv::Point(WINDOW_WIDTH - 180, WINDOW_HEIGHT - 25), cv::Point(WINDOW_WIDTH - 160, WINDOW_HEIGHT - 25), cv::Scalar(255, 0, 255), 2);
    cv::putText(prediction_img, "Angular Vel", cv::Point(WINDOW_WIDTH - 150, WINDOW_HEIGHT - 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);

    cv::imshow("Prediction Visualization", prediction_img);
  }

  while (!exiter.exit() && !quit) {
    try {
    camera.read(img, t);
    } catch (const std::exception & e) {
      // 如果相机读取失败，创建一个测试图像
      img = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(50, 50, 50));
      cv::putText(img, "No Camera - Test Mode", cv::Point(400, 360),
                  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
      cv::putText(img, "Press 'Q' to quit", cv::Point(450, 420),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2);
      cv::putText(img, fmt::format("Camera Error: {}", e.what()), cv::Point(10, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      t = std::chrono::steady_clock::now();
    }
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 收集预测数据用于可视化
    {
      PredictionData data;
      data.time = std::chrono::duration<double>(t.time_since_epoch()).count();

    if (!targets.empty()) {
        const auto & target = targets.front();
        data.state = target.ekf_x();
        data.aim_xyza = planner.debug_xyza;
        data.valid = true;
      } else {
        data.valid = false;
      }

      prediction_history.push_back(data);
      if (prediction_history.size() > PREDICTION_HISTORY_SIZE) {
        prediction_history.pop_front();
      }
    }

    // 创建调试图像副本
    debug_img = img.clone();

    // 1. 显示YOLO检测结果
    if (show_detection) {
      for (const auto & armor : armors) {
        // 绘制装甲板检测框
        cv::rectangle(debug_img, armor.box, cv::Scalar(255, 0, 0), 2);

        // 显示装甲板信息
        std::string armor_info = fmt::format("{:.1f}", armor.confidence);
        cv::putText(debug_img, armor_info,
                   cv::Point(armor.box.x, armor.box.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

        // 显示装甲板类型和颜色
        std::string type_str = armor.type == auto_aim::ArmorType::big ? "BIG" : "SMALL";
        cv::putText(debug_img, type_str,
                   cv::Point(armor.box.x, armor.box.y + armor.box.height + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
      }
    }

    // 2. 显示跟踪和预测结果
    if (!targets.empty() && show_tracking) {
      auto target = targets.front();

      // 绘制所有装甲板位置
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (size_t i = 0; i < armor_xyza_list.size(); ++i) {
        const auto & xyza = armor_xyza_list[i];
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);

        // 绘制装甲板轮廓
        cv::Scalar color = (i == target.last_id) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
        tools::draw_points(debug_img, image_points, color, 2);

        // 显示装甲板ID
        if (!image_points.empty()) {
          cv::Point center(0, 0);
          for (const auto & pt : image_points) center += cv::Point(pt);
          center.x /= image_points.size();
          center.y /= image_points.size();

          cv::putText(debug_img, std::to_string(i),
                     center - cv::Point(5, -5),
                     cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        }
      }

      // 3. 显示瞄准点
      if (show_prediction) {
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
        if (aim_xyza.norm() > 0) {  // 检查是否有效
          auto aim_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, aim_points, cv::Scalar(0, 0, 255), 3);  // 红色瞄准点

          // 在瞄准点画十字
          if (!aim_points.empty()) {
            cv::Point center(0, 0);
            for (const auto & pt : aim_points) center += cv::Point(pt);
            center.x /= aim_points.size();
            center.y /= aim_points.size();

            cv::line(debug_img, center - cv::Point(10, 0), center + cv::Point(10, 0), cv::Scalar(0, 0, 255), 2);
            cv::line(debug_img, center - cv::Point(0, 10), center + cv::Point(0, 10), cv::Scalar(0, 0, 255), 2);
          }
        }
      }
    }

    // 4. 显示调试信息
    if (show_info) {
      int line_height = 25;
      int y_pos = 30;

      // 云台状态
      Eigen::Vector3d gimbal_euler = tools::eulers(q, 2, 1, 0) * 57.3;
      cv::putText(debug_img, fmt::format("Gimbal: Y{:.1f} P{:.1f} R{:.1f}",
                                        gimbal_euler[0], gimbal_euler[1], gimbal_euler[2]),
                 cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
      y_pos += line_height;

      // 目标数量
      cv::putText(debug_img, fmt::format("Targets: {}", targets.size()),
                 cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      y_pos += line_height;

      // 装甲板数量
      cv::putText(debug_img, fmt::format("Armors: {}", armors.size()),
                 cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      y_pos += line_height;

      // 子弹速度
      cv::putText(debug_img, fmt::format("Bullet: {:.1f} m/s", cboard.bullet_speed),
                 cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
      y_pos += line_height;

      // 显示模式
      cv::putText(debug_img, fmt::format("Mode: {}", io::MODES[cboard.mode]),
                 cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
      y_pos += line_height;

      // 曝光设置
      cv::putText(debug_img, fmt::format("Exposure: {:.1f} ms", current_exposure),
                 cv::Point(10, y_pos), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
      y_pos += line_height;

      // 控制帮助
      cv::putText(debug_img, "Controls: [D]etect [T]rack [P]redict [I]nfo [V]isual [E]xpos+ [R]educe- [W]ide+ [S]ave [Q]uit (Exp:20-1000ms)",
                 cv::Point(10, debug_img.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
    }

    // 缩放图像
    cv::resize(debug_img, debug_img, {}, scale_factor, scale_factor);

    // 显示图像
    cv::imshow("RoboMaster Vision Debug", debug_img);

    // 显示预测可视化窗口
    if (show_prediction_window) {
      draw_prediction_window(prediction_history);
    }

    // 键盘控制
    auto key = cv::waitKey(1);
    switch (key) {
      case 'q': case 'Q':
        tools::logger()->info("退出调试程序");
        quit = true;
        break;
      case 'd': case 'D':
        show_detection = !show_detection;
        tools::logger()->info("检测显示: {}", show_detection ? "开启" : "关闭");
        break;
      case 't': case 'T':
        show_tracking = !show_tracking;
        tools::logger()->info("跟踪显示: {}", show_tracking ? "开启" : "关闭");
        break;
      case 'p': case 'P':
        show_prediction = !show_prediction;
        tools::logger()->info("预测显示: {}", show_prediction ? "开启" : "关闭");
        break;
      case 'i': case 'I':
        show_info = !show_info;
        tools::logger()->info("信息显示: {}", show_info ? "开启" : "关闭");
        break;
      case '+': case '=':
        scale_factor = std::min(scale_factor + 0.1, 2.0);
        tools::logger()->info("缩放比例: {:.1f}", scale_factor);
        break;
      case '-': case '_':
        scale_factor = std::max(scale_factor - 0.1, 0.3);
        tools::logger()->info("缩放比例: {:.1f}", scale_factor);
        break;
      case 's': case 'S':
        // 保存当前帧
        cv::imwrite(fmt::format("debug_frame_{}.jpg", std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count()), debug_img);
        tools::logger()->info("保存调试帧");
        break;
      case 'e': case 'E':
        // 增加曝光
        current_exposure = std::min(current_exposure + exposure_step, 1000.0);  // 最大1000ms (相机限制)
        camera.set_exposure(current_exposure);
        tools::logger()->info("曝光增加: {:.1f} ms", current_exposure);
        break;
      case 'r': case 'R':
        // 减少曝光
        current_exposure = std::max(current_exposure - exposure_step, 20.0);  // 最小20us (相机限制)
        camera.set_exposure(current_exposure);
        tools::logger()->info("曝光减少: {:.1f} ms", current_exposure);
        break;
      case 'w': case 'W':
        // 大幅增加曝光 (加速调整)
        current_exposure = std::min(current_exposure + exposure_step * 10, 1000.0);
        camera.set_exposure(current_exposure);
        tools::logger()->info("大幅增加曝光: {:.1f} ms", current_exposure);
        break;
      case 'v': case 'V':
        // 切换预测可视化窗口
        show_prediction_window = !show_prediction_window;
        if (show_prediction_window) {
          tools::logger()->info("预测可视化窗口: 开启");
        } else {
          cv::destroyWindow("Prediction Visualization");
          tools::logger()->info("预测可视化窗口: 关闭");
        }
        break;
    }
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}