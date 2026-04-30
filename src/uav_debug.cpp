#include <fmt/core.h>

#include <chrono>
#include <deque>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

// 预测数据结构体
struct PredictionData {
  double time;
  Eigen::VectorXd state;  // 11维状态向量
  Eigen::Vector4d aim_xyza;  // 瞄准点坐标
  bool valid = false;
};

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
  draw_plot_area(MARGIN * 2 + PLOT_WIDTH, MARGIN * 2 + PLOT_HEIGHT, PLOT_WIDTH, PLOT_HEIGHT, "Angular Velocity");

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

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  auto t0 = std::chrono::steady_clock::now();

  // 预测可视化变量
  bool show_prediction_window = false;
  const int PREDICTION_HISTORY_SIZE = 200;  // 存储最近200帧的预测数据

  // 预测历史数据存储
  std::deque<PredictionData> prediction_history;

  while (!exiter.exit()) {
    try {
      camera.read(img, t);
    } catch (const std::exception & e) {
      // 如果相机读取失败，创建一个测试图像
      img = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(50, 50, 50));
      cv::putText(img, "No Camera - Test Mode", cv::Point(400, 360),
                  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
      cv::putText(img, "Press 'V' for prediction visualization", cv::Point(350, 420),
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200), 2);
      cv::putText(img, fmt::format("Camera Error: {}", e.what()), cv::Point(10, 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      t = std::chrono::steady_clock::now();
    }
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;
    // recorder.record(img, q, t);
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    /// 自瞄
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    // 收集预测数据用于可视化
    {
      PredictionData data;
      data.time = std::chrono::duration<double>(t.time_since_epoch()).count();

      if (!targets.empty()) {
        const auto & target = targets.front();
        data.state = target.ekf_x();
        data.aim_xyza = aimer.debug_aim_point.xyza;
        data.valid = true;
      } else {
        data.valid = false;
      }

      prediction_history.push_back(data);
      if (prediction_history.size() > PREDICTION_HISTORY_SIZE) {
        prediction_history.pop_front();
      }
    }

    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    cboard.send(command);

    /// debug
    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    // 云台响应情况
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = ypr[1] * 57.3;
    data["bullet_speed"] = cboard.bullet_speed;
    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["cmd_shoot"] = command.shoot;
    }
    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);

    // 显示预测可视化窗口
    if (show_prediction_window) {
      draw_prediction_window(prediction_history);
    }

    auto key = cv::waitKey(1);
    if (key == 'q') break;
    else if (key == 'v' || key == 'V') {
      // 切换预测可视化窗口
      show_prediction_window = !show_prediction_window;
      if (show_prediction_window) {
        tools::logger()->info("预测可视化窗口: 开启");
      } else {
        cv::destroyWindow("Prediction Visualization");
        tools::logger()->info("预测可视化窗口: 关闭");
      }
    }
  }

  return 0;
}