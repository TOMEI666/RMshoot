#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help           |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/sentry.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto video_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;
  tools::Plotter plotter;

  tools::logger()->info("video_path: {}", video_path);
  tools::logger()->info("config_path: {}", config_path);
  tools::logger()->info("start_index: {}, end_index: {}, tradition: {}",
                        start_index, end_index, use_tradition);

  // 打开视频（失败则尝试使用内置demo视频）
  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->warn("无法打开视频: {}，尝试使用内置demo视频", video_path);
    video.open("assets/demo/demo.avi");
    if (!video.isOpened()) {
      tools::logger()->error("无法打开任何视频源，请检查路径或编解码器");
      return -1;
    }
  }

  // 构造检测器，异常保护避免程序直接崩溃
  std::unique_ptr<auto_aim::Detector> detector_ptr;
  std::unique_ptr<auto_aim::YOLO> yolo_ptr;
  try {
    detector_ptr = std::make_unique<auto_aim::Detector>(config_path);
  } catch (const std::exception &e) {
    tools::logger()->error("构造 Detector 失败: {}", e.what());
  }
  try {
    yolo_ptr = std::make_unique<auto_aim::YOLO>(config_path);
  } catch (const std::exception &e) {
    tools::logger()->error("构造 YOLO 失败: {}", e.what());
  }
  if (!detector_ptr && !yolo_ptr) {
    tools::logger()->error("Detector 与 YOLO 均初始化失败，退出");
    return -1;
  }

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    std::list<auto_aim::Armor> armors;
    video.read(img);
    if (img.empty()) break;
    // cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);

    if (use_tradition) {
      if (detector_ptr) armors = detector_ptr->detect(img, frame_count);
    } else {
      if (yolo_ptr) armors = yolo_ptr->detect(img, frame_count);
    }

    if (!armors.empty()) {
      nlohmann::json data;
      auto armor = armors.front();

      data["armor_0_pixel_x"] = armor.points[0].x;
      data["armor_0_pixel_y"] = armor.points[0].y;
      data["armor_1_pixel_x"] = armor.points[1].x;
      data["armor_1_pixel_y"] = armor.points[1].y;
      data["armor_2_pixel_x"] = armor.points[2].x;
      data["armor_2_pixel_y"] = armor.points[2].y;
      data["armor_3_pixel_x"] = armor.points[3].x;
      data["armor_3_pixel_y"] = armor.points[3].y;
      plotter.plot(data);
      // 绘制首个装甲板的四点轮廓，直观可视化
      for (int i = 0; i < 4; ++i) {
        cv::line(img, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }
    }

    // 始终显示当前帧，提供可视化窗口
    cv::imshow("detector_video_test", img);
    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}