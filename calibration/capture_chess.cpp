#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/chess_images  | 输出文件夹路径   }";

void capture_loop(const std::string & config_path, const std::string & output_folder)
{
  // 读取配置参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  tools::logger()->info("棋盘格标定板尺寸: {}x{} (内角点数)", pattern_cols, pattern_rows);
  tools::logger()->info("提示: 按 's' 保存图像，按 'q' 退出");

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    try {
      camera.read(img, timestamp);
    } catch (const std::exception & e) {
      tools::logger()->error("相机读取失败: {}", e.what());
      break;
    }

    if (img.empty()) {
      tools::logger()->warn("图像为空，跳过");
      continue;
    }

    // 检测棋盘格角点
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(
      img, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    // 如果找到角点，进行亚像素精化
    if (found) {
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    }

    // 在图像上绘制检测结果
    auto img_with_corners = img.clone();
    cv::drawChessboardCorners(img_with_corners, pattern_size, corners, found);

    // 显示状态信息
    std::string status_text = found ? "检测成功 - 按 's' 保存" : "未检测到棋盘格";
    cv::Scalar status_color = found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    tools::draw_text(img_with_corners, status_text, {40, 40}, status_color);
    tools::draw_text(img_with_corners, fmt::format("已保存: {} 张", count), {40, 80}, {255, 255, 255});
    tools::draw_text(img_with_corners, "按 'q' 退出", {40, 120}, {255, 255, 255});

    // 缩小图像以便显示
    cv::Mat img_display;
    cv::resize(img_with_corners, img_display, {}, 0.5, 0.5);

    // 显示图像
    cv::imshow("棋盘格标定 - Press 's' to save, 'q' to quit", img_display);
    auto key = cv::waitKey(1) & 0xFF;

    if (key == 'q' || key == 27) {  // 'q' 或 ESC 退出
      break;
    } else if (key == 's' && found) {  // 只有检测成功时才保存
      count++;
      auto img_path = fmt::format("{}/{:04d}.jpg", output_folder, count);
      cv::imwrite(img_path, img);
      tools::logger()->info("[{}] 已保存: {}", count, img_path);
    } else if (key == 's' && !found) {
      tools::logger()->warn("未检测到棋盘格，无法保存");
    }
  }

  tools::logger()->info("共保存 {} 张图像到 {}", count, output_folder);
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);
  tools::logger()->info("输出文件夹: {}", output_folder);

  // 主循环，保存棋盘格图像
  capture_loop(config_path, output_folder);

  return 0;
}


