#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path, CommunicationMode comm_mode)
: comm_mode_(comm_mode)
{
  auto yaml = tools::load(config_path);

  if (comm_mode_ == CommunicationMode::SERIAL) {
    // 串口通信模式
    auto com_port = tools::read<std::string>(yaml, "com_port");

    try {
      serial_.setPort(com_port);
      serial_.open();
      tools::logger()->info("[Gimbal] Using SERIAL communication on {}", com_port);
    } catch (const std::exception & e) {
      tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
      exit(1);
    }
  } else if (comm_mode_ == CommunicationMode::CAN) {
    // CAN通信模式
    auto can_interface = tools::read<std::string>(yaml, "can_interface");

    try {
      can_ = new SocketCAN(can_interface, [this](const can_frame & frame) {
        this->can_callback(frame);
      });
      tools::logger()->info("[Gimbal] Using CAN communication on {}", can_interface);

      // 从YAML读取CAN ID（如果有的话）
      if (yaml["gimbal_canid"]) {
        gimbal_canid_ = tools::read<int>(yaml, "gimbal_canid");
      }
      tools::logger()->info("[Gimbal] Gimbal CAN ID: 0x{:X}", gimbal_canid_);

    } catch (const std::exception & e) {
      tools::logger()->error("[Gimbal] Failed to initialize CAN: {}", e.what());
      exit(1);
    }
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();

  if (comm_mode_ == CommunicationMode::SERIAL) {
    serial_.close();
  } else if (comm_mode_ == CommunicationMode::CAN) {
    delete can_;
    can_ = nullptr;
  }
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  if (comm_mode_ == CommunicationMode::SERIAL) {
    // 串口发送
    tx_data_.mode = VisionToGimbal.mode;
    tx_data_.yaw = VisionToGimbal.yaw;
    tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
    tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
    tx_data_.pitch = VisionToGimbal.pitch;
    tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
    tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
    tx_data_.crc16 = tools::get_crc16(
      reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

    try {
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  } else if (comm_mode_ == CommunicationMode::CAN) {
    // CAN发送
    can_frame frame;
    frame.can_id = gimbal_canid_;
    frame.can_dlc = 8;

    // 将控制数据打包到CAN帧中
    // 注意：这里需要根据实际的下位机CAN协议进行调整
    uint8_t mode = VisionToGimbal.mode;
    int16_t yaw_raw = static_cast<int16_t>(VisionToGimbal.yaw * 1000);    // 放大1000倍
    int16_t pitch_raw = static_cast<int16_t>(VisionToGimbal.pitch * 1000);

    frame.data[0] = mode;
    frame.data[1] = (yaw_raw >> 8) & 0xFF;
    frame.data[2] = yaw_raw & 0xFF;
    frame.data[3] = (pitch_raw >> 8) & 0xFF;
    frame.data[4] = pitch_raw & 0xFF;
    // 可以继续添加速度和加速度数据
    frame.data[5] = 0; // 预留
    frame.data[6] = 0; // 预留
    frame.data[7] = 0; // 预留

    try {
      can_->write(&frame);
      tools::logger()->debug("[Gimbal] CAN frame sent: ID=0x{:X}, DLC={}, Data=[{:02X},{:02X},{:02X},{:02X},{:02X},{:02X},{:02X},{:02X}]",
                            frame.can_id, frame.can_dlc,
                            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                            frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write CAN: {}", e.what());
    }
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  if (comm_mode_ == CommunicationMode::SERIAL) {
    read_thread_serial();
  } else if (comm_mode_ == CommunicationMode::CAN) {
    read_thread_can();
  }
}

void Gimbal::read_thread_serial()
{
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    process_gimbal_data(rx_data_, t);
  }

  tools::logger()->info("[Gimbal] serial read_thread stopped.");
}

void Gimbal::read_thread_can()
{
  tools::logger()->info("[Gimbal] CAN read_thread started.");

  // CAN通信是事件驱动的，主要处理在can_callback中
  // 这里只需要保持线程运行
  while (!quit_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  tools::logger()->info("[Gimbal] CAN read_thread stopped.");
}

void Gimbal::can_callback(const can_frame & frame)
{
  // 检查是否是云台发送的数据
  if (frame.can_id != gimbal_canid_) {
    return; // 不是云台数据，忽略
  }

  auto t = std::chrono::steady_clock::now();

  // 解析CAN帧数据
  // 注意：这里需要根据实际的下位机CAN协议进行调整
  if (frame.can_dlc >= 8) {
    // 解析云台状态数据
    GimbalToVision can_data;

    // 从CAN帧解析数据
    uint8_t mode = frame.data[0];
    int16_t yaw_raw = (frame.data[1] << 8) | frame.data[2];
    int16_t pitch_raw = (frame.data[3] << 8) | frame.data[4];
    int16_t yaw_vel_raw = (frame.data[5] << 8) | frame.data[6];
    // 可以继续解析更多数据

    // 转换为实际值
    can_data.mode = mode;
    can_data.yaw = yaw_raw / 1000.0f;        // 还原缩放
    can_data.pitch = pitch_raw / 1000.0f;
    can_data.yaw_vel = yaw_vel_raw / 1000.0f;

    // 设置默认值（CAN帧可能不包含所有数据）
    can_data.q[0] = 1.0f; // w
    can_data.q[1] = 0.0f; // x
    can_data.q[2] = 0.0f; // y
    can_data.q[3] = 0.0f; // z
    can_data.bullet_speed = 22.0f; // 默认值
    can_data.bullet_count = 0;

    tools::logger()->debug("[Gimbal] CAN frame received: ID=0x{:X}, Mode={}", frame.can_id, mode);

    process_gimbal_data(can_data, t);
  }
}

void Gimbal::process_gimbal_data(const GimbalToVision & data, std::chrono::steady_clock::time_point t)
{
  Eigen::Quaterniond q(data.q[0], data.q[1], data.q[2], data.q[3]);
  queue_.push({q, t});

  std::lock_guard<std::mutex> lock(mutex_);

  state_.yaw = data.yaw;
  state_.yaw_vel = data.yaw_vel;
  state_.pitch = data.pitch;
  state_.pitch_vel = data.pitch_vel;
  state_.bullet_speed = data.bullet_speed;
  state_.bullet_count = data.bullet_count;

  switch (data.mode) {
    case 0:
      mode_ = GimbalMode::IDLE;
      break;
    case 1:
      mode_ = GimbalMode::AUTO_AIM;
      break;
    case 2:
      mode_ = GimbalMode::SMALL_BUFF;
      break;
    case 3:
      mode_ = GimbalMode::BIG_BUFF;
      break;
    default:
      mode_ = GimbalMode::IDLE;
      tools::logger()->warn("[Gimbal] Invalid mode: {}", data.mode);
      break;
  }
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io