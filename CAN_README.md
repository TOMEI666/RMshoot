# USB转CAN快速开始指南

## 🚀 快速设置

### 1. 运行自动设置脚本
```bash
sudo ./setup_can.sh
```

此脚本将自动：
- 安装必要的CAN工具
- 配置udev规则
- 设置CAN接口
- 创建测试脚本

### 2. 硬件连接
```
电脑USB端口 ── USB转CAN设备 ── CAN线 ── RoboMaster C板
```

### 3. 运行测试
```bash
sudo test_can.sh
```
选择选项4测试与RoboMaster C板的通信

### 4. 运行视觉系统
```bash
./build/auto_aim_debug_mpc configs/sentry.yaml
```

## 📋 配置文件

确保 `configs/sentry.yaml` 中的CAN配置正确：

```yaml
# CAN通信配置
quaternion_canid: 0x01      # IMU四元数ID
bullet_speed_canid: 0x110   # 子弹速度ID
send_canid: 0xff            # 发送指令ID
can_interface: "can0"       # CAN接口名
```

## 🔧 手动配置（可选）

如果自动脚本不工作，可以手动配置：

```bash
# 1. 安装工具
sudo apt install can-utils

# 2. 加载模块
sudo modprobe can can_raw can_dev

# 3. 创建udev规则
sudo cp /etc/udev/rules.d/99-can-up.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules

# 4. 启动CAN接口
sudo ip link set can0 up type can bitrate 1000000
```

## 🐛 故障排除

### 问题：CAN接口无法启动
```bash
# 检查驱动
lsmod | grep can

# 查看错误信息
dmesg | grep can

# 重新加载模块
sudo modprobe -r can_dev can_raw can
sudo modprobe can_dev can_raw can
```

### 问题：无数据接收
```bash
# 检查CAN连接
candump can0

# 验证波特率
ip -details link show can0

# 测试发送
cansend can0 001#1122334455667788
```

### 问题：数据异常
- 检查CAN ID配置是否匹配下位机
- 验证数据格式（大端序）
- 确认电压和终端电阻

## 📖 详细文档

完整的使用指南请参考：`USB转CAN使用指南.md`

## 🆘 获取帮助

1. 查看系统日志：`dmesg | grep can`
2. 检查网络接口：`ip link show can0`
3. 监听CAN帧：`candump can0`
4. 查看项目日志：`tail -f logs/*.log`

## 📞 技术支持

- [RoboMaster论坛](https://bbs.robomaster.com)
- [周立功CAN技术支持](http://www.zlg.cn)
- 项目Issues
