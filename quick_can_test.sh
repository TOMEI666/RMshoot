#!/bin/bash

# 快速CAN通信测试脚本

echo "======================================="
echo "        快速CAN测试"
echo "======================================="

# 检查接口状态
echo "1. CAN接口状态："
ip link show can0

# 发送测试帧
echo ""
echo "2. 发送测试CAN帧..."
cansend can0 001#1122334455667788 2>/dev/null && echo "✓ 四元数测试帧发送成功" || echo "✗ 四元数测试帧发送失败"
cansend can0 110#162E0100 2>/dev/null && echo "✓ 子弹速度测试帧发送成功" || echo "✗ 子弹速度测试帧发送失败"

# 监听数据
echo ""
echo "3. 监听CAN数据（10秒）..."
echo "如果看到数据帧，说明下位机正在发送数据..."
echo "按Ctrl+C停止监听"
echo ""

timeout 10 candump can0 || echo "✗ 10秒内未收到任何CAN数据"

echo ""
echo "======================================="
echo "        测试完成"
echo "======================================="
