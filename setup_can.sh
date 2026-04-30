#!/bin/bash

# USB转CAN快速设置脚本
# 用于配置Linux系统下的USB转CAN设备
# 支持周立功USBCAN-II、Peak CAN-USB等主流设备

set -e  # 遇到错误立即退出

echo "======================================="
echo "    USB转CAN设备设置脚本"
echo "    RoboMaster视觉系统"
echo "======================================="

# 检查root权限
if [[ $EUID -ne 0 ]]; then
   echo "请使用sudo运行此脚本："
   echo "sudo $0"
   exit 1
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查操作系统
check_os() {
    if [[ ! -f /etc/os-release ]]; then
        log_error "不支持的操作系统"
        exit 1
    fi

    . /etc/os-release
    case $ID in
        ubuntu|debian)
            log_info "检测到 $PRETTY_NAME"
            ;;
        *)
            log_warn "未测试的操作系统: $PRETTY_NAME"
            ;;
    esac
}

# 安装依赖包
install_dependencies() {
    log_info "安装CAN工具和依赖..."

    apt update
    apt install -y can-utils linux-modules-extra-$(uname -r)

    if [[ $? -eq 0 ]]; then
        log_info "依赖安装完成"
    else
        log_error "依赖安装失败"
        exit 1
    fi
}

# 加载内核模块
load_kernel_modules() {
    log_info "加载CAN内核模块..."

    modprobe can 2>/dev/null || true
    modprobe can_raw 2>/dev/null || true
    modprobe can_dev 2>/dev/null || true

    # 检查模块是否加载成功
    if lsmod | grep -q "^can "; then
        log_info "CAN模块加载成功"
    else
        log_error "CAN模块加载失败"
        exit 1
    fi
}

# 创建udev规则
create_udev_rules() {
    log_info "创建udev规则..."

    local rules_file="/etc/udev/rules.d/99-can-up.rules"

    cat > "$rules_file" << 'EOF'
# USB转CAN设备自动配置规则
# 支持1000kbps波特率

# 周立功USBCAN-II
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", RUN+="/sbin/ip link set %k up type can bitrate 1000000 dbitrate 5000000 restart-ms 1000"

# Peak CAN-USB
ACTION=="add", ATTR{idVendor}=="0c72", ATTR{idProduct}=="000c", RUN+="/sbin/ip link set %k up type can bitrate 1000000"

# 通用CAN设备
ACTION=="add", KERNEL=="can[0-9]*", RUN+="/sbin/ip link set %k up type can bitrate 1000000"

# 虚拟CAN设备（用于测试）
ACTION=="add", KERNEL=="vcan*", RUN+="/sbin/ip link set %k up"
EOF

    if [[ $? -eq 0 ]]; then
        log_info "udev规则创建完成: $rules_file"
    else
        log_error "udev规则创建失败"
        exit 1
    fi
}

# 重新加载udev规则
reload_udev_rules() {
    log_info "重新加载udev规则..."

    udevadm control --reload-rules
    udevadm trigger

    if [[ $? -eq 0 ]]; then
        log_info "udev规则重新加载完成"
    else
        log_error "udev规则重新加载失败"
        exit 1
    fi
}

# 配置systemd-networkd（可选）
configure_systemd_networkd() {
    log_info "配置systemd-networkd..."

    local network_file="/etc/systemd/network/80-can0.network"

    cat > "$network_file" << 'EOF'
[Match]
Name=can0

[CAN]
BitRate=1000000
DataBitRate=5000000
FDMode=yes
FDNonISO=no
RestartSec=100ms
EOF

    systemctl enable systemd-networkd 2>/dev/null || true
    systemctl start systemd-networkd 2>/dev/null || true

    log_info "systemd-networkd配置完成"
}

# 测试CAN接口
test_can_interface() {
    log_info "测试CAN接口..."

    # 等待udev规则生效
    sleep 2

    # 检查can0接口是否存在
    if ip link show can0 &>/dev/null; then
        log_info "CAN接口 can0 已创建"

        # 显示接口信息
        ip link show can0

        # 测试虚拟回环（如果有其他CAN设备连接）
        log_info "CAN接口测试完成"
        log_info "可以使用以下命令测试："
        echo "  candump can0          # 监听CAN帧"
        echo "  cansend can0 123#DEADBEEF    # 发送测试帧"

    else
        log_warn "CAN接口 can0 未检测到"
        log_info "可能的原因："
        echo "  1. USB转CAN设备未连接"
        echo "  2. 设备驱动未正确安装"
        echo "  3. udev规则需要重启生效"
        echo ""
        log_info "请连接USB转CAN设备后重试"
    fi
}

# 创建测试脚本
create_test_script() {
    log_info "创建测试脚本..."

    local test_script="/usr/local/bin/test_can.sh"

    cat > "$test_script" << 'EOF'
#!/bin/bash

echo "======================================="
echo "        CAN通信测试脚本"
echo "======================================="

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo "CAN接口状态："
ip link show can0 2>/dev/null || echo "can0 接口不存在"

echo ""
echo "测试选项："
echo "1. 监听CAN帧 (candump can0)"
echo "2. 发送测试帧 (cansend can0 123#DEADBEEF)"
echo "3. 创建虚拟CAN接口测试"
echo "4. 运行RoboMaster视觉系统测试"
echo ""

read -p "选择测试项目 (1-4): " choice

case $choice in
    1)
        log_info "开始监听CAN帧..."
        candump can0
        ;;
    2)
        log_info "发送测试CAN帧..."
        cansend can0 123#DEADBEEF
        log_info "测试帧已发送"
        ;;
    3)
        log_info "创建虚拟CAN接口用于测试..."
        modprobe vcan
        ip link add dev vcan0 type vcan
        ip link set up vcan0
        log_info "虚拟CAN接口 vcan0 已创建"
        log_info "测试命令：candump vcan0 & cansend vcan0 123#DEADBEEF"
        ;;
    4)
        if [[ -f "./build/cboard_test" ]]; then
            log_info "运行C板通信测试..."
            ./build/cboard_test configs/sentry.yaml
        else
            log_error "测试程序不存在，请先编译项目"
            echo "编译命令：cmake -B build && make -C build -j$(nproc)"
        fi
        ;;
    *)
        log_error "无效选择"
        ;;
esac
EOF

    chmod +x "$test_script"

    if [[ $? -eq 0 ]]; then
        log_info "测试脚本创建完成: $test_script"
        log_info "运行测试: sudo test_can.sh"
    fi
}

# 创建开机自启服务
create_startup_service() {
    log_info "创建CAN开机自启服务..."

    local service_file="/etc/systemd/system/can-setup.service"

    cat > "$service_file" << 'EOF'
[Unit]
Description=CAN Interface Setup
After=network.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup_can_interfaces.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    # 创建CAN接口设置脚本
    cat > "/usr/local/bin/setup_can_interfaces.sh" << 'EOF'
#!/bin/bash

# CAN接口设置脚本
# 由systemd服务调用

# 加载内核模块
modprobe can 2>/dev/null || true
modprobe can_raw 2>/dev/null || true
modprobe can_dev 2>/dev/null || true

# 设置CAN接口（如果存在）
for interface in can0 can1; do
    if ip link show "$interface" &>/dev/null; then
        ip link set "$interface" up type can bitrate 1000000
        echo "CAN interface $interface configured"
    fi
done
EOF

    chmod +x "/usr/local/bin/setup_can_interfaces.sh"

    # 启用服务
    systemctl daemon-reload
    systemctl enable can-setup.service 2>/dev/null || true

    log_info "CAN开机自启服务创建完成"
}

# 主函数
main() {
    echo "开始USB转CAN设备设置..."
    echo ""

    check_os
    install_dependencies
    load_kernel_modules
    create_udev_rules
    reload_udev_rules
    configure_systemd_networkd
    create_startup_service
    test_can_interface
    create_test_script

    echo ""
    log_info "======================================="
    log_info "    USB转CAN设置完成！"
    log_info "======================================="
    echo ""
    log_info "使用说明："
    echo "1. 连接USB转CAN设备到电脑"
    echo "2. 连接CAN线到RoboMaster C板"
    echo "3. 运行测试: sudo test_can.sh"
    echo "4. 运行视觉系统: ./build/auto_aim_debug_mpc configs/sentry.yaml"
    echo ""
    log_info "配置文件位置: configs/sentry.yaml"
    log_info "CAN接口配置: can_interface: \"can0\""
    echo ""
    log_info "如有问题，请查看文档: USB转CAN使用指南.md"
}

# 运行主函数
main "$@"
