from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import FindExecutable
from launch.conditions import IfCondition

def generate_launch_description():
    # 检查用户是否在video组（避免使用sudo）
    check_group_cmd = ExecuteProcess(
        cmd=["groups"],
        output='screen',
        on_exit=[
            LogInfo(msg="---------- 重要提示 ----------"),
            LogInfo(msg="请确保用户已加入video组："),
            LogInfo(msg="1. 执行: sudo usermod -aG video $USER"),
            LogInfo(msg="2. 重新登录系统生效"),
            LogInfo(msg="3. 或配置UDEV规则避免权限问题")
        ]
    )

    # 相机节点配置（不需要sudo权限）
    camera_node = Node(
        package='daheng_camera_driver',
        executable='image_publisher',
        name='daheng_camera',
        parameters=[{
            'frame_rate': 30,
            'exposure_time': 10000.0,  # 10ms曝光
            'gain': 10.0               # 10dB增益
        }],
        output='screen',
        # 添加设备序列号选择（多相机时使用）
        arguments=['--device-serial', '']  # 留空自动选择第一个相机
    )

    return LaunchDescription([
        # 条件检查（仅当不在video组时提示）
        check_group_cmd,
        
        # 相机节点（始终启动）
        camera_node,
        
        # 调试信息
        LogInfo(msg="启动参数配置:"),
        LogInfo(msg="  曝光时间 = 10ms"),
        LogInfo(msg="  增益 = 10dB"),
        LogInfo(msg="  帧率 = 30FPS"),
        LogInfo(msg="--------------------------------"),
        LogInfo(msg="图像话题: /daheng_camera/image"),
        LogInfo(msg="参数调整: ros2 param set /daheng_camera exposure_time 20000.0")
    ])