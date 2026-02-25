import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 参数定义
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    
    # 获取包路径
    # 注意：这里假设这些包已经正确安装在工作空间中
    # 如果是本地开发环境，可能需要确认包名是否一致
    try:
        ros_robot_controller_pkg = get_package_share_directory('ros_robot_controller')
        tank_base_controller_pkg = get_package_share_directory('tank_base_controller')
    except Exception as e:
        print(f"Error finding packages: {e}")
        # 这里为了演示，如果是未编译环境，可能会报错，实际部署时应确保包已编译
        return LaunchDescription([])

    # 1. 启动底层硬件驱动 (ROS Robot Controller)
    # 这对应于原文件中的 robot_controller_launch
    ros_robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_robot_controller_pkg, 'launch', 'ros_robot_controller.launch.py')
        ),
        launch_arguments={
            'imu_frame': imu_frame,
        }.items()
    )

    # 2. 启动新的底盘控制器 (Tank Base Controller)
    # 这替代了原文件中的 odom_publisher_node
    tank_base_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tank_base_controller_pkg, 'launch', 'tank_base_controller.launch.py')
        )
    )

    # 3. (可选) 启动其他辅助节点，如静态 TF 发布等
    # 原文件中可能有 robot_description 等，这里只保留最核心的控制部分

    return LaunchDescription([
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        ros_robot_controller_launch,
        tank_base_controller_launch,
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
