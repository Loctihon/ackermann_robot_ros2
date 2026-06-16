from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Lấy file cấu hình nút bấm từ package ackermann_control
    joy_config_path = os.path.join(
        get_package_share_directory('ackermann_control'),
        'config',
        'xbox.yaml'
    )

    # 2. Gọi lại file launch Gazebo tổng của bro (file đã sửa lỗi vật lý và tích hợp node C++)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robot_2_description'),
                'launch',
                'gazebo.launch.py'
            )
        ])
    )

    # 3. Node đọc driver tay cầm Xbox
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.05}]
    )

    # 4. Node dịch nút bấm tay cầm thành /cmd_vel
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_path]
    )

    return LaunchDescription([
        gazebo_launch,
        joy_node,
        teleop_joy_node
    ])