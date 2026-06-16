from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lấy đường dẫn file cấu hình nút bấm tay cầm
    joy_config_path = os.path.join(
        get_package_share_directory('ackermann_control'),
        'config',
        'xbox.yaml'
    )

    # 1. Node đọc driver phần cứng của tay cầm Xbox
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.05}] # Tránh bị trôi cần gạt khi không bấm
    )

    # 2. Node dịch chuyển đổi từ nút bấm sang lệnh cmd_vel
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_path]
    )

    # 3. Node C++ tính toán động học nghịch Ackermann (Code cũ của mình)
    ackermann_kinematics_node = Node(
        package='ackermann_control',
        executable='ackermann_kinematics',
        name='ackermann_kinematics',
        output='screen'
    )

    # 4. Node C++ cầu nối Serial giao tiếp UART với STM32
    serial_bridge_node = Node(
        package='ackermann_control',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_joy_node,
        ackermann_kinematics_node,
        serial_bridge_node
    ])