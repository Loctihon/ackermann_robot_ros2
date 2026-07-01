import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Đường dẫn tới file cấu hình tay cầm xbox.yaml của bro
    xbox_config_path = os.path.join(
        get_package_share_directory('ackermann_control'),
        'config',
        'xbox.yaml'
    )
    camera_v4l2 = Node(
        package='v4l2_camera', 
        executable='v4l2_camera_node',
        name='real_camera_node',
        parameters=[{'image_size': [640, 480]}], 
        remappings=[('/image_raw', '/camera/image_raw')] 
    )

    # 3. NÃO BỘ NHẬN DIỆN VÀ BÁM LÀN (OpenCV + YOLO)
    detect_line_node = Node(
        package='robot_2_description',
        # Lưu ý: Nhớ ghi rõ đuôi .py nếu tên file thực thi của bro có đuôi này
        executable='detect_line', 
        name='detect_line',
        output='screen'
    )

    # Node 1: Đọc tín hiệu tay cầm vật lý (joy_node)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Node 2: Dịch tín hiệu tay cầm thành lệnh /cmd_vel ăn theo file xbox.yaml
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[xbox_config_path],
        output='screen'
    )

    # Node 3: Bộ não động học Ackermann C++
    ackermann_kinematics_node = Node(
        package='ackermann_control',
        executable='ackermann_kinematics',
        name='ackermann_kinematics',
        output='screen'
    )

    # Node 4: Cầu nối Bluetooth Socket bắn chuỗi xuống STM32
    serial_bridge_node = Node(
        package='robot_2_description',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen'
    )

    # Gom tất cả các node lại để khởi chạy đồng thời
    return LaunchDescription([
        joy_node,
        teleop_node,
        camera_v4l2,
        detect_line_node,
        ackermann_kinematics_node,
        serial_bridge_node,
    ])