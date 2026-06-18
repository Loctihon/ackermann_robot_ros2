import os
from launch_ros.actions import Node
from launch import LaunchDescription
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. LẤY THÔNG SỐ KHUNG XE (Cần thiết để ROS 2 hiểu vị trí lắp Camera thật)
    share_dir = get_package_share_directory('robot_2_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot_2.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # 2. KHỞI ĐỘNG WEBCAM THỰC TẾ
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

    # 4. HỆ ĐỘNG HỌC ACKERMANN (Tính toán góc lái, vận tốc)
    ackermann_kinematics_node = Node(
        package='ackermann_control',
        executable='ackermann_kinematics',
        name='ackermann_kinematics',
        output='screen'
    )

    # 5. CẦU NỐI BLUETOOTH XUỐNG STM32
    serial_bridge_node = Node(
        package='robot_2_description',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        camera_v4l2,
        detect_line_node,
        ackermann_kinematics_node,
        serial_bridge_node
    ])