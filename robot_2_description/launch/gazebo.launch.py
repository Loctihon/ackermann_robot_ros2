from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('robot_2_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'robot_2.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Node phát thông số robot (BẮT BUỘC PHẢI CÓ)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    # Khởi động Server Gazebo
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    # Khởi động Giao diện Gazebo (Client)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Node đưa mô hình xe vào Gazebo
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_2',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # --- CẤU HÌNH CÁC CONTROLLER TỪ ROS2_CONTROL ---
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller"],
    )

    load_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
    )

    # --- TÍCH HỢP NODE C++ ĐỘNG HỌC ACKERMANN ---
    ackermann_kinematics_node = Node(
        package='ackermann_control',                      # Tên package chứa node C++
        executable='ackermann_kinematics',       # Tên file thực thi (đã khai báo trong CMakeLists.txt)
        name='ackermann_kinematics',
        output='screen'
    )


    return LaunchDescription([
        # 1. Khởi động phần lõi hệ thống và đồ họa trước
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        
        # 2. Kích hoạt các bộ điều khiển lốp và góc lái
        load_position_controller,
        load_velocity_controller,
        load_joint_state_broadcaster,
        
        # 3. Kích hoạt bộ não dịch chuyển góc lái C++
        ackermann_kinematics_node
    ])