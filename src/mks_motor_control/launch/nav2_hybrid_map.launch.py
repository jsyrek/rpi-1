"""
Nav2 Hybrid Map Launch File - MAKSYMALNA JAKOŚĆ LiDAR (bez ograniczeń)
Używa:
- Odometry z motorów
- Unitree LiDAR L2 - PEŁNA GĘSTOŚĆ PUNKTÓW (cloud_scan_num=5)
- Kompletne TF tree
- Initial pose (0.3, 0.3)m na mapie stołu
- Nav2/AMCL lokalizacja
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mks_motor_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params_odom_only.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')
    map_yaml_file = os.path.expanduser('~/maps/table_2x1.5m_edges.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_file = LaunchConfiguration('map', default=map_yaml_file)
    
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='True')
    declare_map_cmd = DeclareLaunchArgument('map', default_value=map_yaml_file)

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': use_sim_time}]
    )

    # 2. INITIAL POSE: map -> base_link (robot start 30cm,30cm)
    initial_pose_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_initial_pose',
        arguments=['0.3', '0.3', '0', '0', '0', '0', 'map', 'base_link']
    )

    # 3. TF #1: base_link -> unilidar_imu_initial (montaż 15cm)
    #base_to_imu_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='base_to_lidar_imu',
    #    arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'unilidar_imu_initial']
    #)

    # 4. TF #2: unilidar_imu_initial -> unilidar_lidar (kąt 112.5°)
    #imu_to_lidar_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    name='imu_to_lidar_tf',
    #    arguments=['0', '0', '0', '0', '1.9635', '0', 'unilidar_imu_initial', 'unilidar_lidar']
    #)
    
    # Jeden TF: base_link → unilidar_lidar (montaż + kąt)
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_unilidar_lidar',
        arguments=['0.1', '0', '0.2', '0', '1.9635', '0', 'base_link', 'unilidar_lidar']
    )

    # 5. UNITREE L2 LiDAR - MAKSYMALNA GĘSTOŚĆ PUNKTÓW!
    unitree_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_max',
        output='screen',
        parameters=[{
            'initialize_type': 2,           # IMU init
            'work_mode': 1,                 # Normal mode
            'cloud_scan_num': 5,            # MAKSIMUM PUNKTÓW (pełna gęstość)
            'lidar_ip': '192.168.1.62',     # ZMIEŃ NA SWÓJ IP
            'local_ip': '192.168.1.2',      # ZMIEŃ NA IP PC
            'udp_timeout': 3.0,             # Więcej czasu na pakiety
            'packet_buffer_size': 50000     # Duży bufor na max gęstość
        }]
    )

    # 6. Motor Driver (odometry)
    motor_driver_node = Node(
        package='mks_motor_control',
        executable='motor_driver_speed',
        output='screen',
        parameters=[controller_config]
    )

    # 7. Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_file}]
    )
    
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server']
        }]
    )

    # 8. Hybrid Localization
    hybrid_localization_node = Node(
        package='mks_motor_control',
        executable='hybrid_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 9. Nav2 Stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_map_cmd,
        
        # Core nodes
        robot_state_publisher_node,
        initial_pose_tf,           # map → base_link (pozycja start)
        base_to_lidar_tf,          # base_link → unilidar_lidar (jedyny TF LiDAR!)
        
        # Sensors
        unitree_lidar_node,
        motor_driver_node,
        
        # Nav2
        map_server_node,
        lifecycle_manager_map,
        hybrid_localization_node,
        nav2_bringup_launch,
    ])

