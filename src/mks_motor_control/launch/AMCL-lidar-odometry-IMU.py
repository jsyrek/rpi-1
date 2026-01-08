"""
AMCL Launch File - Lokalizacja na mapie z LiDAR, odometrią i IMU
Używa:
- Odometry z motorów MKS (motor_driver_speed) - publikuje /odom
- Unitree LiDAR L2 - wykrywanie krawędzi stołu - publikuje PointCloud2
- PointCloud2 -> LaserScan converter - konwersja dla AMCL
- AMCL - lokalizacja na mapie używając LiDAR + odometrii
- Map Server - mapa stołu 2x1.5m (domyślnie ~/maps/table_2x1.5m_edges.yaml)
- Initial pose: (0.2, 0.2) - lewy dolny róg stołu = (0, 0)

WYMAGANIA:
- Pakiet: pointcloud_to_laserscan (sudo apt install ros-<distro>-pointcloud-to-laserscan)
- Mapa stołu w ~/maps/table_2x1.5m_edges.yaml
- Konfiguracja IP LiDAR w parametrach (lidar_ip, local_ip)

UŻYCIE:
ros2 launch mks_motor_control AMCL-lidar-odometry-IMU.py map:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('mks_motor_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')  # Używamy wersji z mapą!
    amcl_config_file = os.path.join(pkg_share, 'config', 'amcl_config.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')
    
    # Map file path - domyślnie w ~/maps/
    map_yaml_file = os.path.expanduser('~/maps/table_2x1.5m_edges.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_file = LaunchConfiguration('map', default=map_yaml_file)
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map yaml file to load')

    # ============================================================
    # 0. CAN Interface Initialization (przed wszystkimi node'ami)
    # Inicjalizacja interfejsu CAN dla canable adaptera
    # ============================================================
    can_init_setup = ExecuteProcess(
        cmd=['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can', 'bitrate', '1000000'],
        output='screen',
        name='can_setup_bitrate'
    )
    
    # Druga komenda uruchamia się z małym opóźnieniem po pierwszej
    can_init_up = TimerAction(
        period=0.5,  # 0.5 sekundy opóźnienia
        actions=[
            ExecuteProcess(
                cmd=['sudo', 'ip', 'link', 'set', 'can0', 'up'],
                output='screen',
                name='can_setup_up'
            )
        ]
    )
    
    # Motor driver startuje z opóźnieniem (po inicjalizacji CAN)
    motor_driver_node = TimerAction(
        period=1.5,  # 1.5 sekundy opóźnienia (po CAN setup)
        actions=[
            Node(
                package='mks_motor_control',
                executable='motor_driver_speed',
                output='screen',
                parameters=[controller_config]
            )
        ]
    )

    # ============================================================
    # 1. Robot State Publisher
    # ============================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': use_sim_time
        }]
    )

    # ============================================================
    # 3. TF: base_link -> unilidar_lidar
    # Montaż: x=0.1m (przód), z=0.2m (wysokość)
    # Kąt: pitch 112.5° = 1.9635 rad (skierowany w dół)
    # ============================================================
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_unilidar_lidar',
        output='screen',
        arguments=['0.1', '0', '0.2', '0', '1.9635', '0', 'base_link', 'unilidar_lidar']
    )

    # ============================================================
    # 4. Unitree L2 LiDAR Node
    # Publikuje PointCloud2 na /unilidar/cloud
    # ============================================================
    unitree_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_l2',
        output='screen',
        parameters=[{
            'initialize_type': 2,           # IMU initialization
            'work_mode': 1,                 # Normal mode
            'cloud_scan_num': 3,            # Gęstość punktów (3-5 dla lepszej jakości)
            'lidar_ip': '192.168.1.62',     # IP LiDAR - ZMIEŃ NA SWÓJ!
            'local_ip': '192.168.1.2',      # IP Raspberry Pi - ZMIEŃ NA SWÓJ!
            'udp_timeout': 3.0,
            'packet_buffer_size': 30000
        }]
    )

    # ============================================================
    # 5. PointCloud2 -> LaserScan Converter
    # AMCL wymaga LaserScan, a Unitree publikuje PointCloud2
    # ============================================================
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'unilidar_lidar',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 0.5,
            'angle_min': -3.14159,  # -180°
            'angle_max': 3.14159,   # +180°
            'angle_increment': 0.00872665,  # ~0.5° resolution
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 3.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1
        }],
        remappings=[
            ('cloud_in', '/unilidar/cloud'),  # Topic z Unitree L2 - sprawdź w runtime!
            ('scan', '/scan')
        ]
    )

    # ============================================================
    # 6. Map Server
    # Ładuje mapę stołu do Nav2
    # ============================================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }]
    )
    
    # Lifecycle manager dla map_server
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

    # ============================================================
    # 7. AMCL (Adaptive Monte Carlo Localization)
    # Lokalizuje robota na mapie używając LiDAR + odometrii
    # ============================================================
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_file],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
    )

    # Lifecycle manager dla AMCL
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['amcl']
        }]
    )

    # ============================================================
    # 8. Nav2 Bringup (Navigation Stack)
    # Używa nav2_params.yaml z konfiguracją dla mapy
    # ============================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file
        }.items()
    )

    # ============================================================
    # Launch Description
    # ============================================================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_map_cmd,
        
        # CAN Interface Initialization (pierwsze!)
        can_init_setup,
        can_init_up,
        
        # Core nodes
        robot_state_publisher_node,
        motor_driver_node,  # Startuje z opóźnieniem po CAN
        
        # LiDAR setup
        base_to_lidar_tf,
        unitree_lidar_node,
        pointcloud_to_laserscan_node,
        
        # Localization
        map_server_node,
        lifecycle_manager_map,
        amcl_node,
        lifecycle_manager_amcl,
        
        # Navigation
        nav2_bringup_launch,
    ])
