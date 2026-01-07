"""
SLAM on-the-fly Launch File - Lokalizacja i mapowanie w czasie rzeczywistym
Używa:
- Odometry z motorów MKS (motor_driver_speed) przez CAN - publikuje /odom
- Unitree LiDAR L2 - wykrywanie otoczenia - publikuje PointCloud2
- PointCloud2 -> LaserScan converter - konwersja dla SLAM Toolbox
- SLAM Toolbox - tworzenie mapy w czasie rzeczywistym (on-the-fly)
- Nav2 Stack - nawigacja na dynamicznie tworzonej mapie

Działa w każdym pomieszczeniu bez wcześniejszego mapowania!

WYMAGANIA:
- Pakiet: pointcloud_to_laserscan (sudo apt install ros-<distro>-pointcloud-to-laserscan)
- Pakiet: slam_toolbox (sudo apt install ros-<distro>-slam-toolbox)
- Konfiguracja IP LiDAR w parametrach (lidar_ip, local_ip)
- CAN interface skonfigurowany (can0)

UŻYCIE:
ros2 launch mks_motor_control SLAM-on-the-fly.py

Zapisywanie mapy:
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('mks_motor_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Launch RViz2 for visualization')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Launch RViz2 for visualization')

    # ============================================================
    # 1. Robot State Publisher
    # Publikuje URDF model robota
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
    # 2. Motor Driver Speed (odometria z enkoderów MKS przez CAN)
    # Publikuje /odom z odometrii kół
    # ============================================================
    motor_driver_node = Node(
        package='mks_motor_control',
        executable='motor_driver_speed',
        output='screen',
        parameters=[controller_config]
    )

    # ============================================================
    # 3. TF: base_link -> unilidar_lidar
    # Montaż LiDAR: x=0.1m (przód), z=0.2m (wysokość)
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
    # UWAGA: Zmień IP na swoje!
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
            'lidar_ip': '192.168.1.62',     # IP LiDAR Unitree L2 - ZMIEŃ NA SWÓJ!
            'local_ip': '192.168.1.2',      # IP mini PC na robocie - ZMIEŃ NA SWÓJ!
            'udp_timeout': 3.0,
            'packet_buffer_size': 30000
        }]
    )

    # ============================================================
    # 5. PointCloud2 -> LaserScan Converter
    # SLAM Toolbox wymaga LaserScan, a Unitree publikuje PointCloud2
    # ============================================================
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'unilidar_lidar',
            'transform_tolerance': 0.01,
            'min_height': -0.5,              # Filtruj punkty poniżej 0.5m (podłoga)
            'max_height': 0.5,               # Filtruj punkty powyżej 0.5m (sufit)
            'angle_min': -3.14159,          # -180°
            'angle_max': 3.14159,           # +180°
            'angle_increment': 0.00872665,  # ~0.5° resolution
            'scan_time': 0.1,
            'range_min': 0.05,              # Min odległość 5cm
            'range_max': 3.0,               # Max odległość 3m (dostosuj do stołu)
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1
        }],
        remappings=[
            ('cloud_in', '/unilidar/cloud'),  # Topic z Unitree L2
            ('scan', '/scan')                  # Topic dla SLAM Toolbox
        ]
    )

    # ============================================================
    # 6. SLAM Toolbox (Async Mode - on-the-fly mapping)
    # Tworzy mapę w czasie rzeczywistym podczas poruszania się
    # Publikuje mapę na /map i transform map->odom
    # ============================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ============================================================
    # 7. Nav2 Bringup (Navigation Stack)
    # Używa dynamicznie tworzonej mapy z SLAM Toolbox
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
    # 8. RViz2 (Opcjonalnie - tylko do wizualizacji)
    # Uruchom: ros2 launch mks_motor_control SLAM-on-the-fly.py use_rviz:=True
    # ============================================================
    rviz_config_file = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ============================================================
    # Launch Description
    # ============================================================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_use_rviz_cmd,
        
        # Core nodes
        robot_state_publisher_node,
        motor_driver_node,
        
        # LiDAR setup
        base_to_lidar_tf,
        unitree_lidar_node,
        pointcloud_to_laserscan_node,
        
        # SLAM & Navigation
        slam_toolbox_node,
        nav2_bringup_launch,
        
        # Visualization (opcjonalnie)
        rviz_node,
    ])
