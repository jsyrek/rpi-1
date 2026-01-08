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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    # ============================================================
    # 0. CAN Interface Initialization (przed wszystkimi node'ami)
    # Inicjalizacja interfejsu CAN dla canable adaptera
    # Uwaga: Wymaga sudo bez hasła (NOPASSWD) lub uruchom jako root
    # Alternatywnie: uruchom ręcznie przed launch file:
    #   sudo ip link set can0 type can bitrate 1000000
    #   sudo ip link set can0 up
    # Błędy "Device or resource busy" są ignorowane (CAN już skonfigurowany)
    # ============================================================
    can_init_setup = ExecuteProcess(
        cmd=['sh', '-c', 'sudo -n ip link set can0 type can bitrate 1000000 || true'],
        output='screen',
        name='can_setup_bitrate',
        shell=True
    )
    
    # Druga komenda uruchamia się z małym opóźnieniem po pierwszej
    can_init_up = TimerAction(
        period=0.5,  # 0.5 sekundy opóźnienia
        actions=[
            ExecuteProcess(
                cmd=['sh', '-c', 'sudo -n ip link set can0 up || true'],
                output='screen',
                name='can_setup_up',
                shell=True
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
    # 1b. Fake Odometry (Fallback dla SLAM Toolbox)
    # Publikuje podstawową /odom gdy motor_driver nie działa
    # SLAM Toolbox Message Filter wymaga synchronizacji scan z odom
    # ============================================================
    fake_odom_node = Node(
        package='mks_motor_control',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_rate': 50.0  # 50 Hz - wystarczająco dla synchronizacji
        }]
    )

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
    # Publikuje na /scan dla Nav2 i scan_throttle node
    # ============================================================
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'unilidar_lidar',
            'transform_tolerance': 0.01,
            'min_height': -2.0,              # Rozszerzony zakres - LiDAR widzi w dół
            'max_height': 2.0,               # Rozszerzony zakres - LiDAR widzi w górę
            'angle_min': -3.14159,          # -180°
            'angle_max': 3.14159,           # +180°
            'angle_increment': 0.00872665,  # ~0.5° resolution
            'scan_time': 0.014,             # ~72 Hz - oryginalna częstotliwość
            'range_min': 0.05,              # Min odległość 5cm
            'range_max': 5.0,               # Max odległość 5m (pomieszczenie + stół)
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1
        }],
        remappings=[
            ('cloud_in', '/unilidar/cloud'),  # Topic z Unitree L2
            ('scan', '/scan')                  # Topic dla Nav2 i throttling
        ]
    )

    # ============================================================
    # 5b. Scan Throttle Node
    # Redukuje częstotliwość /scan z ~72 Hz do 5 Hz dla SLAM Toolbox
    # Nav2 nadal używa pełnej częstotliwości /scan dla lepszej reaktywności
    # ============================================================
    scan_throttle_node = Node(
        package='mks_motor_control',
        executable='scan_throttle',
        name='scan_throttle',
        output='screen',
        parameters=[{
            'throttle_rate': 5.0,           # 5 Hz - wystarczająco dla SLAM
            'input_topic': '/scan',
            'output_topic': '/scan_throttled'
        }]
    )

    # ============================================================
    # 6. SLAM Toolbox (Async Mode - on-the-fly mapping)
    # Tworzy mapę w czasie rzeczywistym podczas poruszania się
    # Publikuje mapę na /map i transform map->odom
    # Używa throttled /scan_throttled topic (5 Hz zamiast 72 Hz)
    # ============================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/scan_throttled'  # Używa throttled topic (5 Hz)
            }
        ]
    )

    # Lifecycle manager dla SLAM Toolbox
    # SLAM Toolbox to lifecycle node - musi być aktywowany
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['slam_toolbox']
        }]
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
        
        # CAN Interface Initialization (pierwsze!)
        can_init_setup,
        can_init_up,
        
        # Core nodes
        robot_state_publisher_node,
        fake_odom_node,  # Fallback odometry dla SLAM Toolbox
        motor_driver_node,  # Startuje z opóźnieniem po CAN (nadpisze fake_odom jeśli działa)
        
        # LiDAR setup
        base_to_lidar_tf,
        unitree_lidar_node,
        pointcloud_to_laserscan_node,
        scan_throttle_node,
        
        # SLAM & Navigation
        slam_toolbox_node,
        lifecycle_manager_slam,
        nav2_bringup_launch,
        
        # Visualization (opcjonalnie)
        rviz_node,
    ])
