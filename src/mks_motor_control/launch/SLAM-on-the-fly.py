"""
SLAM on-the-fly Launch File - Minimal vanilla configuration
Uses standard Nav2 bringup with SLAM Toolbox
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('mks_motor_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup nav2 stack')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': use_sim_time
        }]
    )

    # Motor Driver
    motor_driver_node = Node(
        package='mks_motor_control',
        executable='motor_driver_speed',
        output='screen',
        parameters=[controller_config]
    )

    # Static TF: base_link -> unilidar_lidar
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_unilidar_lidar',
        output='screen',
        arguments=['0.1', '0', '0.2', '0', '1.9635', '0', 'base_link', 'unilidar_lidar']
    )

    # Static TF: odom -> base_link (identity) - CRITICAL fallback when motor_driver doesn't publish TF
    # This ensures the TF tree is connected even without CAN/encoders
    # motor_driver_speed will override this with dynamic TF when working
    odom_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_static',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Unitree LiDAR
    unitree_lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_l2',
        output='screen',
        parameters=[{
            'initialize_type': 2,
            'work_mode': 1,
            'cloud_scan_num': 3,
            'lidar_ip': '192.168.1.62',
            'local_ip': '192.168.1.2',
            'udp_timeout': 3.0,
            'packet_buffer_size': 30000
        }]
    )

    # PointCloud2 -> LaserScan - DELAYED by 2 seconds
    # Gives motor_driver time to start publishing odom -> base_link TF
    # target_frame: 'unilidar_lidar' means no TF lookup needed (source == target)
    pointcloud_to_laserscan_node = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                output='screen',
                parameters=[{
                    'target_frame': 'unilidar_lidar',  # Same as cloud frame - no TF needed
                    'transform_tolerance': 0.1,
                    'min_height': -2.0,
                    'max_height': 2.0,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.00872665,
                    'scan_time': 0.1,
                    'range_min': 0.05,
                    'range_max': 5.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                remappings=[
                    ('cloud_in', '/unilidar/cloud'),
                    ('scan', '/scan')
                ]
            )
        ]
    )

    # Scan Throttle - DELAYED by 3 seconds (after pointcloud_to_laserscan)
    scan_throttle_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='mks_motor_control',
                executable='scan_throttle',
                name='scan_throttle',
                output='screen',
                parameters=[{
                    'throttle_rate': 5.0,  # Hz
                    'input_topic': '/scan',
                    'output_topic': '/scan_throttled'
                }]
            )
        ]
    )

    # SLAM Toolbox (standard async mode) - DELAYED by 10 seconds
    # Delay allows TF tree to fully initialize (odom -> base_link needs time to be visible)
    slam_toolbox_node = TimerAction(
        period=10.0,  # 10 second delay for TF initialization
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # SLAM Lifecycle Manager - DELAYED by 12 seconds (after slam_toolbox)
    lifecycle_manager_slam = TimerAction(
        period=12.0,  # Start after slam_toolbox
        actions=[
            Node(
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
        ]
    )

    # Nav2 Bringup (standard vanilla) - DELAYED by 15 seconds
    nav2_bringup_launch = TimerAction(
        period=15.0,  # Start after SLAM is ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        robot_state_publisher_node,
        motor_driver_node,
        base_to_lidar_tf,
        odom_to_base_link_tf,  # CRITICAL: connects odom -> base_link when motor_driver fails
        unitree_lidar_node,
        pointcloud_to_laserscan_node,
        scan_throttle_node,
        slam_toolbox_node,
        lifecycle_manager_slam,
        nav2_bringup_launch,
    ])
