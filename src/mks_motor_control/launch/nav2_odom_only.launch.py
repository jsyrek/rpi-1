"""
Nav2 Odom-Only Launch File - Nawigacja bez mapy i LIDARa
Używa tylko odometrii z motor_driver_speed
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('mks_motor_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to config files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params_odom_only.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')

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
    # 2. Motor Driver Speed (z odometrią)
    # ============================================================
    motor_driver_node = Node(
        package='mks_motor_control',
        executable='motor_driver_speed',
        output='screen',
        parameters=[
            controller_config,
            {
                'wheel_radius': 0.05,
                'wheel_separation': 0.18,
                'motor_1_inverted': False,
                'motor_2_inverted': True,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ============================================================
    # 3. Static TF: odom -> map (zamiast SLAM)
    # ============================================================
    static_tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # ============================================================
    # 4. Nav2 Bringup (Navigation Stack) - BEZ map_server!
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
        
        # Nodes
        robot_state_publisher_node,
        motor_driver_node,
        static_tf_odom_to_map,
        nav2_bringup_launch,
    ])
