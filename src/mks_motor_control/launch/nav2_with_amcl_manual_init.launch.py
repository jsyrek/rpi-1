"""
Nav2 Bringup with AMCL and Manual Initial Pose Setting
For robot with laser pointer offset: -0.30m (rear) x -0.30m (left)

Usage:
  ros2 launch mks_motor_control nav2_with_amcl_manual_init.launch.py \
    map_yaml_file:=path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    amcl_params_file = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'URDF', 'two_wheel_robot.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_theta = LaunchConfiguration('initial_pose_theta')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')
    
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map_yaml_file',
        default_value='',
        description='Full path to map.yaml file for AMCL')
    
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.30',
        description='Initial X position (in meters)')
    
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.30',
        description='Initial Y position (in meters)')
    
    declare_initial_pose_theta_cmd = DeclareLaunchArgument(
        'initial_pose_theta',
        default_value='0.0',
        description='Initial orientation (in radians)')

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
    # 2. Motor Driver Speed (with odometry)
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
    # 3. Initial Pose Setter
    # ============================================================
    initial_pose_setter_node = Node(
        package='mks_motor_control',
        executable='initial_pose_setter',
        output='screen',
        parameters=[
            {
                'initial_x': initial_pose_x,
                'initial_y': initial_pose_y,
                'initial_theta': initial_pose_theta,
                'cov_x': 0.25,
                'cov_y': 0.25,
                'cov_theta': 0.068
            }
        ]
    )

    # ============================================================
    # 4. Nav2 Bringup (includes AMCL)
    # ============================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'map': map_yaml_file
        }.items()
    )

    # ============================================================
    # Launch Description
    # ============================================================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_map_yaml_file_cmd,
        declare_initial_pose_x_cmd,
        declare_initial_pose_y_cmd,
        declare_initial_pose_theta_cmd,
        
        # Nodes
        robot_state_publisher_node,
        motor_driver_node,
        initial_pose_setter_node,
        nav2_bringup_launch,
    ])
