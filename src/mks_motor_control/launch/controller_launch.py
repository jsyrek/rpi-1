from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Pobierz katalog pakietu mks_motor_control
    pkg_share = get_package_share_directory('mks_motor_control')
    
    # Pełne ścieżki do plików po instalacji
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')
    controller_yaml = os.path.join(pkg_share, 'config', 'controller.yaml')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_yaml, {'robot_description': open(urdf_path).read()}],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['differential_drive_controller'],
            output='screen'
        )
    ])
