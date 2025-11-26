"""
Kompletny launch file dla motor_driver_speed - OPTIMIZED dla RPi 4

UWAGA: Ta wersja ZAKŁADA że CAN interface jest już inicjalizowany
       przez Systemd Service: /etc/systemd/system/can-init.service
       
INSTRUKCJA SETUP (jednorazowo):
  1. sudo nano /etc/systemd/system/can-init.service
  2. [Dodaj zawartość z CAN_INITIALIZATION.md]
  3. sudo systemctl daemon-reload
  4. sudo systemctl enable can-init.service
  5. sudo systemctl start can-init.service
  6. Sprawdź: ip link show can0 (powinna być flaga UP)

PO SETUP: Po prostu uruchom ten launch file!
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mks_motor_control')
    config_file = os.path.join(pkg_share, 'config', 'controller.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'two_wheel_robot.urdf')

    # ============================================================
    # KROK 1: Weryfikacja CAN interface
    # ============================================================
    # CAN powinno być UP z systemd service, tylko sprawdzamy
    
    log_can_check = LogInfo(msg="[1/4] Sprawdzam CAN interface...")
    
    can_check = ExecuteProcess(
        cmd=['ip', 'link', 'show', 'can0'],
        output='screen',
        shell=False
    )

    # ============================================================
    # KROK 2: Robot State Publisher (dla URDF)
    # ============================================================
    
    log_rsp = LogInfo(msg="[2/4] Uruchamiam Robot State Publisher...")
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # ============================================================
    # KROK 3: Motor Driver Speed (główny sterownik)
    # ============================================================
    
    log_motor = LogInfo(msg="[3/4] Uruchamiam Motor Driver Speed...")
    
    motor_driver_node = Node(
        package='mks_motor_control',
        executable='motor_driver_speed',
        output='screen',
        parameters=[
            config_file,
            {
                'wheel_radius': 0.05,
                'wheel_separation': 0.18,
                'motor_1_inverted': False,
                'motor_2_inverted': True,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
            }
        ]
    )

    # ============================================================
    # KROK 4: Joint State Publisher (opcjonalnie)
    # ============================================================
    # Jeśli chcesz publikować joint states dla RViz
    
    log_jsp = LogInfo(msg="[4/4] Uruchamiam Joint State Publisher...")
    
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        # ========== STARTUP ==========
        LogInfo(msg=""),
        LogInfo(msg="=" * 70),
        LogInfo(msg="MOTOR_DRIVER_SPEED LAUNCH FILE"),
        LogInfo(msg="Optimized dla: Raspberry Pi 4 + Systemd CAN Service"),
        LogInfo(msg="=" * 70),
        LogInfo(msg=""),
        
        # ========== CAN CHECK ==========
        log_can_check,
        can_check,
        
        # ========== ROBOT STATE PUBLISHER ==========
        log_rsp,
        rsp_node,
        
        # ========== MOTOR DRIVER ==========
        log_motor,
        motor_driver_node,
        
        # ========== JOINT STATE PUBLISHER ==========
        # Skomentować jeśli nie potrzebne
        # log_jsp,
        # jsp_node,
        
        # ========== SUMMARY ==========
        LogInfo(msg=""),
        LogInfo(msg="=" * 70),
        LogInfo(msg="✅ Motor Driver Speed uruchomiony!"),
        LogInfo(msg="=" * 70),
        LogInfo(msg=""),
        LogInfo(msg="Dostępne topics:"),
        LogInfo(msg="  /cmd_vel                    - wejście: komendy prędkości"),
        LogInfo(msg="  /odom                       - wyjście: odometria"),
        LogInfo(msg="  /joint_states               - wyjście: stany kół"),
        LogInfo(msg=""),
        LogInfo(msg="Dostępne nodes:"),
        LogInfo(msg="  /motor_driver_speed         - główny sterownik"),
        LogInfo(msg="  /robot_state_publisher      - publikuje URDF"),
        LogInfo(msg=""),
        LogInfo(msg="Aby zatrzymać: Ctrl+C"),
        LogInfo(msg=""),
    ])
