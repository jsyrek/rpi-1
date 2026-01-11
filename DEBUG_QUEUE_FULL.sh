#!/bin/bash
# Debug script - Sprawdź dlaczego SLAM Toolbox ma "queue is full" błędy

echo "=========================================="
echo "1. WERYFIKACJA ZMIAN W GIT"
echo "=========================================="
cd ~/ros2_ws/src/rpi-1
echo "Ostatni commit:"
git log --oneline -3
echo ""
echo "Czy są niecommitowane zmiany?"
git status --short
echo ""

echo "=========================================="
echo "2. PORÓWNANIE SOURCE vs INSTALL"
echo "=========================================="
cd ~/ros2_ws

echo "--- Launch file (scan_throttle_node) ---"
echo "SOURCE:"
grep -A 3 "scan_throttle_node" src/rpi-1/src/mks_motor_control/launch/SLAM-on-the-fly.py 2>/dev/null || echo "NIE ZNALEZIONO w source!"
echo ""
echo "INSTALL:"
grep -A 3 "scan_throttle_node" install/mks_motor_control/share/mks_motor_control/launch/SLAM-on-the-fly.py 2>/dev/null || echo "NIE ZNALEZIONO w install!"
echo ""

echo "--- Config file (scan_topic) ---"
echo "SOURCE:"
grep "scan_topic:" src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml
echo ""
echo "INSTALL:"
grep "scan_topic:" install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml
echo ""

echo "--- Config file (scan_buffer_size) ---"
echo "SOURCE:"
grep "scan_buffer_size:" src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml
echo ""
echo "INSTALL:"
grep "scan_buffer_size:" install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml
echo ""

echo "=========================================="
echo "3. TEST: Sprawdź parametry SLAM Toolbox (wymaga działającego launch)"
echo "=========================================="
echo "Uruchom launch w osobnym terminalu, a następnie w tym terminalu:"
echo ""
echo "ros2 param get /slam_toolbox scan_topic"
echo "ros2 param get /slam_toolbox scan_buffer_size"
echo "ros2 param get /slam_toolbox transform_timeout"
echo "ros2 param get /slam_toolbox tf_buffer_duration"
echo ""
echo "ros2 node info /slam_toolbox | grep -A 10 Subscribers"
echo ""
echo "ros2 topic info /scan_throttled"
echo "ros2 topic hz /scan_throttled"
echo ""

echo "=========================================="
echo "4. TEST: Sprawdź TF tree (wymaga działającego launch)"
echo "=========================================="
echo "ros2 run tf2_ros tf2_echo map base_link"
echo "ros2 run tf2_ros tf2_echo odom base_link"
echo "ros2 run tf2_ros tf2_echo base_link unilidar_lidar"
echo ""

echo "=========================================="
echo "5. SUGESTIE"
echo "=========================================="
echo "Jeśli pliki w install/ różnią się od source/:"
echo "  rm -rf build/mks_motor_control install/mks_motor_control log/mks_motor_control"
echo "  cd ~/ros2_ws/src/rpi-1"
echo "  git pull origin main"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select mks_motor_control"
echo "  source install/setup.bash"
echo ""
