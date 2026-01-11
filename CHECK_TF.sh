#!/bin/bash
# Sprawdź co publikuje TF

echo "=========================================="
echo "1. LISTA WSZYSTKICH FRAME'ÓW W TF TREE"
echo "=========================================="
ros2 run tf2_ros tf2_monitor 2>/dev/null | head -20 || echo "BŁĄD: Nie można odczytać TF tree"
echo ""

echo "=========================================="
echo "2. SPRAWDŹ CZY NODY SĄ AKTYWNE"
echo "=========================================="
echo "robot_state_publisher:"
ros2 node list | grep robot_state_publisher || echo "BRAK!"
echo ""
echo "motor_driver_speed:"
ros2 node list | grep motor_driver_speed || echo "BRAK!"
echo ""
echo "base_to_unilidar_lidar (static_transform_publisher):"
ros2 node list | grep base_to_unilidar_lidar || echo "BRAK!"
echo ""

echo "=========================================="
echo "3. SPRAWDŹ TF STATIC (powinien być base_link->unilidar_lidar)"
echo "=========================================="
ros2 topic echo /tf_static --once 2>/dev/null | head -30 || echo "BŁĄD: Nie można odczytać /tf_static"
echo ""

echo "=========================================="
echo "4. SPRAWDŹ TF DYNAMIC (powinien być odom->base_link)"
echo "=========================================="
timeout 2 ros2 topic echo /tf --once 2>/dev/null | head -30 || echo "BŁĄD: Nie można odczytać /tf"
echo ""

echo "=========================================="
echo "5. SPRAWDŹ ODOM TOPIC (motor_driver publikuje TF z /odom)"
echo "=========================================="
timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -E "(frame_id|child_frame_id)" || echo "BŁĄD: Nie można odczytać /odom"
echo ""
