#!/bin/bash
# Kompleksowa diagnostyka "Message Filter dropping message" błędu
# Sprawdza wszystkie komponenty: topiki, TF, parametry, subskrypcje

echo "=========================================="
echo "DIAGNOZA: Message Filter dropping message"
echo "=========================================="
echo ""

echo "=========================================="
echo "1. CZĘSTOTLIWOŚĆ TOPICÓW"
echo "=========================================="
echo "--- /scan (powinno być ~72 Hz) ---"
timeout 3 ros2 topic hz /scan 2>&1 | head -3 || echo "BŁĄD: Nie można odczytać częstotliwości /scan"
echo ""

echo "--- /scan_throttled (powinno być ~5 Hz) ---"
timeout 3 ros2 topic hz /scan_throttled 2>&1 | head -3 || echo "BŁĄD: Nie można odczytać częstotliwości /scan_throttled"
echo ""

echo "--- /odom (powinno być ~50 Hz) ---"
timeout 3 ros2 topic hz /odom 2>&1 | head -3 || echo "BŁĄD: Nie można odczytać częstotliwości /odom"
echo ""

echo "=========================================="
echo "2. INFO O TOPICACH"
echo "=========================================="
echo "--- /scan ---"
ros2 topic info /scan 2>&1 | head -8 || echo "BŁĄD: Topic /scan nie istnieje"
echo ""

echo "--- /scan_throttled ---"
ros2 topic info /scan_throttled 2>&1 | head -8 || echo "BŁĄD: Topic /scan_throttled nie istnieje"
echo ""

echo "=========================================="
echo "3. PARAMETRY SLAM TOOLBOX"
echo "=========================================="
echo "scan_topic:"
ros2 param get /slam_toolbox scan_topic 2>/dev/null || echo "BŁĄD: Nie można odczytać parametru scan_topic"
echo ""

echo "scan_buffer_size:"
ros2 param get /slam_toolbox scan_buffer_size 2>/dev/null || echo "BŁĄD: Nie można odczytać parametru scan_buffer_size"
echo ""

echo "transform_timeout:"
ros2 param get /slam_toolbox transform_timeout 2>/dev/null || echo "BŁĄD: Nie można odczytać parametru transform_timeout"
echo ""

echo "tf_buffer_duration:"
ros2 param get /slam_toolbox tf_buffer_duration 2>/dev/null || echo "BŁĄD: Nie można odczytać parametru tf_buffer_duration"
echo ""

echo "=========================================="
echo "4. SUBSCRIPTIONS SLAM TOOLBOX"
echo "=========================================="
ros2 node info /slam_toolbox 2>/dev/null | grep -A 15 "Subscribers:" || echo "BŁĄD: Nie można odczytać info o nodzie /slam_toolbox"
echo ""

echo "=========================================="
echo "5. TF TREE - LISTA FRAME'ÓW"
echo "=========================================="
ros2 run tf2_ros tf2_monitor 2>&1 | head -25 || echo "BŁĄD: Nie można odczytać TF tree"
echo ""

echo "=========================================="
echo "6. TF TRANSFORMACJE (timeout 2s)"
echo "=========================================="
echo "--- odom -> base_link ---"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10 || echo "BRAK transformacji odom->base_link (lub timeout)"
echo ""

echo "--- base_link -> unilidar_lidar ---"
timeout 2 ros2 run tf2_ros tf2_echo base_link unilidar_lidar 2>&1 | head -10 || echo "BRAK transformacji base_link->unilidar_lidar (lub timeout)"
echo ""

echo "--- map -> base_link ---"
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -10 || echo "BRAK transformacji map->base_link (lub timeout - normalne na początku)"
echo ""

echo "=========================================="
echo "7. SPRAWDŹ NODY"
echo "=========================================="
echo "scan_throttle node:"
ros2 node list | grep scan_throttle || echo "BRAK! scan_throttle node nie działa"
echo ""

echo "slam_toolbox node:"
ros2 node list | grep slam_toolbox || echo "BRAK! slam_toolbox node nie działa"
echo ""

echo "motor_driver_speed node:"
ros2 node list | grep motor_driver_speed || echo "BRAK! motor_driver_speed node nie działa"
echo ""

echo "=========================================="
echo "8. OSTATNIE WIADOMOŚCI Z TOPICÓW (frame_id)"
echo "=========================================="
echo "--- /scan (ostatnia wiadomość) ---"
timeout 1 ros2 topic echo /scan --once 2>/dev/null | grep -E "(frame_id|header)" | head -3 || echo "BŁĄD: Nie można odczytać /scan"
echo ""

echo "--- /scan_throttled (ostatnia wiadomość) ---"
timeout 1 ros2 topic echo /scan_throttled --once 2>/dev/null | grep -E "(frame_id|header)" | head -3 || echo "BŁĄD: Nie można odczytać /scan_throttled"
echo ""

echo "=========================================="
echo "KONIEC DIAGNOZY"
echo "=========================================="
