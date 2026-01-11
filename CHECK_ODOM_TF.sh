#!/bin/bash
# Sprawdź czy odom -> base_link TF jest publikowany

echo "=========================================="
echo "1. SPRAWDŹ CZY ODOM FRAME ISTNIEJE W TF TREE"
echo "=========================================="
timeout 5 ros2 run tf2_ros tf2_monitor 2>/dev/null | grep -i "odom\|base_link" || echo "BŁĄD lub brak odom frame"
echo ""

echo "=========================================="
echo "2. SPRAWDŹ TF bezpośrednio (10 próbek)"
echo "=========================================="
timeout 3 ros2 topic echo /tf --once 2>/dev/null | grep -A 10 "child_frame_id.*base_link" | head -20 || echo "BŁĄD: Nie można odczytać /tf"
echo ""

echo "=========================================="
echo "3. SPRAWDŹ ODOM -> BASE_LINK TRANSFORM"
echo "=========================================="
timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -10 || echo "BŁĄD: Transformacja odom->base_link nie istnieje"
echo ""

echo "=========================================="
echo "4. SPRAWDŹ CZY MOTOR_DRIVER PUBLIKUJE /odom"
echo "=========================================="
timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -E "(frame_id|child_frame_id|header)" | head -5 || echo "BŁĄD: Nie można odczytać /odom"
echo ""

echo "=========================================="
echo "5. SPRAWDŹ LOGI MOTOR_DRIVER (jeśli widoczne)"
echo "=========================================="
echo "Sprawdź w terminalu z launch czy są błędy z motor_driver_speed"
echo ""
