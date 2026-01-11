#!/bin/bash
# Sprawdź czy motor_driver_speed publikuje TF odom->base_link

echo "=========================================="
echo "SPRAWDZENIE: motor_driver_speed TF publikacja"
echo "=========================================="
echo ""

echo "1. SPRAWDŹ LOGI MOTOR_DRIVER_SPEED (szukaj 'Brak danych z enkodera CAN')"
echo "---"
echo "Uruchom w innym terminalu i szukaj w logach:"
echo "  ros2 topic echo /rosout | grep -i 'motor_driver_speed' | grep -i 'brak danych'"
echo ""

echo "2. SPRAWDŹ CZY /odom JEST PUBLIKOWANY"
echo "---"
timeout 2 ros2 topic echo /odom --once 2>&1 | head -20 || echo "BŁĄD: /odom nie jest publikowany"
echo ""

echo "3. SPRAWDŹ TF /tf TOPIC (czy odom->base_link jest publikowany)"
echo "---"
timeout 3 ros2 topic echo /tf --once 2>&1 | grep -A 15 "odom" || echo "BŁĄD: odom->base_link nie jest w /tf"
echo ""

echo "4. SPRAWDŹ CZY FRAME 'odom' JEST W TF TREE"
echo "---"
ros2 run tf2_ros tf2_monitor 2>&1 | grep -i "odom" || echo "BRAK: frame 'odom' nie istnieje w TF tree"
echo ""

echo "5. PRÓBA ODCZYTANIA TF odom->base_link (timeout 3s)"
echo "---"
timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -15 || echo "BŁĄD: Nie można odczytać TF odom->base_link"
echo ""

echo "=========================================="
echo "DIAGNOZA:"
echo "- Jeśli 'Brak danych z enkodera CAN' → motor_driver nie może odczytać enkoderów CAN"
echo "- Jeśli /odom jest publikowany, ale TF nie → problem z tf_broadcaster.sendTransform()"
echo "- Jeśli odom frame nie istnieje w TF tree → TF nie jest publikowany"
echo "=========================================="
