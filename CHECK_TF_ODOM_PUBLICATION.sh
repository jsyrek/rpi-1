#!/bin/bash
# Sprawdź czy TF odom->base_link jest publikowany regularnie

echo "=========================================="
echo "SPRAWDZENIE: TF odom->base_link publikacja"
echo "=========================================="
echo ""

echo "1. SPRAWDŹ CZY /tf TOPIC PUBLIKUJE odom->base_link"
echo "---"
timeout 5 ros2 topic echo /tf --once 2>&1 | grep -A 10 "odom" | head -15 || echo "BŁĄD: Nie można odczytać /tf"
echo ""

echo "2. SPRAWDŹ CZĘSTOTLIWOŚĆ PUBLIKACJI /tf"
echo "---"
timeout 3 ros2 topic hz /tf 2>&1 | head -5 || echo "BŁĄD: Nie można odczytać częstotliwości"
echo ""

echo "3. SPRAWDŹ CZY FRAME 'odom' JEST W TF TREE"
echo "---"
ros2 run tf2_ros tf2_monitor 2>&1 | grep -i "odom" || echo "BRAK: frame 'odom' nie istnieje w TF tree"
echo ""

echo "4. PRÓBA ODCZYTANIA TF odom->base_link (timeout 5s)"
echo "---"
timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -20 || echo "BŁĄD: Nie można odczytać TF odom->base_link"
echo ""

echo "5. SPRAWDŹ CZY MOTOR_DRIVER_SPEED NODE DZIAŁA"
echo "---"
ros2 node list | grep motor_driver_speed || echo "BRAK: motor_driver_speed node nie działa"
echo ""

echo "6. SPRAWDŹ LOGI MOTOR_DRIVER_SPEED (szukaj 'Brak danych z enkodera CAN')"
echo "---"
echo "Uruchom w innym terminalu:"
echo "  ros2 topic echo /rosout | grep -i 'motor_driver_speed' | grep -i 'brak danych'"
echo ""

echo "=========================================="
echo "DIAGNOZA:"
echo "- Jeśli TF odom->base_link nie jest publikowany → motor_driver nie może odczytać enkoderów CAN"
echo "- Jeśli TF jest publikowany, ale frame 'odom' nie istnieje w TF tree → problem z TF2 buffer"
echo "- Jeśli tf2_echo nie może odczytać TF → TF nie jest dostępny dla innych nodów"
echo "=========================================="
