#!/bin/bash
# Weryfikacja czy colcon build poprawnie zainstalował pliki config

echo "=========================================="
echo "WERYFIKACJA: colcon build instalacja plików config"
echo "=========================================="
echo ""

echo "1. SPRAWDŹ CZY PLIKI W INSTALL/ SĄ AKTUALNE"
echo "---"

# Sprawdź slam_toolbox.yaml
echo "slam_toolbox.yaml - scan_topic:"
echo "SOURCE:"
grep "scan_topic:" ~/ros2_ws/src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml 2>/dev/null || echo "BŁĄD: Nie można odczytać źródła"
echo ""
echo "INSTALL:"
grep "scan_topic:" ~/ros2_ws/install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml 2>/dev/null || echo "BŁĄD: Nie można odczytać install"
echo ""

# Sprawdź nav2_params.yaml - collision_monitor polygons
echo "nav2_params.yaml - collision_monitor polygons:"
echo "SOURCE:"
grep -A 2 "polygons:" ~/ros2_ws/src/rpi-1/src/mks_motor_control/mks_motor_control/config/nav2_params.yaml 2>/dev/null | head -3 || echo "BŁĄD: Nie można odczytać źródła"
echo ""
echo "INSTALL:"
grep -A 2 "polygons:" ~/ros2_ws/install/mks_motor_control/share/mks_motor_control/config/nav2_params.yaml 2>/dev/null | head -3 || echo "BŁĄD: Nie można odczytać install"
echo ""

echo "2. PORÓWNANIE TIMESTAMPÓW PLIKÓW"
echo "---"
echo "SOURCE slam_toolbox.yaml:"
ls -lh ~/ros2_ws/src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml 2>/dev/null || echo "BŁĄD"
echo ""
echo "INSTALL slam_toolbox.yaml:"
ls -lh ~/ros2_ws/install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml 2>/dev/null || echo "BŁĄD"
echo ""

echo "3. SPRAWDŹ CZY PLIKI SĄ IDENTYCZNE (diff)"
echo "---"
diff ~/ros2_ws/src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml ~/ros2_ws/install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml 2>&1 | head -10 || echo "Pliki są identyczne lub jeden nie istnieje"
echo ""

echo "=========================================="
echo "JEŚLI PLIKI RÓŻNIĄ SIĘ:"
echo "1. Usuń build i install: rm -rf ~/ros2_ws/build/mks_motor_control ~/ros2_ws/install/mks_motor_control"
echo "2. Przebuduj: cd ~/ros2_ws && colcon build --packages-select mks_motor_control"
echo "3. Uruchom ponownie ten skrypt"
echo "=========================================="
