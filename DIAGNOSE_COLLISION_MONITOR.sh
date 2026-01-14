#!/bin/bash
# Skrypt diagnostyczny dla problemu collision_monitor w Nav2 Jazzy

echo "=========================================="
echo "DIAGNOSTYKA COLLISION_MONITOR"
echo "=========================================="
echo ""

# Test 1: Sprawdź źródło kodu Nav2 navigation_launch.py
echo "=== TEST 1: Sprawdzanie navigation_launch.py ==="
NAV2_LAUNCH=$(find /opt/ros/jazzy -name "navigation_launch.py" 2>/dev/null | head -1)
if [ -n "$NAV2_LAUNCH" ]; then
    echo "Znaleziono: $NAV2_LAUNCH"
    echo ""
    echo "--- Fragmenty związane z collision_monitor ---"
    grep -n -i "collision\|use_collision" "$NAV2_LAUNCH" | head -20
    echo ""
    echo "--- Sprawdzanie IfCondition dla use_collision_monitor ---"
    grep -A 10 -B 5 "use_collision_monitor" "$NAV2_LAUNCH" | head -30
else
    echo "Nie znaleziono navigation_launch.py"
fi
echo ""

# Test 2: Sprawdź, czy use_collision_monitor jest obsługiwane
echo "=== TEST 2: Sprawdzanie obsługi use_collision_monitor ==="
if [ -n "$NAV2_LAUNCH" ]; then
    if grep -q "use_collision_monitor" "$NAV2_LAUNCH"; then
        echo "✓ use_collision_monitor jest obsługiwane w navigation_launch.py"
    else
        echo "✗ use_collision_monitor NIE jest obsługiwane w navigation_launch.py"
        echo "  To może być przyczyna problemu!"
    fi
else
    echo "Nie można sprawdzić (brak pliku)"
fi
echo ""

# Test 3: Sprawdź parametry collision_monitor (wymaga uruchomionego systemu)
echo "=== TEST 3: Sprawdzanie parametrów collision_monitor ==="
echo "UWAGA: Ten test wymaga uruchomionego systemu Nav2"
if ros2 node list 2>/dev/null | grep -q "collision_monitor"; then
    echo "Węzeł collision_monitor jest aktywny"
    echo ""
    echo "--- Lista parametrów ---"
    ros2 param list /collision_monitor 2>/dev/null | head -20
    echo ""
    echo "--- Wartość 'enabled' ---"
    ros2 param get /collision_monitor enabled 2>/dev/null || echo "Nie można odczytać"
    echo ""
    echo "--- Wartość 'observation_sources' ---"
    ros2 param get /collision_monitor observation_sources 2>/dev/null || echo "Nie można odczytać"
else
    echo "Węzeł collision_monitor nie jest aktywny (system nie uruchomiony lub węzeł nie istnieje)"
fi
echo ""

# Test 4: Sprawdź lifecycle_manager_navigation
echo "=== TEST 4: Sprawdzanie lifecycle_manager_navigation ==="
if ros2 node list 2>/dev/null | grep -q "lifecycle_manager_navigation"; then
    echo "Węzeł lifecycle_manager_navigation jest aktywny"
    echo ""
    echo "--- Lista węzłów zarządzanych przez lifecycle_manager ---"
    ros2 param get /lifecycle_manager_navigation node_names 2>/dev/null || echo "Nie można odczytać"
    echo ""
    if ros2 param get /lifecycle_manager_navigation node_names 2>/dev/null | grep -q "collision_monitor"; then
        echo "⚠️  collision_monitor JEST na liście węzłów lifecycle_manager!"
        echo "   To może być przyczyna - węzeł jest wymuszany przez lifecycle_manager"
    else
        echo "✓ collision_monitor NIE jest na liście węzłów lifecycle_manager"
    fi
else
    echo "Węzeł lifecycle_manager_navigation nie jest aktywny"
fi
echo ""

# Test 5: Sprawdź pliki tymczasowe z parametrami
echo "=== TEST 5: Sprawdzanie plików tymczasowych z parametrami ==="
TMP_FILES=$(ls -t /tmp/launch_params_* 2>/dev/null | head -5)
if [ -n "$TMP_FILES" ]; then
    echo "Znalezione pliki tymczasowe:"
    for file in $TMP_FILES; do
        echo "  - $file"
        if grep -q "collision_monitor" "$file" 2>/dev/null; then
            echo "    Zawiera sekcję collision_monitor:"
            grep -A 10 "collision_monitor" "$file" | head -15
        fi
    done
else
    echo "Nie znaleziono plików tymczasowych (system nie był uruchomiony)"
fi
echo ""

# Test 6: Sprawdź konfigurację w nav2_params.yaml
echo "=== TEST 6: Sprawdzanie konfiguracji w nav2_params.yaml ==="
NAV2_PARAMS="src/mks_motor_control/mks_motor_control/config/nav2_params.yaml"
if [ -f "$NAV2_PARAMS" ]; then
    echo "Plik: $NAV2_PARAMS"
    echo ""
    echo "--- Sekcja collision_monitor ---"
    if grep -q "collision_monitor:" "$NAV2_PARAMS"; then
        grep -A 15 "collision_monitor:" "$NAV2_PARAMS" | head -20
    else
        echo "Brak sekcji collision_monitor w pliku"
    fi
else
    echo "Nie znaleziono pliku nav2_params.yaml"
fi
echo ""

# Test 7: Sprawdź launch file
echo "=== TEST 7: Sprawdzanie launch file ==="
LAUNCH_FILE="src/mks_motor_control/launch/SLAM-on-the-fly.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "Plik: $LAUNCH_FILE"
    echo ""
    echo "--- Użycie use_collision_monitor ---"
    grep -n -A 5 -B 5 "use_collision_monitor" "$LAUNCH_FILE" || echo "Nie znaleziono use_collision_monitor"
else
    echo "Nie znaleziono launch file"
fi
echo ""

echo "=========================================="
echo "PODSUMOWANIE"
echo "=========================================="
echo ""
echo "Najważniejsze pytania:"
echo "1. Czy use_collision_monitor jest obsługiwane w Nav2 Jazzy? (Test 1, 2)"
echo "2. Czy collision_monitor jest na liście lifecycle_manager? (Test 4)"
echo "3. Jakie parametry są przekazywane do collision_monitor? (Test 3, 5)"
echo ""
echo "Jeśli use_collision_monitor NIE jest obsługiwane, trzeba:"
echo "- Ręcznie usunąć collision_monitor z navigation_launch.py (lub użyć własnego launch file)"
echo "- Usunąć collision_monitor z listy węzłów w lifecycle_manager"
echo ""
