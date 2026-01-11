# Weryfikacja instalacji - Sprawdź czy zmiany trafiają do install/

## Problem
Zmiany w źródłowych plikach mogą nie trafiać do `install/`, przez co colcon używa starych plików.

## Test 1: Sprawdź launch file

```bash
cd ~/ros2_ws

# Sprawdź źródłowy launch file
echo "=== SOURCE launch file ==="
grep -A 5 "scan_throttle_node" src/rpi-1/src/mks_motor_control/launch/SLAM-on-the-fly.py

# Sprawdź zainstalowany launch file
echo "=== INSTALLED launch file ==="
grep -A 5 "scan_throttle_node" install/mks_motor_control/share/mks_motor_control/launch/SLAM-on-the-fly.py
```

**Oczekiwany wynik:** Oba powinny pokazać `scan_throttle_node`.

## Test 2: Sprawdź config file

```bash
# Sprawdź źródłowy config
echo "=== SOURCE config ==="
grep "scan_topic:" src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml

# Sprawdź zainstalowany config
echo "=== INSTALLED config ==="
grep "scan_topic:" install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml
```

**Oczekiwany wynik:** Oba powinny pokazać `scan_topic: /scan_throttled`.

## Test 3: Sprawdź parametry SLAM Toolbox

```bash
# Sprawdź scan_buffer_size w źródłowym
echo "=== SOURCE scan_buffer_size ==="
grep "scan_buffer_size:" src/rpi-1/src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml

# Sprawdź scan_buffer_size w zainstalowanym
echo "=== INSTALLED scan_buffer_size ==="
grep "scan_buffer_size:" install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml
```

**Oczekiwany wynik:** Oba powinny pokazać `scan_buffer_size: 5000`.

## Jeśli pliki są różne:

1. **Usuń build i install:**
```bash
cd ~/ros2_ws
rm -rf build/mks_motor_control install/mks_motor_control log/mks_motor_control
```

2. **Sprawdź git status:**
```bash
cd ~/ros2_ws/src/rpi-1
git status
git log --oneline -5
```

3. **Upewnij się, że masz najnowsze zmiany:**
```bash
git pull origin main
```

4. **Przebuduj:**
```bash
cd ~/ros2_ws
colcon build --packages-select mks_motor_control
source install/setup.bash
```

5. **Ponownie zweryfikuj** używając testów powyżej.
