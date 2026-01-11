# Opcja 1: Zmiana config file na `/scan` (zgodnie z uproszczonym launch file)

## Problem
- Launch file (`SLAM-on-the-fly.py`) nie ma `scan_throttle` node (uproszczony)
- Config file (`slam_toolbox.yaml`) ma `scan_topic: /scan_throttled`
- SLAM Toolbox subskrybuje do `/scan_throttled`, ale nie ma publishera → SLAM nie otrzymuje skanów

## Rozwiązanie
Zmienić config file na `scan_topic: /scan` aby pasował do uproszczonego launch file.

## Kroki do wykonania na Ubuntu:

### 1. Zmień config file
```bash
cd ~/ros2_ws/src/rpi-1
nano src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml
```

Znajdź linię:
```yaml
    scan_topic: /scan_throttled  # Używa throttled topic (5 Hz) zamiast /scan (72 Hz)
```

Zmień na:
```yaml
    scan_topic: /scan  # Zgodnie z uproszczonym launch file (bez scan_throttle node)
```

Zapisz (Ctrl+X, Y, Enter)

### 2. Zweryfikuj zmianę
```bash
cat src/mks_motor_control/mks_motor_control/config/slam_toolbox.yaml | grep scan_topic
# Powinno pokazać: scan_topic: /scan
```

### 3. Przebuduj pakiet
```bash
cd ~/ros2_ws
rm -rf build/mks_motor_control install/mks_motor_control log/mks_motor_control
colcon build --packages-select mks_motor_control
source install/setup.bash
```

### 4. Zweryfikuj zainstalowany config
```bash
cat install/mks_motor_control/share/mks_motor_control/config/slam_toolbox.yaml | grep scan_topic
# Powinno pokazać: scan_topic: /scan
```

### 5. Przetestuj
```bash
# Terminal 1: Uruchom launch
ros2 launch mks_motor_control SLAM-on-the-fly.py

# Terminal 2: Sprawdź czy SLAM subskrybuje do /scan
ros2 topic info /scan
# Powinno pokazać: Subscription count: 1 (slam_toolbox)

ros2 node info /slam_toolbox | grep -A 5 "Subscribers"
# Powinno pokazać: /scan: sensor_msgs/msg/LaserScan (nie /scan_throttled)
```

## Oczekiwany rezultat:
- ✅ SLAM Toolbox subskrybuje do `/scan`
- ✅ `/scan` ma publisher (pointcloud_to_laserscan) i subscriber (slam_toolbox)
- ✅ SLAM otrzymuje skany i może tworzyć mapę
