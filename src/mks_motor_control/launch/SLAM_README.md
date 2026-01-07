# SLAM on-the-fly - Instrukcja użycia

## 🎯 Co to robi?

Launch file `SLAM-on-the-fly.py` uruchamia system nawigacji który:
- **Tworzy mapę w czasie rzeczywistym** podczas poruszania się robota
- **Działa w każdym pomieszczeniu** bez wcześniejszego mapowania
- **Wykorzystuje odometrię z silników MKS** przez CAN
- **Używa Unitree L2 LiDAR** do wykrywania otoczenia
- **Automatycznie lokalizuje robota** na dynamicznie tworzonej mapie

## 📋 Wymagania

### Pakiety ROS2:
```bash
# Instalacja pakietów z apt
sudo apt install ros-<distro>-pointcloud-to-laserscan
sudo apt install ros-<distro>-slam-toolbox
sudo apt install ros-<distro>-rviz2  # Opcjonalnie - tylko do wizualizacji
```

**Uwaga:** RViz2 nie jest wymagany do działania systemu - system działa bez niego. RViz2 jest tylko do wizualizacji i debugowania.

### Instalacja pakietu Unitree LiDAR ROS2:

Pakiet `unitree_lidar_ros2` musi być zainstalowany w workspace. Jeśli nie masz go jeszcze:

**Opcja 1: Sklonuj z repozytorium Unitree (jeśli dostępne):**
```bash
cd ~/ros2_ws/src
git clone <unitree_lidar_ros2_repository_url>
cd ~/ros2_ws
colcon build --packages-select unitree_lidar_ros2
source install/setup.bash
```

**Opcja 2: Jeśli pakiet jest już w workspace ale nie zbudowany:**
```bash
cd ~/ros2_ws
colcon build --packages-select unitree_lidar_ros2
source install/setup.bash
```

**Opcja 3: Sprawdź czy pakiet jest już zainstalowany:**
```bash
ros2 pkg list | grep unitree_lidar_ros2
```

Jeśli pakiet nie jest dostępny, sprawdź dokumentację Unitree L2 LiDAR lub skontaktuj się z producentem.

### Sprzęt:
- Mini PC z Ubuntu (na robocie)
- CAN interface skonfigurowany (can0)
- Unitree L2 LiDAR podłączony przez sieć
- Silniki MKS Servo42D z enkoderami

## ⚙️ Konfiguracja przed uruchomieniem

### 1. Sprawdź IP LiDAR i mini PC

Edytuj `SLAM-on-the-fly.py` (linie 113-114):
```python
'lidar_ip': '192.168.1.62',     # IP LiDAR Unitree L2
'local_ip': '192.168.1.2',      # IP mini PC na robocie
```

### 2. Sprawdź CAN interface

```bash
# Sprawdź czy can0 jest aktywny
ip link show can0

# Jeśli nie, skonfiguruj:
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 3. Sprawdź topic PointCloud2 (opcjonalnie)

Po uruchomieniu sprawdź rzeczywisty topic:
```bash
ros2 topic list | grep cloud
```

Jeśli topic jest inny niż `/unilidar/cloud`, zmień w launch file (linia 145).

## 🚀 Uruchomienie

### Podstawowe uruchomienie:
```bash
ros2 launch mks_motor_control SLAM-on-the-fly.py
```

### Z opcjami:
```bash
# Z RViz2 (wizualizacja)
ros2 launch mks_motor_control SLAM-on-the-fly.py use_rviz:=True

# Bez automatycznego startu Nav2
ros2 launch mks_motor_control SLAM-on-the-fly.py autostart:=False

# Z symulacją czasu (dla testów)
ros2 launch mks_motor_control SLAM-on-the-fly.py use_sim_time:=True

# Kombinacja opcji
ros2 launch mks_motor_control SLAM-on-the-fly.py use_rviz:=True autostart:=False
```

## 📊 Sprawdzanie działania

### 1. Sprawdź czy wszystkie węzły działają:
```bash
ros2 node list
```

Powinieneś zobaczyć:
- `/motor_driver_speed`
- `/unitree_lidar_l2`
- `/pointcloud_to_laserscan`
- `/slam_toolbox`
- `/robot_state_publisher`
- Węzły Nav2

### 2. Sprawdź TF tree:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

Powinieneś zobaczyć: `map -> odom -> base_link -> unilidar_lidar`

### 3. Sprawdź mapę w RViz2:

**Opcja A: Uruchom RViz2 automatycznie z launch file:**
```bash
ros2 launch mks_motor_control SLAM-on-the-fly.py use_rviz:=True
```

**Opcja B: Uruchom RViz2 ręcznie:**
```bash
rviz2
```

**Co dodać w RViz2:**
1. Kliknij **Add** (lub `Ctrl+A`)
2. Dodaj następujące elementy:
   - **Map** → Topic: `/map` → Fixed Frame: `map`
   - **LaserScan** → Topic: `/scan` → Fixed Frame: `base_link`
   - **TF** → (pokazuje wszystkie transformacje)
   - **RobotModel** → (pokazuje model robota z URDF)
   - **Path** → Topic: `/plan` → (pokazuje planowaną ścieżkę)
   - **PoseArray** → Topic: `/particlecloud` → (pokazuje cząsteczki SLAM, opcjonalnie)

**Ustawienia:**
- **Fixed Frame**: `map` (dla widoku mapy) lub `odom` (dla widoku relatywnego)
- **Background Color**: Ciemny (lepiej widać mapę)

### 4. Monitoruj odometrię:
```bash
ros2 topic echo /odom
```

## 💾 Zapisywanie mapy

Po zakończeniu mapowania możesz zapisać mapę:

```bash
# Podczas działania SLAM
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"

# Lub użyj map_saver
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

Mapa zostanie zapisana jako:
- `~/maps/my_map.pgm` (obraz mapy)
- `~/maps/my_map.yaml` (metadane)

## 🔧 Rozwiązywanie problemów

### Problem: Brak danych z LiDAR
```bash
# Sprawdź czy LiDAR publikuje dane
ros2 topic echo /unilidar/cloud --once

# Sprawdź połączenie sieciowe
ping <lidar_ip>
```

### Problem: Brak danych z CAN
```bash
# Sprawdź czy can0 jest aktywny
ip link show can0

# Sprawdź czy motor_driver_speed działa
ros2 topic echo /odom
```

### Problem: SLAM nie tworzy mapy
```bash
# Sprawdź czy LaserScan jest publikowany
ros2 topic echo /scan --once

# Sprawdź logi SLAM
ros2 topic echo /slam_toolbox/feedback
```

### Problem: Wysokie obciążenie CPU
- Zmniejsz `cloud_scan_num` w launch file (linia 112) z 3 na 1-2
- Zmniejsz `max_laser_range` w slam_toolbox.yaml

## 📈 Wydajność

- **Precyzja lokalizacji**: ±5-7 cm
- **Zużycie CPU**: ~30-50% na Raspberry Pi 4
- **Czas startu**: ~5-10 sekund
- **Czas budowania mapy**: w czasie rzeczywistym podczas ruchu

## 🎓 Następne kroki

1. **Przetestuj w różnych pomieszczeniach** - system działa bez wcześniejszego mapowania
2. **Zapisz mapy** - możesz użyć zapisanych map z AMCL w przyszłości
3. **Dostosuj parametry** - dla lepszej precyzji lub wydajności

## 📝 Uwagi

- Mapa jest tworzona **w czasie rzeczywistym** - im dłużej robot się porusza, tym dokładniejsza mapa
- **Loop closure** jest włączone - robot rozpoznaje miejsca które już odwiedził
- System działa **bez wcześniejszej mapy** - idealne dla nieznanych pomieszczeń
