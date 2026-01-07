# Instrukcja: Pobieranie plików z GitHub na Ubuntu

## Jeśli repozytorium już istnieje (klonowane wcześniej)

### Aktualizacja istniejącego repozytorium:
```bash
cd ~/ros2_ws/src/rpi-1  # lub gdzie masz sklonowane repozytorium
git pull origin main
```

## Jeśli repozytorium nie istnieje (pierwsze pobranie)

### Sklonuj całe repozytorium:
```bash
cd ~/ros2_ws/src  # lub gdzie chcesz mieć kod
git clone https://github.com/jsyrek/rpi-1.git
cd rpi-1
```

## Pełna procedura dla ROS2 workspace

### 1. Przejdź do workspace:
```bash
cd ~/ros2_ws/src
```

### 2. Jeśli masz już sklonowane:
```bash
cd rpi-1
git pull origin main
```

### 3. Jeśli nie masz jeszcze sklonowanego:
```bash
git clone https://github.com/jsyrek/rpi-1.git
cd rpi-1
```

### 4. Zbuduj pakiet (jeśli były zmiany w setup.py):
```bash
cd ~/ros2_ws
colcon build --packages-select mks_motor_control
source install/setup.bash
```

## Sprawdzenie czy wszystko jest aktualne

```bash
cd ~/ros2_ws/src/rpi-1
git status
git log --oneline -5
```

Powinieneś zobaczyć najnowsze commity:
- `4306013 Add GitHub push instructions`
- `4576dde Add SLAM on-the-fly and AMCL launch files with LiDAR integration`
- `6489993 Remove odom: file (invalid filename on Windows)`

## Sprawdzenie nowych plików

```bash
ls -la ~/ros2_ws/src/rpi-1/src/mks_motor_control/launch/
```

Powinieneś zobaczyć:
- `AMCL-lidar-odometry-IMU.py`
- `SLAM-on-the-fly.py`
- `SLAM_README.md`

## Uwagi

- Jeśli masz lokalne zmiany, które nie są commitowane, git pull może się nie powieść
- W takim przypadku użyj: `git stash` (zapisz zmiany), potem `git pull`, potem `git stash pop` (przywróć zmiany)
- Jeśli chcesz nadpisać lokalne zmiany: `git reset --hard origin/main` (UWAGA: utracisz lokalne zmiany!)
