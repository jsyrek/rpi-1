# INSTRUKCJA: Setup Motor Driver dla RPi 4 (z Systemd CAN)

## 🎯 Cel
Skonfigurować robota na RPi 4 tak żeby:
1. ✅ CAN interface inicjalizuje się **automatycznie na boot**
2. ✅ Motor Driver startuje **bez problemu z sudo**
3. ✅ Wszystko działa **bez ręcznych komend**

---

## 📋 KROK 1: Setup Systemd Service (jednorazowo)

### 1.1. Otwórz edytor
```bash
sudo nano /etc/systemd/system/can-init.service
```

### 1.2. Skopiuj zawartość
```ini
[Unit]
Description=Initialize CAN interface for MKS Motor Control
Before=multi-user.target
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/bin/bash -c 'ip link set can0 type can bitrate 500000 && ip link set can0 up'
ExecStop=/usr/bin/bash -c 'ip link set can0 down || true'
User=root
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### 1.3. Zapisz
- Ctrl+X
- Y (yes)
- Enter

### 1.4. Aktywuj service
```bash
sudo systemctl daemon-reload
sudo systemctl enable can-init.service
sudo systemctl start can-init.service
```

### 1.5. Sprawdzenie
```bash
sudo systemctl status can-init.service
```

**Oczekiwany wynik:**
```
● can-init.service - Initialize CAN interface for MKS Motor Control
   Loaded: loaded (.../can-init.service; enabled; vendor preset: enabled)
   Active: active (exited) since ...
   
   Process: 1234 ExecStart=...
```

---

## 📦 KROK 2: Build ROS2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select mks_motor_control
```

**Oczekiwany wynik:**
```
Summary: 1 package built successfully
```

---

## 🔧 KROK 3: Source Environment

```bash
source install/setup.bash
```

**Lub dodaj do ~/.bashrc (aby auto-source):**
```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
exec bash
```

---

## 🚀 KROK 4: Uruchomienie Motor Driver

### 4.1. Pierwsza próba
```bash
ros2 launch mks_motor_control motor_driver_complete.launch.py
```

**Oczekiwany output:**
```
======================================================================
MOTOR_DRIVER_SPEED LAUNCH FILE
Optimized dla: Raspberry Pi 4 + Systemd CAN Service
======================================================================

[1/4] Sprawdzam CAN interface...
1: lo: <LOOPBACK,UP,LOWER_UP>
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP>
3: can0: <NOARP,UP,RUNNING> mtu 16
    link/can

[2/4] Uruchamiam Robot State Publisher...
[3/4] Uruchamiam Motor Driver Speed...

======================================================================
✅ Motor Driver Speed uruchomiony!
======================================================================

Dostępne topics:
  /cmd_vel - wejście: komendy prędkości
  /odom - wyjście: odometria
  /joint_states - wyjście: stany kół
```

---

## 🧪 KROK 5: Testowanie

### 5.1. Sprawdzenie topików (nowy terminal)

```bash
source install/setup.bash
ros2 topic list
```

Powinna być:
```
/cmd_vel
/joint_states
/odom
/tf
/tf_static
```

### 5.2. Monitorowanie odometrii
```bash
ros2 topic echo /odom -n 1
```

### 5.3. Test jazdy na wprost
```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## ✅ CHECKLIST WERYFIKACJI

- [ ] `sudo systemctl status can-init.service` → active (exited)
- [ ] `ip link show can0` → UP flag widoczna
- [ ] `colcon build` → успешно
- [ ] `ros2 launch ...` → bez błędów
- [ ] `ros2 topic list` → widoczne /cmd_vel, /odom
- [ ] Robot reaguje na `ros2 topic pub /cmd_vel ...`

---

## 🔴 TROUBLESHOOTING

### Problem: "can-init.service" nie uruchamia się

```bash
# Sprawdzenie błędu
sudo systemctl status can-init.service

# Jeśli show error:
sudo journalctl -u can-init.service -n 20
```

**Rozwiązanie:**
- Sprawdź czy `/bin/bash` istnieje (może być `/usr/bin/bash`)
- Sprawdź czy `ip` command jest dostępny

---

### Problem: "ros2 launch" prosi o hasło

```
[sudo] password for jarek:
```

**Rozwiązanie:**
- Systemd service powinno obsługiwać inicjalizację
- Sprawdź czy service jest active: `sudo systemctl status can-init.service`

---

### Problem: CAN0 nie jest UP

```bash
# Sprawdzenie
ip link show can0

# Jeśli DOWN:
sudo ip link set can0 up
```

---

## 📝 AUTOMATYCZNE URUCHAMIANIE (opcjonalnie)

Jeśli chcesz żeby Motor Driver startował automatycznie na boot:

### Stwórz systemd service dla ROS2

```bash
sudo nano /etc/systemd/system/motor-driver.service
```

**Zawartość:**
```ini
[Unit]
Description=ROS2 Motor Driver Speed
After=can-init.service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/ros2_ws
ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch mks_motor_control motor_driver_complete.launch.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Aktywuj:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable motor-driver.service
sudo systemctl start motor-driver.service
```

**Sprawdzenie:**
```bash
sudo systemctl status motor-driver.service
```

---

## 🎯 PODSUMOWANIE

### Dla RPi 4 z tym setupem:

| Etap | Kiedy | Komenda | Automatycznie |
|------|-------|---------|---|
| **CAN Init** | Boot | Systemd service | ✅ TAK |
| **ROS2 Build** | Przed uruchomieniem | `colcon build` | ❌ Raz |
| **Environment** | Każdy terminal | `source install/setup.bash` | ✅ TAK (jeśli w ~/.bashrc) |
| **Launch** | Użytkownik | `ros2 launch ...` | ❌ Ręcznie (lub systemd service) |

---

## 🚀 SZYBKI START (po setup)

```bash
# Terminal 1: Uruchom Motor Driver
ros2 launch mks_motor_control motor_driver_complete.launch.py

# Terminal 2: Test
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Terminal 3: Monitor
ros2 topic echo /odom
```

---

## 💡 PRO TIPS

1. **Dodaj do ~/.bashrc:**
   ```bash
   echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
   ```
   Wtedy nie trzeba sourcować na każdy terminal.

2. **Skrót do launch:**
   ```bash
   alias motor-start='ros2 launch mks_motor_control motor_driver_complete.launch.py'
   ```
   Potem: `motor-start`

3. **Monitoring w tle:**
   ```bash
   ros2 topic echo /odom > odom.log &
   ```

4. **Debug mode:**
   ```bash
   ros2 launch mks_motor_control motor_driver_complete.launch.py --log-level DEBUG
   ```

---

**Powodzenia!** 🤖✨

Jeśli coś nie działa - zwróć się z błędem, pokaż `sudo systemctl status can-init.service` i logi z launch file!
