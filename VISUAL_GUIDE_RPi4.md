# VISUAL GUIDE: Motor Driver Setup dla RPi 4

## 🔄 Workflow - Krok po kroku

```
┌─────────────────────────────────────────────────────────────┐
│ ZANIM ZACZNIESZ: Sprawdzenie                                │
├─────────────────────────────────────────────────────────────┤
│ ✓ RPi 4 z ROS2 Humble zainstalowany                        │
│ ✓ CAN hardware (USB CAN adapter lub wbudowany)             │
│ ✓ Motor driver package w ~/ros2_ws/src/mks_motor_control   │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ SETUP 1: Systemd Service (jednorazowo)                      │
├─────────────────────────────────────────────────────────────┤
│ 1. sudo nano /etc/systemd/system/can-init.service          │
│ 2. [Dodaj zawartość]                                        │
│ 3. sudo systemctl daemon-reload                            │
│ 4. sudo systemctl enable can-init.service                  │
│ 5. sudo systemctl start can-init.service                   │
│ 6. Sprawdzenie: ip link show can0                          │
│                                                             │
│ Wynik: CAN0 zawsze UP na boot! ✅                          │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ SETUP 2: Build ROS2 Package                                │
├─────────────────────────────────────────────────────────────┤
│ $ cd ~/ros2_ws                                              │
│ $ colcon build --packages-select mks_motor_control        │
│                                                             │
│ Wynik: Pakiet skompilowany, ready to launch! ✅            │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ SETUP 3: Source Environment (każdy terminal)               │
├─────────────────────────────────────────────────────────────┤
│ $ source ~/ros2_ws/install/setup.bash                      │
│                                                             │
│ LUB na stałe w ~/.bashrc:                                  │
│ $ echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc  │
│ $ exec bash                                                 │
│                                                             │
│ Wynik: ROS2 wie gdzie szukać pakietów! ✅                 │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ LAUNCH: Motor Driver                                        │
├─────────────────────────────────────────────────────────────┤
│ $ ros2 launch mks_motor_control \                          │
│   motor_driver_complete.launch.py                          │
│                                                             │
│ [Launch file startuje:]                                    │
│   • Sprawdza CAN (już UP!)                                 │
│   • Uruchamia Robot State Publisher                        │
│   • Uruchamia Motor Driver Speed                           │
│                                                             │
│ Wynik: ✅ ALL SYSTEMS GO!                                 │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│ TESTING: Weryfikacja                                       │
├─────────────────────────────────────────────────────────────┤
│ Terminal 2:                                                 │
│ $ ros2 topic list                                          │
│   /cmd_vel          ← wejście (komendy)                    │
│   /odom             ← wyjście (odometria)                  │
│   /joint_states     ← wyjście (stany kół)                  │
│                                                             │
│ Terminal 3:                                                 │
│ $ ros2 topic pub -1 /cmd_vel \                             │
│   geometry_msgs/msg/Twist "{linear: {x: 0.5}}"             │
│                                                             │
│ Wynik: Robot jeździ! ✅                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 Stan systemu na każdym etapie

```
STAGE 1: Boot
┌──────────────────────────────────────┐
│ RPi 4 uruchamia się                  │
│ ↓                                    │
│ Systemd loaduje can-init.service    │
│ ↓                                    │
│ Wykonuje: ip link set can0 up        │
│ ↓                                    │
│ can0 interface: UP ✅                │
│ (Gotowy do CAN komunikacji)          │
└──────────────────────────────────────┘

STAGE 2: Developers uruchamia launch
┌──────────────────────────────────────┐
│ $ ros2 launch ...                    │
│ ↓                                    │
│ Launch file sprawdza: ip link show   │
│ ↓                                    │
│ can0: UP ✅ (już jest!)              │
│ ↓                                    │
│ Starts: robot_state_publisher        │
│ Starts: motor_driver_speed           │
│ ↓                                    │
│ Topics available: ✅                 │
│ /cmd_vel, /odom, /joint_states       │
└──────────────────────────────────────┘

STAGE 3: User wysyła komendy
┌──────────────────────────────────────┐
│ $ ros2 topic pub /cmd_vel ...        │
│ ↓                                    │
│ motor_driver_speed odbiera           │
│ ↓                                    │
│ Konwertuje cmd_vel → CAN frames      │
│ ↓                                    │
│ Wysyła do silników przez CAN         │
│ ↓                                    │
│ Silniki pracują! 🤖                  │
│ ↓                                    │
│ Enkodery odczytują pozycję           │
│ ↓                                    │
│ motor_driver publikuje /odom         │
│ ↓                                    │
│ User monitoruje: ros2 topic echo ... │
└──────────────────────────────────────┘
```

---

## 🎯 Minimalna lista komend (Quick Start)

### SETUP (jednorazowo na RPi)
```bash
# 1. Systemd Service
sudo nano /etc/systemd/system/can-init.service
# [Dodaj 15 linii zawartości]

sudo systemctl daemon-reload
sudo systemctl enable can-init.service
sudo systemctl start can-init.service

# 2. Build
cd ~/ros2_ws
colcon build --packages-select mks_motor_control
```

### KAŻDY START
```bash
# Terminal 1: Launch
source ~/ros2_ws/install/setup.bash
ros2 launch mks_motor_control motor_driver_complete.launch.py

# Terminal 2: Test
source ~/ros2_ws/install/setup.bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Terminal 3: Monitor
source ~/ros2_ws/install/setup.bash
ros2 topic echo /odom
```

---

## 🔍 Status Check

```bash
# 1. Systemd Service
sudo systemctl status can-init.service
# Expected: active (exited)

# 2. CAN Interface
ip link show can0
# Expected: can0: <NOARP,UP,RUNNING>

# 3. ROS2 Nodes
ros2 node list
# Expected: /motor_driver_speed, /robot_state_publisher

# 4. ROS2 Topics
ros2 topic list
# Expected: /cmd_vel, /odom, /joint_states

# 5. Full test
ros2 topic hz /odom
# Expected: ~20 Hz (20 publishes per second)
```

---

## ✅ Sukces = Gdy...

```
✅ sudo systemctl status can-init.service
   ↓
   Active: active (exited)

✅ ip link show can0
   ↓
   3: can0: <NOARP,UP,RUNNING>

✅ ros2 topic list | grep -E "cmd_vel|odom|joint_states"
   ↓
   /cmd_vel
   /joint_states
   /odom

✅ ros2 topic echo /odom -n 1
   ↓
   [Widać pozycję robota]

✅ ros2 topic pub -1 /cmd_vel ...
   ↓
   [Robot się porusza]

🎉 SUKCES! 🎉
```

---

## 🚨 Problemy - Quick Diagnosis

```
SYMPTOM: "ros2 launch" prosi o hasło
┌────────────────────────────────┐
│ Przyczyna: sudo bez NOPASSWD   │
│ Sprawdź: sudo systemctl status │
│ Rozwiąż: Systemd powinien UP   │
└────────────────────────────────┘

SYMPTOM: "CAN interface not available"
┌────────────────────────────────┐
│ Sprawdź: ip link show can0     │
│ Jeśli DOWN: sudo ip link set   │
│           can0 up              │
│ Debug: sudo journalctl -u can- │
│        init.service -n 20      │
└────────────────────────────────┘

SYMPTOM: "Package not found"
┌────────────────────────────────┐
│ Przyczyna: Nie sourced         │
│ Rozwiąż: source install/setup  │
│ Lub dodaj do ~/.bashrc         │
└────────────────────────────────┘

SYMPTOM: Topics nie widać
┌────────────────────────────────┐
│ Sprawdź: ros2 node list        │
│ Sprawdzić logi: ros2 launch    │
│ ... --log-level DEBUG          │
└────────────────────────────────┘
```

---

## 📝 Szybka ściąga

| Co | Gdzie | Komenda |
|---|---|---|
| **CAN Init Service** | /etc/systemd/system/can-init.service | `sudo systemctl status` |
| **Build Package** | ~/ros2_ws/ | `colcon build --packages-select mks_motor_control` |
| **Source Env** | ~/.bashrc | `source ~/ros2_ws/install/setup.bash` |
| **Launch** | terminal | `ros2 launch mks_motor_control motor_driver_complete.launch.py` |
| **Test** | terminal | `ros2 topic pub /cmd_vel ...` |
| **Monitor** | terminal | `ros2 topic echo /odom` |

---

**Powodzenia!** 🚀
