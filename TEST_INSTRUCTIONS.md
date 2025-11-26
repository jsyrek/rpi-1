# TEST MOTOR_DRIVER_SPEED BEZ NAV2

## 📋 Krótko o testach:

### ✅ Czego potrzebujesz:
- **Uruchomiony motor_driver_speed.py** - publikuje `/cmd_vel` i `/odom`
- **ROS2 zainstalowany** na robotie
- **Fizycznie dostępny robot** do testów
- **(Opcjonalnie) Taśma lub znaczniki** na podłodze do mierzenia

### ❌ Czego NIE potrzebujesz:
- ❌ Nav2 - to jest do autonomicznej nawigacji
- ❌ LIDAR - to jest do mapy
- ❌ AMCL - to jest do lokalizacji na mapie
- ❌ Żadnych dodatkowych sensorów

---

## 🚀 Procedura testowania:

### TERMINAL 1: Uruchom motor_driver_speed
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run mks_motor_control motor_driver_speed
```

### TERMINAL 2: Uruchom tester (automatycznie)
```bash
cd ~/ros2_ws
source install/setup.bash
python3 test_motor_driver.py
```

Tester będzie:
1. Czekać na połączenie z odometrią (2 sekundy)
2. Wykonać **TEST 1: Jazda na wprost przez 5 sekund** przy 0.5 m/s
3. Czekać 3 sekundy
4. Wykonać **TEST 2: Jazda po okręgu** (promień 1m, 20 sekund na obrót)
5. Wypisać raport z walidacją

---

## 📊 Czego będzie szukać tester:

### Test 1: Jazda na wprost (5 sekund, 0.5 m/s)
```
✅ SPODZIEWANE REZULTATY:
- Delta X: ~2.5 metra (0.5 m/s × 5s)
- Delta Y: ~0 metrów (powinna jechać prosto!)
- Delta Theta: ~0 radianów (bez obracania się)

🔴 JEŚLI BĘDZIE INACZEJ:
- Y > 0.05m → robot boczył (silniki źle wyregulowane)
- Theta > 0.1 rad → robot się obracał (różne prędkości silników)
```

### Test 2: Jazda po okręgu (radius=1m, 20s/obrót)
```
✅ SPODZIEWANE REZULTATY:
- Droga: ~6.28 metra (obwód okręgu 2πR)
- Powrót do startu: X≈0, Y≈0 (zamknięty okrąg!)
- Rotacja: ~2π radianów (jeden pełny obrót)

🔴 JEŚLI BĘDZIE INACZEJ:
- Closure error > 0.1m → okrąg się nie zamknie (błąd odometrii)
- Theta < 2π → robot nie obrócił się wystarczająco
```

---

## 🎯 RĘCZNE TESTY (jeśli wolisz):

### Test 1: Jazda na wprost
```bash
# Terminal A: monitor odometrii
ros2 topic echo /odom | grep -E "position|orientation"

# Terminal B: wyślij komendę
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1

# Po 5 sekundach - wciśnij Ctrl+C w Terminal B
# Sprawdź czy robot:
# - Jechał prosto (brak odchylenia w Y)
# - Nie obracał się (theta pozostała ~0)
```

### Test 2: Jazda po okręgu
```bash
# Terminal A: monitor odometrii (jak wyżej)

# Terminal B: wyślij komendę (promień 1m)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.314}, angular: {z: 0.314}}" -1

# Po ~20 sekundach - wciśnij Ctrl+C
# Sprawdź czy robot:
# - Vrócił się do startu (X≈0, Y≈0)
# - Obwód drogi ~6.28m
# - Theta zmienił się o ~2π
```

---

## 🔍 Diagnostyka problemów:

### Problem: Robot nie jedzie (nie reaguje na cmd_vel)
```
1. Sprawdź czy motor_driver_speed jest uruchomiony
2. Sprawdź czy mogą komunikować się przez CAN
3. Monitoruj: ros2 topic echo /cmd_vel
```

### Problem: Robot jedzie ale nieprosto (odboczenie w Y)
```
PRZYCZYNY:
- Silniki mają różne prędkości
- Koła mają różne rozmiary
- Enkodery źle kalibrowane

ROZWIĄZANIE:
- Wyreguluj prędkość (DEFAULT_SPEED w motor_driver_speed.py)
- Zmierz faktyczne promienie kół
- Sprawdź wheel_radius w config/controller.yaml
```

### Problem: Błędy w odometrii (odchylenie po okręgu)
```
PRZYCZYNY:
- Enkodery dają niedokładne odczyty
- Boczne poślizgnięcie kół
- Parametry wheel_radius/wheel_separation niedokładne

ROZWIĄZANIE:
- Kalibruj koła (zmierz dokładnie promień i rozstaw)
- Patrz: https://wiki.ros.org/navigation/Tutorials/RobotSetup/Calibration
```

---

## 📝 Raport z testów

Po każdym teście otrzymasz raport:

```
============================================================
TEST 1: JAZDA NA WPROST
============================================================

Pozycja początkowa: x=0.0000, y=0.0000, θ=0.0000
Pozycja końcowa:    x=2.4567, y=0.0123, θ=0.0456

Delta X:      2.4567m
Delta Y:      0.0123m
Delta Theta:  0.0456 rad (2.61°)
Droga (euclidean): 2.4567m

------------------------------------------------------------
WALIDACJA:
------------------------------------------------------------
Oczekiwana droga: 2.5000m
Rzeczywista droga (x): 2.4567m
Błąd: 1.73%

✅ Jazda na wprost OK (Y offset = 0.0123m)
✅ Orientacja stabilna (θ offset = 0.0456 rad)

============================================================
```

---

## 💡 Wskazówki:

1. **Uruchom testy kilka razy** - sprawdź spójność wyników
2. **Jeśli wyniki się różnią** - to oznacza problemy z rejestracją enkodera
3. **Mierz fizycznie** - porównaj rzeczywistą drogę z odometrią
4. **Taśma na podłodze** - zaznacz start i sprawdź gdzie robot skończył

---

## 📞 Co zrobić jeśli coś nie działa:

1. Sprawdź czy `motor_driver_speed.py` jest uruchomiony
2. Sprawdź czy dostaje komendy: `ros2 topic echo /cmd_vel`
3. Sprawdź czy publikuje odometrię: `ros2 topic echo /odom`
4. Monitoruj błędy w logach: `ros2 node list` i `ros2 node info /motor_driver_speed`

---

**Powodzenia w testach!** 🤖✨
