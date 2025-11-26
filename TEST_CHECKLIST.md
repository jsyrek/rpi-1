# CHECKLIST TESTÓW MOTOR_DRIVER_SPEED

## ✅ PRZEDtestami - Przygotowanie

- [ ] motor_driver_speed.py jest skompilowany i gotowy do uruchomienia
- [ ] ROS2 środowisko jest sourced (`source install/setup.bash`)
- [ ] CAN interface jest dostępny (`can0`)
- [ ] Silniki MKS Servo42D są podłączone (ID 0x01 i 0x02)
- [ ] Koła mogą się swobodnie obracać
- [ ] Robot ma miejsce do jazdy (min. 3m x 3m)

---

## 🚗 TEST 1: JAZDA NA WPROST

### Parametry:
- Czas: 5 sekund
- Prędkość liniowa: 0.5 m/s
- Prędkość kątowa: 0 rad/s (bez rotacji!)
- Oczekiwana droga: ~2.5 metra

### Kroki:

```bash
# Terminal 1: Uruchom sterownik
ros2 run mks_motor_control motor_driver_speed

# Terminal 2: Monitoruj odometrię (PRZED testem)
ros2 topic echo /odom

# Terminal 3: Wyślij komendę jazdy
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Poczekaj 5 sekund, obserwując Terminal 2

# Po 5 sekundach: Ctrl+C w Terminal 3 (aby zatrzymać publikowanie)
```

### Czego szukać:

```
✅ DOBRZE:
- Position.x rośnie monotonnie (~0.5m/s)
- Position.y pozostaje ~0 (±0.05m)
- Orientation (z,w) pozostaje ~[0,1]
- Prędkości: linear.x≈0.5, angular.z≈0

❌ ŹLE:
- Position.y ma dużą wartość → robot boczył
- Orientation szybko się zmienia → robot się obraca
- Prędkości nie odpowiadają komendzie
```

### Walidacja danych (z echo /odom):
```
Należy sprawdzić ostatnią wartość:
- Delta X = final_x - initial_x = ? (powinno być ~2.5m)
- Delta Y = final_y - initial_y = ? (powinno być ~0)
- Delta theta = ? (powinno być ~0)

Błąd: |(Delta X - 2.5)| / 2.5 * 100 = ? % (powinno <5%)
```

---

## 🔄 TEST 2: JAZDA PO OKRĘGU

### Parametry:
- Promień: 1.0 metr
- Czas na pełny obrót: 20 sekund
- Obwód: 2π × 1.0 = ~6.28 metra
- Prędkość liniowa: 6.28 / 20 = 0.314 m/s
- Prędkość kątowa: 0.314 / 1.0 = 0.314 rad/s

### Kroki:

```bash
# Terminal 1: Uruchom sterownik
ros2 run mks_motor_control motor_driver_speed

# Terminal 2: Monitoruj odometrię (PRZED testem)
ros2 topic echo /odom

# Terminal 3: Wyślij komendę jazdy po okręgu
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.314}, angular: {z: 0.314}}"

# Poczekaj 20 sekund, obserwując Terminal 2
# Po 20 sekundach: Ctrl+C w Terminal 3
```

### Czego szukać:

```
✅ DOBRZE:
- Robot porusza się w łuku (obie współrzędne X i Y się zmieniają)
- Pozycja zakreśla ćwierćkę/połowę/pełny okrąg
- Orientacja (theta) systematycznie rośnie
- Droga Euclidean ≈ 6.28m

❌ ŹLE:
- Robot nie tworzy łuku (porusza się prosto)
- Pozycja X i Y szybko się zmieniają losowo (błędy)
- Theta nie zmienia się (problem z rotacją)
- Droga znacznie inna niż 6.28m
```

### Walidacja danych (z echo /odom):
```
Zamknięcie okręgu:
- final_x powinno być bliskie initial_x (±0.2m)
- final_y powinno być bliskie initial_y (±0.2m)
- closure_error = sqrt((Δx)² + (Δy)²) powinno być <0.2m

Rotacja:
- Δtheta powinno być ≈ 2π ≈ 6.28 rad
- Jeśli <2π: robot nie obrócił się wystarczająco
- Jeśli >2π: robot się "przewrócił" - błąd odometrii

Droga:
- euclidean_distance ≈ 6.28m (obwód)
```

---

## 🔧 DIAGNOSTYKA BŁĘDÓW

### Problem: Robot nie reaguje na komendy

```
1. Sprawdź czy motor_driver_speed się uruchamia:
   ros2 node list
   → Powinno być /motor_driver_speed

2. Sprawdź czy dostaje komendy:
   ros2 topic echo /cmd_vel
   → Powinno być publiczne

3. Sprawdzaj logi sterownika:
   ros2 run mks_motor_control motor_driver_speed  (bez & na końcu)
   → Powinna być nazwa węzła i komunikat inicjalizacji

4. Sprawdź czy CAN się otwiera:
   ip link show can0
   → Powinien być UP
```

### Problem: Robot jedzie ale nieprosto

```
Objaw: Podczas Test 1 (jazda na wprost) Position.y rośnie
Przyczyna: Silniki mają różne prędkości

Rozwiązanie:
1. Zmierz rzeczywiste promienie kół
2. Kalibruj DEFAULT_SPEED w motor_driver_speed.py
3. Sprawdź czy koła się nie ślizgają
4. Czy motor_1_inverted i motor_2_inverted są poprawnie ustawione?
```

### Problem: Test okręgu się nie zamyka

```
Objawy:
- final_x i final_y znacznie różnią się od 0
- Błąd zamknięcia okręgu >0.2m

Przyczyny:
1. Enkodery słabo kalibrowane
2. Parametry wheel_radius lub wheel_separation niedokładne
3. Systemic bias w prędkości kół

Rozwiązanie:
1. Zmierz kół dokładnie (linijka + suwmiarka)
2. Update controller.yaml: wheel_radius, wheel_separation
3. Uruchom kalibrację enkodera
```

### Problem: Theta nie zmienia się prawidłowo

```
Objawy: 
- Delta theta podczas okręgu <2π
- Robot się nie obraca

Przyczyny:
1. Odwrócenie silnika źle skonfigurowane
2. Angular velocity za niska
3. Enkodery nie liczą rotacji

Rozwiązanie:
1. Sprawdź motor_1_inverted, motor_2_inverted w config
2. Zwiększ angular.z w komendzie
3. Testuj: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
   (tylko rotacja bez ruchu)
```

---

## 📊 TRACKING WYNIKÓW

Zapisz wyniki testów (kilka powtórzeń):

### Test 1 - Jazda na wprost:
```
Run 1: ΔX=2.45m ΔY=0.01m Δθ=0.04rad → ERROR: 2%
Run 2: ΔX=2.43m ΔY=0.02m Δθ=0.05rad → ERROR: 2.8%
Run 3: ΔX=2.46m ΔY=0.00m Δθ=0.03rad → ERROR: 1.6%

Średni błąd: ~2.1% → ✅ AKCEPTOWALNY
```

### Test 2 - Okrąg:
```
Run 1: Zamknięcie=0.15m Droga=6.20m Δθ=6.15rad (vs 6.28)
Run 2: Zamknięcie=0.18m Droga=6.25m Δθ=6.20rad
Run 3: Zamknięcie=0.12m Droga=6.28m Δθ=6.18rad

Średnie: zamknięcie=0.15m, droga=6.24m, rotacja=6.18rad
→ ⚠️ Niewielki błąd - może być lepiej po kalibracji
```

---

## ✅ KRYTERIA AKCEPTACJI

Test przechodzi jeśli:

### Test 1 (Jazda na wprost):
- ✅ Y offset < 0.05m (nie ma boczenia)
- ✅ Theta offset < 0.1 rad (nie ma obrotu)
- ✅ X droga w zakresie 2.3-2.7m (błąd <8%)

### Test 2 (Okrąg):
- ✅ Zamknięcie okręgu < 0.2m
- ✅ Droga w zakresie 6.0-6.6m
- ✅ Rotacja > 2π * 0.9 (co najmniej 5.65 rad)

---

## 📝 RAPORT TESTOWY

Po testach zanotuj:

```markdown
## Raport testów motor_driver_speed

Data: [DATA]
Robot: efoil
Środowisko: ROS2 Humble

### Test 1: Jazda na wprost
Status: ✅ PASSED / ❌ FAILED
- Przesunięcie X: 2.45m (oczekiwane: 2.5m, błąd: 2%)
- Przesunięcie Y: 0.01m (max: 0.05m) ✅
- Rotacja: 0.04rad (max: 0.1rad) ✅

### Test 2: Okrąg
Status: ✅ PASSED / ❌ FAILED
- Zamknięcie: 0.15m (max: 0.2m) ✅
- Droga: 6.25m (oczekiwane: 6.28m, błąd: 0.5%) ✅
- Rotacja: 6.18rad (oczekiwane: 6.28rad, błąd: 1.6%) ✅

### Uwagi:
[Tutaj notatki o problemach i obserwacjach]

### Następne kroki:
- [ ] Kalibruj enkodery
- [ ] Zmierz koła dokładnie
- [ ] Test z LIDARem (kiedy będzie)
```

---

**Powodzenia w testach!** 🤖
