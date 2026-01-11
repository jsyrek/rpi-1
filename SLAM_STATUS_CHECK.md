# Sprawdzenie statusu SLAM Toolbox

## Test 1: Sprawdź czy SLAM Toolbox node działa

```bash
# Terminal 2 (gdy launch działa w Terminal 1)
ros2 node list | grep slam

# Sprawdź informacje o node
ros2 node info /slam_toolbox
```

## Test 2: Sprawdź czy SLAM publikuje mapę

```bash
# Terminal 2: Sprawdź czy topic /map istnieje
ros2 topic list | grep map

# Sprawdź czy /map ma publisher
ros2 topic info /map

# Sprawdź czy map jest publikowana (pierwszy message)
timeout 5 ros2 topic echo /map --once 2>/dev/null || echo "No map data yet"
```

## Test 3: Sprawdź czy SLAM otrzymuje skany

```bash
# Terminal 2: Sprawdź czy SLAM subskrybuje do scan topic
ros2 topic info /scan_throttled
# Powinien pokazać subscriber: /slam_toolbox

# Sprawdź częstotliwość skanów
ros2 topic hz /scan_throttled
```

## Test 4: Sprawdź czy SLAM jest aktywny (lifecycle state)

```bash
# Terminal 2: Sprawdź lifecycle state SLAM Toolbox
ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState "{}"
```

**Oczekiwany wynik:**
- State ID: 3 (active)
- Jeśli State ID: 1 (unconfigured) lub 2 (inactive) - SLAM nie jest aktywny

## Oczekiwane rezultaty dla działającego SLAM:

✅ **SLAM działa poprawnie jeśli:**
- `/slam_toolbox` node istnieje w `ros2 node list`
- `/map` topic ma publisher i publikuje dane
- `/scan_throttled` ma subscriber `/slam_toolbox`
- Lifecycle state = 3 (active)
- Brak błędów "Message Filter dropping message"

❌ **SLAM NIE działa jeśli:**
- Node nie istnieje
- `/map` nie ma publishera
- Lifecycle state != 3 (active)
- Są błędy w logach
