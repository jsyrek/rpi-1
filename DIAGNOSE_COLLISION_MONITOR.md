# Diagnostyka problemu collision_monitor w Nav2 Jazzy

## Problem
`collision_monitor` crashuje z błędem `parameter 'observation_sources' is not initialized`, mimo że:
- `use_collision_monitor: 'False'` jest ustawione w launch file
- Sekcja `collision_monitor` w `nav2_params.yaml` ma `enabled: False` i `observation_sources`

## Testy diagnostyczne

### Test 1: Sprawdź, czy parametr `use_collision_monitor` jest przekazywany
```bash
# Uruchom launch file i sprawdź logi
ros2 launch mks_motor_control SLAM-on-the-fly.py 2>&1 | grep -i "collision\|use_collision"
```

### Test 2: Sprawdź źródło kodu Nav2 navigation_launch.py
```bash
# Znajdź plik navigation_launch.py w Nav2
find /opt/ros/jazzy -name "navigation_launch.py" 2>/dev/null

# Jeśli znajdziesz, sprawdź jak obsługuje use_collision_monitor
cat /opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py | grep -A 20 -B 5 "collision"
```

### Test 3: Sprawdź, jakie parametry są przekazywane do collision_monitor
```bash
# Po uruchomieniu launch file, sprawdź parametry węzła collision_monitor
ros2 param list /collision_monitor

# Sprawdź konkretne parametry
ros2 param get /collision_monitor enabled
ros2 param get /collision_monitor observation_sources
ros2 param get /collision_monitor use_sim_time
```

### Test 4: Sprawdź, czy collision_monitor jest w liście lifecycle_manager
```bash
# Sprawdź parametry lifecycle_manager_navigation
ros2 param list /lifecycle_manager_navigation
ros2 param get /lifecycle_manager_navigation node_names
```

### Test 5: Sprawdź plik tymczasowy z parametrami
```bash
# W logach launch file widzisz: --params-file /tmp/launch_params_XXXXX
# Sprawdź zawartość tego pliku (podmień XXXXX na rzeczywistą nazwę z logów)
cat /tmp/launch_params_XXXXX | grep -A 20 "collision_monitor"
```

### Test 6: Sprawdź, czy use_collision_monitor jest w launch arguments
```bash
# Uruchom launch file z dodatkowym logowaniem
ros2 launch mks_motor_control SLAM-on-the-fly.py --show-args
```

### Test 7: Sprawdź dokumentację Nav2 Jazzy
```bash
# Sprawdź, czy use_collision_monitor jest obsługiwane w Jazzy
ros2 run nav2_bringup nav2_bringup --help 2>&1 | grep -i collision
```

### Test 8: Sprawdź, czy collision_monitor jest wymagany przez lifecycle_manager
```bash
# Po uruchomieniu, sprawdź konfigurację lifecycle_manager
ros2 param get /lifecycle_manager_navigation node_names
# Sprawdź, czy 'collision_monitor' jest na liście
```

### Test 9: Sprawdź logi z większą szczegółowością
```bash
# Uruchom z większą szczegółowością logowania
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch mks_motor_control SLAM-on-the-fly.py 2>&1 | tee launch_debug.log
# Następnie przeszukaj logi
grep -i "collision\|use_collision\|observation_sources" launch_debug.log
```

### Test 10: Sprawdź, czy można całkowicie pominąć collision_monitor
```bash
# Sprawdź, czy w navigation_launch.py jest warunek IfCondition dla use_collision_monitor
python3 -c "
import sys
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
try:
    from nav2_bringup.launch import navigation_launch
    import inspect
    source = inspect.getsource(navigation_launch)
    print('=== SEARCHING FOR collision_monitor ===')
    for i, line in enumerate(source.split('\n'), 1):
        if 'collision' in line.lower():
            print(f'{i}: {line}')
except Exception as e:
    print(f'Error: {e}')
"
```

## Najważniejsze pytania do odpowiedzi:

1. **Czy `use_collision_monitor: 'False'` jest faktycznie przekazywane do `navigation_launch.py`?**
   - Wykonaj Test 1, 6, 9

2. **Jak `navigation_launch.py` obsługuje `use_collision_monitor`?**
   - Wykonaj Test 2, 10

3. **Czy `collision_monitor` jest na liście węzłów w `lifecycle_manager`?**
   - Wykonaj Test 4, 8

4. **Jakie parametry są faktycznie przekazywane do węzła `collision_monitor`?**
   - Wykonaj Test 3, 5

5. **Czy w Nav2 Jazzy `use_collision_monitor` jest w ogóle obsługiwane?**
   - Wykonaj Test 7, 2

## Rekomendowane kolejne kroki:

Po wykonaniu testów, zbierz wyniki i przeanalizuj:
- Jeśli `use_collision_monitor` nie jest obsługiwane w Nav2 Jazzy → trzeba ręcznie usunąć `collision_monitor` z launch file
- Jeśli `collision_monitor` jest na liście `lifecycle_manager` mimo `use_collision_monitor: False` → trzeba ręcznie zmodyfikować listę węzłów
- Jeśli parametry nie są przekazywane poprawnie → trzeba sprawdzić format YAML lub sposób przekazywania parametrów
