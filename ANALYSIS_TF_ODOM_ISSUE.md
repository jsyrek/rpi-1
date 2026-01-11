# Analiza: Frame 'odom' nie jest widoczny w TF tree mimo publikacji TF

## Problem
- ✅ TF `odom -> base_link` **JEST** publikowany na topicu `/tf` (potwierdzono przez `ros2 topic echo /tf`)
- ❌ Frame `odom` **NIE JEST** widoczny w TF tree (`tf2_monitor` nie pokazuje `odom`)
- ❌ `tf2_echo odom base_link` zgłasza: "Invalid frame ID 'odom' passed to canTransform argument target_frame - frame does not exist"

## Konsekwencje
- SLAM Toolbox potrzebuje transformacji `unilidar_lidar -> odom`
- Bez frame'a `odom` w TF tree, SLAM Toolbox nie może znaleźć tej transformacji
- To powoduje błąd "Message Filter dropping message" - SLAM Toolbox odrzuca wiadomości, bo nie może znaleźć transformacji

## Możliwe przyczyny

### 1. TF2 Buffer nie widzi dynamicznych frame'ów bez statycznego root'a
- W ROS2, dynamiczne TF mogą być publikowane bez statycznego root'a
- Ale TF2 buffer może potrzebować, aby frame był "zarejestrowany" zanim może być używany
- **Rozwiązanie**: Sprawdź, czy TF2 buffer jest poprawnie skonfigurowany

### 2. Timing issue - TF jest publikowany, ale TF2 buffer nie zdążył go przetworzyć
- `tf2_monitor` zbiera dane przez 10 sekund
- Jeśli TF nie jest publikowany przez cały czas, frame może nie być widoczny
- **Rozwiązanie**: Upewnij się, że TF jest publikowany ciągle (50 Hz)

### 3. Problem z QoS - TF może być publikowany z niewłaściwym QoS
- Jeśli QoS nie pasuje do subskrypcji TF2 buffer, TF może być odrzucany
- **Rozwiązanie**: Sprawdź QoS publikacji TF w `motor_driver_speed.py`

## Sprawdzenie QoS w motor_driver_speed.py
Obecnie kod używa domyślnego `TransformBroadcaster`, który powinien używać właściwego QoS dla TF. Ale sprawdźmy to.

## Następne kroki

1. **Sprawdź logi SLAM Toolbox** - czy są błędy związane z TF?
2. **Sprawdź QoS** - czy TF jest publikowany z właściwym QoS?
3. **Sprawdź timing** - czy TF jest publikowany ciągle (50 Hz)?
4. **Sprawdź TF2 buffer** - czy TF2 buffer jest poprawnie skonfigurowany w SLAM Toolbox?

## Hipoteza
Problem może być w tym, że TF2 buffer w SLAM Toolbox nie widzi frame'a `odom`, mimo że TF jest publikowany. To może być problem z konfiguracją TF2 buffer w SLAM Toolbox lub z timing'iem publikacji TF.
