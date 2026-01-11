# Instrukcje debugowania problemu "Message Filter dropping message"

## Krok 1: Sprawdź czy colcon build poprawnie zainstalował pliki (PRZED uruchomieniem launch)

```bash
cd ~/rpi-1
git pull origin main
chmod +x VERIFY_COLCON_BUILD.sh
./VERIFY_COLCON_BUILD.sh
```

**Jeśli pliki w `install/` różnią się od źródła:**
- Użyj `--symlink-install` (patrz Krok 2)

## Krok 2: Czysty build z --symlink-install (ZALECANE)

```bash
cd ~/ros2_ws
rm -rf build/mks_motor_control install/mks_motor_control
colcon build --symlink-install --packages-select mks_motor_control
source install/setup.bash
```

**Uwaga:** `--symlink-install` tworzy symlinki zamiast kopiować pliki, więc zawsze używa aktualnych plików ze źródła.

## Krok 3: Uruchom launch file (Terminal 1)

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mks_motor_control SLAM-on-the-fly.py
```

**Pozostaw to terminal otwarty** - launch file musi działać!

## Krok 4: Sprawdź publikację TF (Terminal 2 - NOWY TERMINAL, podczas działania launch)

```bash
cd ~/rpi-1
chmod +x CHECK_TF_ODOM_PUBLICATION.sh
./CHECK_TF_ODOM_PUBLICATION.sh
```

**Uwaga:** Ten skrypt musi być uruchomiony PODCZAS działania launch file, bo `motor_driver_speed` publikuje TF tylko gdy jest uruchomiony.

## Krok 5: Sprawdź logi SLAM Toolbox (w Terminalu 1)

Szukaj w logach:
- Błędy związane z TF (szukaj słów: "TF", "transform", "frame", "odom")
- Błędy "Message Filter dropping message"
- Ostrzeżenia o brakujących transformacjach

## Jeśli błąd nadal występuje:

Prześlij wyniki:
1. Wynik `VERIFY_COLCON_BUILD.sh` (Krok 1)
2. Wynik `CHECK_TF_ODOM_PUBLICATION.sh` (Krok 4 - PODCZAS działania launch)
3. Fragment logów z Terminalu 1 (szczególnie błędy związane z TF i "Message Filter")

## Szybka weryfikacja (wszystko w jednym):

```bash
# Terminal 1: Uruchom launch
cd ~/ros2_ws
source install/setup.bash
ros2 launch mks_motor_control SLAM-on-the-fly.py

# Terminal 2 (NOWY): Sprawdź TF (podczas działania launch)
cd ~/rpi-1
chmod +x CHECK_TF_ODOM_PUBLICATION.sh
./CHECK_TF_ODOM_PUBLICATION.sh
```
