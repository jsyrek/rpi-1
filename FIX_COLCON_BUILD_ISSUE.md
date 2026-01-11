# Rozwiązanie problemu: colcon build nie aktualizuje plików config

## Problem
`colcon build` czasami nie aktualizuje plików YAML w `install/` nawet po zmianach w źródle. To powoduje, że SLAM Toolbox używa starych parametrów.

## Rozwiązanie 1: Zawsze używaj czystego builda

```bash
cd ~/ros2_ws
rm -rf build/mks_motor_control install/mks_motor_control
colcon build --packages-select mks_motor_control
source install/setup.bash
```

## Rozwiązanie 2: Użyj --symlink-install (ZALECANE)

`--symlink-install` tworzy symlinki zamiast kopiować pliki, więc zawsze używa plików ze źródła:

```bash
cd ~/ros2_ws
rm -rf build install
colcon build --symlink-install --packages-select mks_motor_control
source install/setup.bash
```

**Uwaga:** Po pierwszym buildzie z `--symlink-install`, zawsze używaj `--symlink-install` dla tego pakietu.

## Rozwiązanie 3: Weryfikacja po buildzie

Uruchom skrypt weryfikacyjny:
```bash
cd ~/rpi-1
chmod +x VERIFY_COLCON_BUILD.sh
./VERIFY_COLCON_BUILD.sh
```

Skrypt sprawdzi, czy pliki w `install/` są identyczne z plikami w źródle.

## Rozwiązanie 4: Ręczna kopia (TEMP - tylko jeśli inne nie działają)

```bash
# Po colcon build, jeśli pliki nie są aktualne:
cp ~/ros2_ws/src/rpi-1/src/mks_motor_control/mks_motor_control/config/*.yaml \
   ~/ros2_ws/install/mks_motor_control/share/mks_motor_control/config/
```

## Zalecane podejście

**Użyj `--symlink-install`** - to najpewniejsze rozwiązanie, które zawsze używa aktualnych plików ze źródła.
