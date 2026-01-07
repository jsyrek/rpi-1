# Instrukcja: Jak wypchnąć pliki na GitHub

## Krok 1: Utwórz Personal Access Token przez przeglądarkę

1. **Otwórz przeglądarkę** i przejdź na:
   ```
   https://github.com/settings/tokens
   ```

2. **Kliknij "Generate new token"** → **"Generate new token (classic)"**

3. **Nadaj nazwę tokenowi** (np. "rpi-1-push")

4. **Wybierz uprawnienia:**
   - ✅ **repo** (pełny dostęp do repozytoriów)
   - To wystarczy do pushowania

5. **Kliknij "Generate token"** na dole strony

6. **SKOPIUJ TOKEN** - pokaże się tylko raz! Zapisz go bezpiecznie.

## Krok 2: Wypchnij pliki używając tokena

### Opcja A: W PowerShell (jednorazowo)
```powershell
cd rpi-1
git push https://<TWÓJ_TOKEN>@github.com/jsyrek/rpi-1.git main
```
Zastąp `<TWÓJ_TOKEN>` skopiowanym tokenem.

### Opcja B: Zapisanie credentials (na stałe)
```powershell
cd rpi-1
git config credential.helper store
git push origin main
```
Gdy zapyta o:
- **Username**: `jsyrek`
- **Password**: `<TWÓJ_TOKEN>` (nie hasło GitHub, tylko token!)

## Krok 3: Sprawdź na GitHub

Przejdź na: https://github.com/jsyrek/rpi-1

Powinieneś zobaczyć nowe pliki:
- `AMCL-lidar-odometry-IMU.py`
- `SLAM-on-the-fly.py`
- `SLAM_README.md`
- `amcl_config.yaml`
- Zaktualizowany `slam_toolbox.yaml`

## Bezpieczeństwo

⚠️ **Token jest jak hasło** - nie udostępniaj go publicznie!
- Jeśli token wycieknie, możesz go usunąć na: https://github.com/settings/tokens
- Możesz utworzyć nowy token w każdej chwili
