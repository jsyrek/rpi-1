import can
import math

# Parametry robota
WHEEL_RADIUS = 0.0425       # [m] promień koła
WHEEL_SEPARATION = 0.18     # [m] rozstaw kół
MOTOR_STEPS_PER_REV = 3200  # Mikrokroki silnika
GEAR_RATIO = 6.3            # Przełożenie

# Oblicz impulsy dla rotacji 360°
arc = math.pi * WHEEL_SEPARATION  # Obwód koła rotacji
wheel_revs = arc / (2 * math.pi * WHEEL_RADIUS)
motor_revs = wheel_revs * GEAR_RATIO
PULSES = int(motor_revs * MOTOR_STEPS_PER_REV)

print(f"Rotacja 360°: {PULSES} impulsów")

# Lista ID silników i kierunki (jeden do przodu, drugi do tyłu)
MOTORS = [
    (0x01, 'CCW'),   # Lewy silnik
    (0x02, 'CCW')   # Prawy silnik (przeciwny kierunek!)
]

def calculate_mks_crc(data, can_id):
    crc_data = [can_id] + data
    return sum(crc_data) & 0xFF

def create_position_command(direction, pulses, speed, acc, can_id):
    dir_bit = 0x80 if direction == 'CCW' else 0x00
    speed_high = (speed >> 8) & 0x0F
    speed_low = speed & 0xFF
    dir_speed = dir_bit | speed_high

    pulses_high = (pulses >> 16) & 0xFF
    pulses_mid = (pulses >> 8) & 0xFF
    pulses_low = pulses & 0xFF

    data = [
        0xFD,
        dir_speed,
        speed_low,
        acc,
        pulses_high,
        pulses_mid,
        pulses_low
    ]
    crc = calculate_mks_crc(data, can_id)
    frame = data + [crc]
    assert len(frame) == 8, f"Nieprawidłowa długość ramki: {len(frame)} bajtów"
    return frame

MIN_SPEED = 200
MIN_ACC = 100

bus = can.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=1000000,
    receive_own_messages=False
)

try:
    for motor_id, direction in MOTORS:
        data = create_position_command(direction, PULSES, MIN_SPEED, MIN_ACC, motor_id)
        msg = can.Message(
            arbitration_id=motor_id,
            is_extended_id=False,
            data=data,
            dlc=8,
            is_fd=False,
            check=True
        )
        try:
            bus.send(msg, timeout=0.1)
            print(f"[ID {motor_id:02X}] {direction} Wysłano: {' '.join(f'{b:02X}' for b in data)}")
        except can.CanError as e:
            print(f"Błąd wysyłki ID {motor_id:02X}: {e}")
finally:
    bus.shutdown()
