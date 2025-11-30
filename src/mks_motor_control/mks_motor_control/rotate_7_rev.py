#!/usr/bin/env python3
import can
import time

MOTOR_ID = 0x01
ENCODER_PPR = 16384
TARGET_IMPULSES = 16384*6.5  # 1 pełny obrót silnika = 360 stopni

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

def send_speed(bus, direction, rpm_speed):
    rpm_speed = max(0, min(int(rpm_speed), 3000))
    rpm_hi = (rpm_speed >> 8) & 0x0F
    rpm_lo = rpm_speed & 0xFF
    dir_bit = 0x80 if direction == 'CCW' else 0x00
    dir_speed = dir_bit | rpm_hi
    data = [0xF6, dir_speed, rpm_lo, 20]  # accel=20
    crc = calculate_crc(data, MOTOR_ID)
    msg = can.Message(
        arbitration_id=MOTOR_ID,
        is_extended_id=False,
        data=bytearray(data + [crc]),
        dlc=5
    )
    bus.send(msg)

def stop_motor(bus):
    send_speed(bus, 'CW', 0)

def read_encoder(bus):
    # Wyczyść bufor
    while bus.recv(timeout=0.0) is not None:
        pass
    
    cmd = [0x31]
    crc = calculate_crc(cmd, MOTOR_ID)
    msg = can.Message(
        arbitration_id=MOTOR_ID,
        is_extended_id=False,
        dlc=2,
        data=bytearray(cmd + [crc])
    )
    bus.send(msg)
    reply = bus.recv(timeout=1.0)
    
    if reply and reply.arbitration_id == MOTOR_ID and len(reply.data) >= 7:
        data = reply.data
        count = (data[1]<<40) | (data[2]<<32) | (data[3]<<24) | (data[4]<<16) | (data[5]<<8) | data[6]
        if count >= 0x800000000000:
            count -= 0x1000000000000
        return count
    return None

def main():
    bus = can.interface.Bus(
        channel='can0',
        interface='socketcan',
        bitrate=500000,
        receive_own_messages=False
    )
    
    print("Start: obrót silnika o 360 stopni (16384 impulsy)")
    
    # Odczytaj pozycję startową
    start = read_encoder(bus)
    if start is None:
        print("BŁĄD: Brak odczytu enkodera!")
        bus.shutdown()
        return
    
    print(f"Pozycja startowa: {start}")
    
    # Uruchom silnik
    send_speed(bus, 'CCW', 100)  # 100 RPM
    
    # Próg zwolnienia przy 99%
    slowdown_threshold = int(0.97 * TARGET_IMPULSES)
    slowed = False
    
    # Czekaj na osiągnięcie celu
    while True:
        curr = read_encoder(bus)
        if curr is None:
            time.sleep(0.001)
            continue
        
        delta = abs(curr - start)
        print(f"Impulsy: {delta}/{TARGET_IMPULSES}")
        
        # Zwolnij przy 99%
        if not slowed and delta >= slowdown_threshold:
            print("Zwalniam...")
            send_speed(bus, 'CCW', 1)
            slowed = True
        
        if delta >= TARGET_IMPULSES:
            break
        
        time.sleep(0.001)
    
    # Zatrzymaj
    stop_motor(bus)
    print("Zakończono - silnik obrócił się o 360 stopni")
    bus.shutdown()

if __name__ == '__main__':
    main()
