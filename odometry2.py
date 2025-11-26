import can
import time

# Parametry enkodera
ENCODER_PPR = 16384  # Impulsy na obrót MKS Servo42D

MOTOR_ID = 0x01
CHANNEL = "can0"
BITRATE = 500000

def calculate_crc(data, can_id):
    """CRC dla protokołu MKS CAN"""
    return (can_id + sum(data)) & 0xFF

def read_encoder_position(bus):
    """Odczytaj pozycję enkodera (komenda 0x31)"""
    cmd = [0x31]
    crc = calculate_crc(cmd, MOTOR_ID)
    msg = can.Message(
        arbitration_id=MOTOR_ID,
        is_extended_id=False,
        dlc=2,
        data=bytearray(cmd + [crc])
    )
    bus.send(msg)
    
    # Odbierz odpowiedź
    reply = bus.recv(timeout=1.0)
    if reply and reply.arbitration_id == MOTOR_ID and len(reply.data) >= 7:
        # Odpowiedź: [0x31, carry_bytes(4), encoder_hi, encoder_lo, CRC]
        # Pomijamy carry (bajty 1-4), bierzemy tylko encoder (bajty 5-6)
        encoder_hi = reply.data[5]
        encoder_lo = reply.data[6]
        encoder_raw = (encoder_hi << 8) | encoder_lo
        
        # Konwersja do int16_t
        if encoder_raw >= 32768:
            encoder_pos = encoder_raw - 65536
        else:
            encoder_pos = encoder_raw
            
        return encoder_pos, reply.data
    else:
        return None, None

if __name__ == "__main__":
    bus = can.interface.Bus(
        channel=CHANNEL,
        interface="socketcan",
        bitrate=BITRATE,
        receive_own_messages=False
    )

    try:
        print("Rozpoczynam pomiar prędkości...")
        print("Naciśnij Ctrl+C aby zatrzymać\n")
        
        # Pierwszy odczyt
        prev_pos, _ = read_encoder_position(bus)
        if prev_pos is None:
            print("Błąd odczytu początkowego!")
            exit(1)
            
        prev_time = time.time()
        time.sleep(0.1)  # Interwał pomiarowy
        
        while True:
            # Drugi odczyt
            curr_pos, raw_data = read_encoder_position(bus)
            if curr_pos is None:
                print("Błąd odczytu CAN!")
                time.sleep(0.1)
                continue
                
            curr_time = time.time()
            
            # Oblicz deltę pozycji i czasu
            delta_pos = curr_pos - prev_pos
            delta_time = curr_time - prev_time
            
            # Obsługa przekręcenia licznika enkodera (-32768 do 32767)
            if delta_pos > 32768:
                delta_pos -= 65536
            elif delta_pos < -32768:
                delta_pos += 65536
            
            # Przelicz na RPM
            if delta_time > 0:
                pulses_per_sec = delta_pos / delta_time
                rpm = abs(pulses_per_sec) * 60.0 / ENCODER_PPR
                direction = "CCW" if delta_pos >= 0 else "CW"
                
                print(f"Pozycja: {curr_pos:6d} | Delta: {delta_pos:6d} | "
                      f"RPM: {rpm:7.2f} | Kierunek: {direction:3s}")
                print(f"RAW bajty: {[hex(b) for b in raw_data]}")
            
            # Aktualizuj poprzednie wartości
            prev_pos = curr_pos
            prev_time = curr_time
            
            time.sleep(0.1)  # Interwał pomiarowy (100ms)
    
    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika")
    except Exception as e:
        print(f"Błąd: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bus.shutdown()
