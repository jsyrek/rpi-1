import can
import time

MOTOR_ID = 0x01
CHANNEL = "can0"
BITRATE = 500000
ENCODER_PPR = 16384

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

def read_encoder_cumulative(bus):
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
        return count, data
    else:
        return None, None

if __name__ == "__main__":
    bus = can.interface.Bus(
        channel=CHANNEL,
        interface="socketcan",
        bitrate=BITRATE,
        receive_own_messages=False
    )

    print("Sumaryczna ilość PEŁNYCH obrotów z MKS Servo42D. Ctrl+C aby zakończyć\n")
    zero = None
    prev_full_turns = None

    try:
        while True:
            count, raw_data = read_encoder_cumulative(bus)
            if count is None:
                print("Błąd odczytu CAN!")
                time.sleep(0.1)
                continue
            if zero is None:
                zero = count

            # LICZ TYLKO PEŁNE OBROTY (liczba całkowita):
            full_turns = (count - zero) // ENCODER_PPR   # dzielenie całkowite

            if prev_full_turns is None or full_turns != prev_full_turns:
                print(f"PEŁNE obroty od startu: {full_turns}")
                print(f"RAMKA: {[hex(b) for b in raw_data]}")
                prev_full_turns = full_turns
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nPrzerwano przez użytkownika")
    except Exception as e:
        print(f"Błąd: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bus.shutdown()
