import can
import time

MOTOR_ID = 0x01
CHANNEL = "can0"
BITRATE = 500000

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

def clear_encoder_counter(bus):
    # Komenda 0x32 - Reset encoder counter
    cmd = [0x32]
    crc = calculate_crc(cmd, MOTOR_ID)
    msg = can.Message(
        arbitration_id=MOTOR_ID,
        is_extended_id=False,
        dlc=2,
        data=bytearray(cmd + [crc])
    )
    bus.send(msg)
    # Zazwyczaj nie ma odpowiedzi, ale można odczekać chwilę
    time.sleep(0.1)

if __name__ == "__main__":
    bus = can.interface.Bus(
        channel=CHANNEL,
        interface="socketcan",
        bitrate=BITRATE,
        receive_own_messages=False
    )
    clear_encoder_counter(bus)
    print("Licznik impulsów wyzerowany!")
    bus.shutdown()
