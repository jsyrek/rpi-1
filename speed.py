import can

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

MOTOR_ID = 0x01      # CAN ID silnika
CHANNEL = "can0"
BITRATE = 500000

# Ustaw zmienne od 0 do 255!
dir = 0           # 0 = CW (prawo), 1 = CCW (lewo)
speed = 0      # RPM (0..4095 po kodowaniu)
acc = 128         # Przyspieszenie (0..255)

if __name__ == "__main__":
    bus = can.interface.Bus(
        channel=CHANNEL,
        interface="socketcan",
        bitrate=BITRATE,
        receive_own_messages=False
    )

    try:
        # Rozbij speed (rpm) na dwa bajty wg protokołu MKS:
        rpm_hi = (speed >> 8) & 0x0F            # starsze bity speed
        rpm_lo = speed & 0xFF                   # młodsze bity speed
        dir_bit = 0x80 if dir == 1 else 0x00    # CCW: 0x80, CW: 0x00

        dir_speed = dir_bit | rpm_hi            # kierunek + rpm_hi

        # Gotowa ramka [0xF6, dir_speed, rpm_lo, acc, CRC]
        cmd = [0xF6, dir_speed, rpm_lo, acc]
        crc = calculate_crc(cmd, MOTOR_ID)

        msg = can.Message(
            arbitration_id=MOTOR_ID,
            is_extended_id=False,
            dlc=len(cmd)+1,
            data=bytearray(cmd + [crc])
        )
        bus.send(msg)
        print(f"Wysłano RUCH (Speed Mode): {[hex(b) for b in msg.data]} (ID={MOTOR_ID:02X})")
        print(f"Ustawienia: dir={dir} (0=CW, 1=CCW), speed={speed}, acc={acc}")

        # Odbierz odpowiedź sterownika (opcjonalnie)
        for _ in range(20):
            reply = bus.recv(0.1)
            if reply:
                print(f"Odebrano: ID={reply.arbitration_id:02X}, DLC={reply.dlc}, DATA={[hex(b) for b in reply.data]}")

    finally:
        bus.shutdown()
