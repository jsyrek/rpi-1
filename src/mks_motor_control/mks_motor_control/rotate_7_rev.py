import rclpy
from rclpy.node import Node
import can
import time

ENCODER_PPR = 16384
MOTOR_ID = 0x01
GEAR_RATIO = 6.6   # liczba obrotów silnika na 1 obrót koła

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

class Rotate7Rev(Node):
    def __init__(self):
        super().__init__('rotate_7_rev')
        self.bus = can.interface.Bus(
            channel='can0',
            interface='socketcan',
            bitrate=500000,
            receive_own_messages=False
        )
        self.target_revs = GEAR_RATIO       # ile obrotów ma zrobić silnik
        self.speed_rpm = 100                # bezpieczna prędkość testowa
        self.get_logger().info(
            f'Start testu: {self.target_revs:.3f} obrotów silnika (1 obrót koła)'
        )

    def send_speed(self, direction, rpm_speed):
        rpm_speed = max(0, min(int(rpm_speed), 4095))
        rpm_hi = (rpm_speed >> 8) & 0x0F
        rpm_lo = rpm_speed & 0xFF
        dir_bit = 0x80 if direction == 'CCW' else 0x00
        dir_speed = dir_bit | rpm_hi
        data = [0xF6, dir_speed, rpm_lo, 128]
        crc = calculate_crc(data, MOTOR_ID)
        msg = can.Message(
            arbitration_id=MOTOR_ID,
            is_extended_id=False,
            data=bytearray(data + [crc]),
            dlc=5,
            is_fd=False,
            check=True
        )
        self.bus.send(msg)

    def stop_motor(self):
        try:
            self.send_speed('CW', 0)
        except can.CanError:
            pass

    def read_encoder_cumulative(self):
        # wyczyść stare wiadomości
        while True:
            old_msg = self.bus.recv(timeout=0.0)
            if old_msg is None:
                break
        cmd = [0x31]
        crc = calculate_crc(cmd, MOTOR_ID)
        msg = can.Message(
            arbitration_id=MOTOR_ID,
            is_extended_id=False,
            dlc=2,
            data=bytearray(cmd + [crc])
        )
        self.bus.send(msg)
        reply = self.bus.recv(timeout=1.0)
        if reply is not None and reply.arbitration_id == MOTOR_ID and len(reply.data) >= 7:
            data = reply.data
            count = (data[1] << 40) | (data[2] << 32) | (data[3] << 24) | \
                    (data[4] << 16) | (data[5] << 8) | data[6]
            if count >= 0x800000000000:
                count -= 0x1000000000000
            return count
        return None

    def rotate_once_wheel(self):
        start = self.read_encoder_cumulative()
        if start is None:
            self.get_logger().error('Brak odczytu enkodera, przerywam')
            return

        target_delta_counts = int(self.target_revs * ENCODER_PPR)
        self.get_logger().info(
            f'Cel: {self.target_revs:.3f} obrotów silnika = {target_delta_counts} impulsów'
        )

        # jedziemy w CCW, możesz zmienić na CW
        self.send_speed('CCW', self.speed_rpm)

        start_time = time.time()
        timeout_s = 695.0

        while True:
            curr = self.read_encoder_cumulative()
            if curr is None:
                self.get_logger().warn('Brak odczytu, próbuję dalej')
                time.sleep(0.05)
                if time.time() - start_time > timeout_s:
                    self.get_logger().error('Timeout – brak impulsów z enkodera')
                    break
                continue

            delta = curr - start
            # uproszczona obsługa ewentualnego przepełnienia
            if delta < 0:
                delta = abs(delta)

            self.get_logger().info(f'delta={delta} / {target_delta_counts}')
            if delta >= target_delta_counts:
                break

            if time.time() - start_time > timeout_s:
                self.get_logger().error('Timeout – nie osiągnięto celu impulsów')
                break

            time.sleep(0.02)

        self.stop_motor()
        self.get_logger().info('Koniec: zatrzymano silnik')

def main(args=None):
    rclpy.init(args=args)
    node = Rotate7Rev()
    try:
        node.rotate_once_wheel()
    finally:
        node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
