import rclpy
from rclpy.node import Node
import can
import time
import math

ENCODER_PPR = 16384
GEAR_RATIO = 6.3
WHEEL_RADIUS = 0.0425        # m
WHEEL_SEPARATION = 0.18      # m
MOTOR_LEFT_ID = 0x01
MOTOR_RIGHT_ID = 0x02

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

class RotateRobot360(Node):
    def __init__(self):
        super().__init__('rotate_robot_360')
        self.bus = can.interface.Bus(
            channel='can0',
            interface='socketcan',
            bitrate=1000000,
            receive_own_messages=False
        )
        
        # Oblicz ile obrotów koła potrzeba na rotację robota o 360°
        arc_length = (2 * math.pi) * (WHEEL_SEPARATION / 2.0)
        wheel_revolutions = arc_length / (2 * math.pi * WHEEL_RADIUS)
        motor_revolutions = wheel_revolutions * GEAR_RATIO
        
        self.target_motor_revs = motor_revolutions
        self.speed_rpm = 80                    # STAŁA prędkość (wolniej = dokładniej)
        self.accel = 20
        
        self.get_logger().info(
            f'Rotacja 360°: koło {wheel_revolutions:.3f} obr, '
            f'silnik {motor_revolutions:.3f} obr ({int(motor_revolutions * ENCODER_PPR)} impulsów)'
        )

    def send_speed(self, motor_id, direction, rpm_speed, accel=None):
        if accel is None:
            accel = self.accel

        rpm_speed = max(0, min(int(rpm_speed), 3000))
        accel = max(0, min(int(accel), 255))

        rpm_hi = (rpm_speed >> 8) & 0x0F
        rpm_lo = rpm_speed & 0xFF

        dir_bit = 0x80 if direction == 'CCW' else 0x00
        dir_speed = dir_bit | rpm_hi

        data = [0xF6, dir_speed, rpm_lo, accel]
        crc = calculate_crc(data, motor_id)

        msg = can.Message(
            arbitration_id=motor_id,
            is_extended_id=False,
            data=bytearray(data + [crc]),
            dlc=5
        )
        self.bus.send(msg)

    def stop_motor(self, motor_id):
        try:
            self.send_speed(motor_id, 'CW', 0, self.accel)
        except can.CanError:
            pass

    def read_encoder_cumulative(self, motor_id):
        while True:
            old_msg = self.bus.recv(timeout=0.0)
            if old_msg is None:
                break

        cmd = [0x31]
        crc = calculate_crc(cmd, motor_id)
        msg = can.Message(
            arbitration_id=motor_id,
            is_extended_id=False,
            dlc=2,
            data=bytearray(cmd + [crc])
        )
        self.bus.send(msg)
        reply = self.bus.recv(timeout=1.0)

        if reply is not None and reply.arbitration_id == motor_id and len(reply.data) >= 7:
            data = reply.data
            count = (data[1] << 40) | (data[2] << 32) | (data[3] << 24) | \
                    (data[4] << 16) | (data[5] << 8) | data[6]
            if count >= 0x800000000000:
                count -= 0x1000000000000
            return count
        return None

    def rotate_360(self):
        start_left = self.read_encoder_cumulative(MOTOR_LEFT_ID)
        start_right = self.read_encoder_cumulative(MOTOR_RIGHT_ID)
        
        if start_left is None or start_right is None:
            self.get_logger().error('Brak odczytu enkodera, przerywam')
            return

        target_counts = int(self.target_motor_revs * ENCODER_PPR)

        self.get_logger().info(f'Start rotacji: cel {target_counts} impulsów, prędkość STAŁA {self.speed_rpm} RPM')

        # Rotacja w prawo - oba silniki CW (prawy silnik jest odwrócony fizycznie)
        self.send_speed(MOTOR_LEFT_ID, 'CW', self.speed_rpm, self.accel)
        self.send_speed(MOTOR_RIGHT_ID, 'CW', self.speed_rpm, self.accel)

        start_time = time.time()
        timeout_s = 180.0

        while True:
            curr_left = self.read_encoder_cumulative(MOTOR_LEFT_ID)
            curr_right = self.read_encoder_cumulative(MOTOR_RIGHT_ID)
            
            if curr_left is None or curr_right is None:
                self.get_logger().warn('Brak odczytu, próbuję dalej')
                time.sleep(0.02)
                if time.time() - start_time > timeout_s:
                    self.get_logger().error('Timeout')
                    break
                continue

            delta_left = abs(curr_left - start_left)
            delta_right = abs(curr_right - start_right)
            delta_avg = (delta_left + delta_right) / 2.0

            self.get_logger().info(
                f'L={int(delta_left)}, R={int(delta_right)}, avg={int(delta_avg)}/{target_counts}'
            )

            # Sprawdź czy osiągnięto cel
            if delta_avg >= target_counts:
                break

            if time.time() - start_time > timeout_s:
                self.get_logger().error('Timeout – nie osiągnięto celu')
                break

            time.sleep(0.02)

        # Stop obu silników
        self.stop_motor(MOTOR_LEFT_ID)
        self.stop_motor(MOTOR_RIGHT_ID)
        
        # Końcowy raport
        final_left = abs(curr_left - start_left) if curr_left else 0
        final_right = abs(curr_right - start_right) if curr_right else 0
        
        self.get_logger().info(
            f'Koniec: L={int(final_left)} imp ({final_left/ENCODER_PPR/GEAR_RATIO:.3f} obr koła), '
            f'R={int(final_right)} imp ({final_right/ENCODER_PPR/GEAR_RATIO:.3f} obr koła)'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot360()
    try:
        node.rotate_360()
    finally:
        node.bus.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
