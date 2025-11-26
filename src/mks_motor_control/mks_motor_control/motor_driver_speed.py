import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import can
import math
import time

MOTOR_IDS = [0x01, 0x02]
ENCODER_PPR = 16384        # Impulsy na obrót silnika MKS Servo42D
GEAR_RATIO = 4.875         # Przełożenie silnik:koło JAK U CIEBIE
DEFAULT_SPEED = 500
DEFAULT_ACC = 128
MAX_SPEED = 4095
DEFAULT_WHEEL_RADIUS = 0.05        # Promień koła [m]
DEFAULT_WHEEL_SEPARATION = 0.18    # Rozstaw kół [m]

def calculate_crc(data, can_id):
    return (can_id + sum(data)) & 0xFF

class MotorDriverSpeed(Node):
    def __init__(self):
        super().__init__('motor_driver_speed')
        self.bus = can.interface.Bus(
            channel='can0',
            interface='socketcan',
            bitrate=500000,
            receive_own_messages=False
        )
        self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS)
        self.declare_parameter('wheel_separation', DEFAULT_WHEEL_SEPARATION)
        self.declare_parameter('motor_1_inverted', False)
        self.declare_parameter('motor_2_inverted', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.motor_1_inverted = self.get_parameter('motor_1_inverted').value
        self.motor_2_inverted = self.get_parameter('motor_2_inverted').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.reset_odom_srv = self.create_service(
            Empty, 'reset_odometry', self.reset_odometry_callback
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_left_cum = None
        self.prev_right_cum = None
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.publisher_timer_callback)

    def create_speed_command(self, direction, rpm_speed, acc, can_id):
        rpm_speed = max(0, min(int(rpm_speed), MAX_SPEED))
        rpm_hi = (rpm_speed >> 8) & 0x0F
        rpm_lo = rpm_speed & 0xFF
        dir_bit = 0x80 if direction == 'CCW' else 0x00
        dir_speed = dir_bit | rpm_hi
        data = [0xF6, dir_speed, rpm_lo, acc]
        crc = calculate_crc(data, can_id)
        return data + [crc]

    def invert_direction(self, direction):
        return 'CW' if direction == 'CCW' else 'CCW'

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
        if (
            reply is not None
            and reply.arbitration_id == motor_id
            and len(reply.data) >= 7
        ):
            data = reply.data
            count = (data[1]<<40) | (data[2]<<32) | (data[3]<<24) | (data[4]<<16) | (data[5]<<8) | data[6]
            if count >= 0x800000000000:
                count -= 0x1000000000000
            return count
        return None

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        v_l = linear - (angular * self.wheel_separation / 2.0)
        v_r = linear + (angular * self.wheel_separation / 2.0)
        if self.wheel_radius > 0:
            rpm_left = (v_l / (2.0 * math.pi * self.wheel_radius)) * 60.0
            rpm_right = (v_r / (2.0 * math.pi * self.wheel_radius)) * 60.0
        else:
            rpm_left = 0.0
            rpm_right = 0.0

        RPM_DEADZONE = 1.0
        if abs(rpm_left) < RPM_DEADZONE:
            rpm_left = 0.0
        if abs(rpm_right) < RPM_DEADZONE:
            rpm_right = 0.0

        def clamp_rpm(rpm):
            if rpm > 0:
                return min(rpm, MAX_SPEED)
            else:
                return max(rpm, -MAX_SPEED)

        rpm_left = clamp_rpm(rpm_left)
        rpm_right = clamp_rpm(rpm_right)

        if rpm_left == 0.0 and rpm_right == 0.0:
            for motor_id in [0x01, 0x02]:
                try:
                    data = self.create_speed_command('CW', 0, 0, motor_id)
                    msg_can = can.Message(
                        arbitration_id=motor_id,
                        is_extended_id=False,
                        data=data,
                        dlc=len(data),
                        is_fd=False,
                        check=True
                    )
                    self.bus.send(msg_can)
                except can.CanError:
                    pass
            return

        direction_left = 'CCW' if rpm_left >= 0.0 else 'CW'
        direction_right = 'CCW' if rpm_right >= 0.0 else 'CW'
        rpm_left_abs = int(abs(rpm_left))
        rpm_right_abs = int(abs(rpm_right))
        if self.motor_1_inverted:
            direction_left = self.invert_direction(direction_left)
        if self.motor_2_inverted:
            direction_right = self.invert_direction(direction_right)
        commands = [
            (0x01, direction_left, rpm_left_abs),
            (0x02, direction_right, rpm_right_abs)
        ]
        for motor_id, direction, speed in commands:
            try:
                data = self.create_speed_command(direction, speed, DEFAULT_ACC, motor_id)
                msg_can = can.Message(
                    arbitration_id=motor_id,
                    is_extended_id=False,
                    data=data,
                    dlc=len(data),
                    is_fd=False,
                    check=True
                )
                self.bus.send(msg_can)
            except can.CanError:
                pass

    def update_odometry(self):
        left_cum = self.read_encoder_cumulative(0x01)
        time.sleep(0.02)
        right_cum = self.read_encoder_cumulative(0x02)

        if left_cum is None or right_cum is None:
            self.get_logger().warning("Brak danych z enkodera CAN")
            return

        if self.prev_left_cum is None or self.prev_right_cum is None:
            self.prev_left_cum = left_cum
            self.prev_right_cum = right_cum
            self.prev_time = self.get_clock().now()
            return

        delta_left = left_cum - self.prev_left_cum
        delta_right = right_cum - self.prev_right_cum
        if self.motor_1_inverted:
            delta_left = -delta_left
        if self.motor_2_inverted:
            delta_right = -delta_right

        # *** MNOŻENIE przez gear_ratio ***
        delta_left_rad = ((delta_left / ENCODER_PPR) * GEAR_RATIO) * 2.0 * math.pi
        delta_right_rad = ((delta_right / ENCODER_PPR) * GEAR_RATIO) * 2.0 * math.pi

        self.left_wheel_pos += delta_left_rad
        self.right_wheel_pos += delta_right_rad

        curr_time = self.get_clock().now()
        dt = (curr_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1e-3

        self.left_wheel_vel = delta_left_rad / dt
        self.right_wheel_vel = delta_right_rad / dt
        dl = delta_left_rad * self.wheel_radius
        dr = delta_right_rad * self.wheel_radius
        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_separation if self.wheel_separation > 0 else 0.0
        if abs(dc) > 1e-6:
            self.x += dc * math.cos(self.theta + dtheta / 2.0)
            self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.prev_left_cum = left_cum
        self.prev_right_cum = right_cum
        self.prev_time = curr_time

    def reset_odometry_callback(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.prev_left_cum = None
        self.prev_right_cum = None
        self.get_logger().info("Odometria zresetowana do (0, 0, 0)")
        return response

    def publisher_timer_callback(self):
        self.update_odometry()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        js.velocity = [self.left_wheel_vel, self.right_wheel_vel]
        self.joint_state_pub.publish(js)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        pose_covariance = [0.0]*36
        pose_covariance[0] = 0.0001
        pose_covariance[7] = 0.0001
        pose_covariance[35] = 0.1
        odom.pose.covariance = [float(x) for x in pose_covariance]
        v_linear = (self.left_wheel_vel + self.right_wheel_vel) * self.wheel_radius / 2.0
        v_angular = (
            (self.right_wheel_vel - self.left_wheel_vel) * self.wheel_radius / self.wheel_separation
            if self.wheel_separation > 0 else 0.0
        )
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = v_angular
        twist_covariance = [0.0]*36
        twist_covariance[0] = 0.001
        twist_covariance[7] = 0.001
        twist_covariance[35] = 0.01
        odom.twist.covariance = [float(x) for x in twist_covariance]
        self.odom_pub.publish(odom)

        # Publikacja TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
