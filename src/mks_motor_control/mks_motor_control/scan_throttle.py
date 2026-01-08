#!/usr/bin/env python3
"""
Scan Throttle Node - Redukuje częstotliwość /scan topic
Używa timer-based throttling - publikuje co określony czas (np. co 0.2s = 5 Hz)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        
        # Parametry
        self.declare_parameter('throttle_rate', 5.0)  # Hz - częstotliwość wyjściowa
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_throttled')
        
        throttle_rate = self.get_parameter('throttle_rate').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Throttle period (w sekundach)
        self.throttle_period = 1.0 / throttle_rate
        
        # QoS - BEST_EFFORT dla kompatybilności z pointcloud_to_laserscan
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriber i Publisher
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile  # Użyj BEST_EFFORT QoS dla kompatybilności
        )
        
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile  # Użyj BEST_EFFORT QoS dla kompatybilności
        )
        
        # Ostatni czas publikacji
        self.last_publish_time = self.get_clock().now()
        self.latest_scan = None
        
        # Timer - publikuje najnowszą wiadomość co throttle_period
        self.timer = self.create_timer(self.throttle_period, self.timer_callback)
        
        self.get_logger().info(
            f'Scan Throttle: {input_topic} -> {output_topic} @ {throttle_rate} Hz'
        )
    
    def scan_callback(self, msg):
        """Zapisz najnowszą wiadomość - timer będzie ją publikował"""
        self.latest_scan = msg
    
    def timer_callback(self):
        """Publish latest scan if available"""
        if self.latest_scan is not None:
            # Skopiuj wiadomość i zaktualizuj timestamp
            scan_msg = LaserScan()
            scan_msg.header = self.latest_scan.header
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = self.latest_scan.angle_min
            scan_msg.angle_max = self.latest_scan.angle_max
            scan_msg.angle_increment = self.latest_scan.angle_increment
            scan_msg.time_increment = self.latest_scan.time_increment
            scan_msg.scan_time = self.throttle_period  # Zaktualizuj scan_time
            scan_msg.range_min = self.latest_scan.range_min
            scan_msg.range_max = self.latest_scan.range_max
            scan_msg.ranges = self.latest_scan.ranges
            scan_msg.intensities = self.latest_scan.intensities
            
            self.publisher.publish(scan_msg)
            self.last_publish_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()