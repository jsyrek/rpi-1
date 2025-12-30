#!/usr/bin/env python3
"""
Initialize robot position on table
Sets robot at (30cm, 30cm) from corner and broadcasts TF
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class TableInitializer(Node):
    def __init__(self):
        super().__init__('table_initializer')
        self.br = TransformBroadcaster(self)
        
        # Robot position: 30cm from corner
        self.x = 0.30
        self.y = 0.30
        self.theta = 0.0
        
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.get_logger().info(
            f"✓ Robot initialized @ ({self.x*100:.0f}cm, {self.y*100:.0f}cm)"
        )
    
    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        qz = math.sin(self.theta / 2)
        qw = math.cos(self.theta / 2)
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TableInitializer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
