#!/usr/bin/env python3
"""
Hybrid localization: odometry + LiDAR edge refinement
Combines wheel odometry with LiDAR edge detection for better precision
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np


class HybridLocalization(Node):
    def __init__(self):
        super().__init__('hybrid_localization')
        
        self.br = TransformBroadcaster(self)
        
        # Robot state
        self.x = 0.30
        self.y = 0.30
        self.theta = 0.0
        
        # Table bounds
        self.table_width = 2.0
        self.table_height = 1.5
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.get_logger().info("✓ Hybrid localization started")
        self.get_logger().info(f"✓ Table: {self.table_width}m x {self.table_height}m")
    
    def odom_callback(self, msg):
        """Update position from odometry"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract theta from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2 * math.atan2(qz, qw)
        
        # Clamp to table bounds
        self.x = max(0.0, min(self.table_width, self.x))
        self.y = max(0.0, min(self.table_height, self.y))
    
    def scan_callback(self, msg):
        """Process LiDAR scan for edge detection"""
        if not hasattr(msg, 'ranges') or len(msg.ranges) < 2:
            return
        
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0)
        
        # Detect edges (discontinuities in range data)
        diffs = np.diff(ranges)
        edge_indices = np.where(np.abs(diffs) > 0.15)[0]
        
        if len(edge_indices) > 0:
            for idx in edge_indices[:1]:  # Process first edge
                angle = msg.angle_min + idx * msg.angle_increment
                distance = ranges[idx]
                
                # Convert to world coordinates
                dx = distance * math.cos(angle + self.theta)
                dy = distance * math.sin(angle + self.theta)
                
                detected_x = self.x + dx
                detected_y = self.y + dy
                
                # Refine position if close to known table edges
                if abs(detected_y - self.table_height) < 0.2:
                    self.y = self.table_height - 0.05
                elif abs(detected_y) < 0.2:
                    self.y = 0.05
                elif abs(detected_x - self.table_width) < 0.2:
                    self.x = self.table_width - 0.05
                elif abs(detected_x) < 0.2:
                    self.x = 0.05
    
    def broadcast_transform(self):
        """Broadcast current position as TF transform"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        qz = math.sin(self.theta / 2)
        qw = math.cos(self.theta / 2)
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = HybridLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
