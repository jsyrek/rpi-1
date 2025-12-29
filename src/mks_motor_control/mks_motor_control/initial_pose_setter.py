#!/usr/bin/env python3
"""
Initial Pose Setter Node
Allows manual setting of robot's initial position based on laser pointer location.

Usage:
  ros2 service call /set_initial_pose geometry_msgs/PoseWithCovarianceStamped
  ros2 run mks_motor_control initial_pose_setter

Example command:
  ros2 service call /set_initial_pose geometry_msgs/PoseWithCovarianceStamped \
    "pose: {pose: {position: {x: 0.30, y: 0.30, z: 0.0}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

Laser pointer offset correction:
  - Physical laser at (x_laser, y_laser) = Robot center at (x_laser + 0.30, y_laser + 0.30)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import math


class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Service to set initial pose
        self.set_pose_service = self.create_service(
            Empty,
            'set_initial_pose_manual',
            self.set_initial_pose_callback
        )
        
        # Publisher for initial pose (AMCL subscribes to this)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            1
        )
        
        # Parameters
        self.declare_parameter('initial_x', 0.30)
        self.declare_parameter('initial_y', 0.30)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('cov_x', 0.25)
        self.declare_parameter('cov_y', 0.25)
        self.declare_parameter('cov_theta', 0.068)
        
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_theta = self.get_parameter('initial_theta').value
        self.cov_x = self.get_parameter('cov_x').value
        self.cov_y = self.get_parameter('cov_y').value
        self.cov_theta = self.get_parameter('cov_theta').value
        
        self.get_logger().info(
            f"Initial Pose Setter initialized\n"
            f"Default position: x={self.initial_x}, y={self.initial_y}, "
            f"theta={self.initial_theta} rad"
        )
        self.get_logger().info(
            "LASER POINTER OFFSET: 0.30m backward, 0.30m left\n"
            "If laser points at (x_laser, y_laser), robot is at "
            "(x_laser + 0.30, y_laser + 0.30)"
        )

    def set_initial_pose_callback(self, request, response):
        """Service callback to set initial pose"""
        self.publish_initial_pose(
            self.initial_x,
            self.initial_y,
            self.initial_theta
        )
        return response

    def publish_initial_pose(self, x, y, theta):
        """Publish initial pose to AMCL"""
        # Convert theta (radians) to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        
        # Covariance (36 elements for 6DOF)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = self.cov_x * self.cov_x    # x variance
        pose_msg.pose.covariance[7] = self.cov_y * self.cov_y    # y variance
        pose_msg.pose.covariance[35] = self.cov_theta * self.cov_theta  # theta variance
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"Published initial pose: x={x:.2f}, y={y:.2f}, "
            f"theta={theta:.2f} rad ({math.degrees(theta):.1f}°)"
        )

    def set_pose_from_laser(self, laser_x, laser_y, laser_theta=0.0):
        """
        Set initial pose based on laser pointer location.
        
        Args:
            laser_x: X coordinate where laser points on the map
            laser_y: Y coordinate where laser points on the map
            laser_theta: Robot's orientation (optional)
        """
        # Apply offset correction
        robot_x = laser_x + 0.30
        robot_y = laser_y + 0.30
        
        self.publish_initial_pose(robot_x, robot_y, laser_theta)
        self.get_logger().info(
            f"Set pose from laser pointer: laser({laser_x:.2f}, {laser_y:.2f}) "
            f"-> robot({robot_x:.2f}, {robot_y:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    
    # Publish initial pose at startup
    node.publish_initial_pose(
        node.initial_x,
        node.initial_y,
        node.initial_theta
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
