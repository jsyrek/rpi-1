#!/usr/bin/env python3
"""
Simple script to move robot forward by specified distance using Nav2
Usage: ros2 run mks_motor_control move_forward 0.05  # Move 5 cm forward
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import sys
import math


class MoveForward(Node):
    def __init__(self, distance):
        super().__init__('move_forward')
        self.distance = distance
        self.current_pose = None
        
        # TF buffer to get current position in map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action client for navigate_to_pose
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info(f'Waiting for navigate_to_pose action server...')
        self.action_client.wait_for_server()
        
        # Wait a bit for TF to be available
        self.get_logger().info('Waiting for TF transform map -> base_link...')
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # Get current position in map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            self.current_pose = Pose()
            self.current_pose.position.x = transform.transform.translation.x
            self.current_pose.position.y = transform.transform.translation.y
            self.current_pose.position.z = transform.transform.translation.z
            self.current_pose.orientation = transform.transform.rotation
            
        except Exception as e:
            self.get_logger().error(f'Could not get current position from TF: {e}')
            self.get_logger().error('Falling back to odom frame...')
            # Fallback: use odom frame
            self.odom_sub = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
            rclpy.spin_once(self, timeout_sec=2.0)
            if self.current_pose is None:
                self.get_logger().error('Could not get current position')
                return
        
        # Calculate target position (forward = +X in robot frame, need to account for orientation)
        # For simplicity, assume robot is facing +X in map frame
        # Get yaw from quaternion
        q = self.current_pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Move forward in robot's current direction
        target_x = self.current_pose.position.x + self.distance * math.cos(yaw)
        target_y = self.current_pose.position.y + self.distance * math.sin(yaw)
        target_z = self.current_pose.position.z
        
        # Keep same orientation
        target_orientation = self.current_pose.orientation
        
        self.get_logger().info(
            f'Moving forward {distance}m: '
            f'({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f}) -> '
            f'({target_x:.3f}, {target_y:.3f})'
        )
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = target_z
        goal_msg.pose.pose.orientation = target_orientation
        
        # Send goal
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def odom_callback(self, msg):
        """Store current position from odometry (fallback)"""
        if self.current_pose is None:
            self.current_pose = Pose()
            self.current_pose.position.x = msg.pose.pose.position.x
            self.current_pose.position.y = msg.pose.pose.position.y
            self.current_pose.position.z = msg.pose.pose.position.z
            self.current_pose.orientation = msg.pose.pose.orientation
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from navigation"""
        feedback = feedback_msg.feedback
        # Optional: log feedback
        pass
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted, navigating...')
        
        # Wait for result
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Navigation succeeded! Robot moved forward.')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
        
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run mks_motor_control move_forward <distance_in_meters>")
        print("Example: ros2 run mks_motor_control move_forward 0.05  # Move 5 cm forward")
        return
    
    try:
        distance = float(sys.argv[1])
    except ValueError:
        print(f"Error: '{sys.argv[1]}' is not a valid number")
        return
    
    node = MoveForward(distance)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
