#!/usr/bin/env python3
"""
Fake Odometry Node - Publikuje podstawową odometrię gdy motor_driver nie działa
Używany jako fallback dla SLAM Toolbox Message Filter (wymaga synchronizacji scan z odom)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        
        # Parametry
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Stan robota (statyczny - robot stoi)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Timer - publikuje odometrię
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Fake Odometry Node started: {self.odom_frame_id} -> {self.base_frame_id} @ {publish_rate} Hz'
        )
    
    def timer_callback(self):
        """Publish odometry and TF"""
        now = self.get_clock().now()
        
        # Publikuj Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Pozycja
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientacja (quaternion)
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        
        # Prędkość (zerowa - robot stoi)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Covariance (wysoka niepewność - to jest fake odom)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        
        self.odom_pub.publish(odom_msg)
        
        # Publikuj TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdom()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()