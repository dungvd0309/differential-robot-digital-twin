#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Đăng ký nhận thông điệp Odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom', 
            self.odom_callback,
            10
        )
        self.get_logger().info("Odom TF Broadcaster node started")

    def odom_callback(self, msg: Odometry):
        # Tạo thông điệp TransformStamped
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        # Lấy frame_id ('odom') và child_frame_id ('base_footprint') trực tiếp từ message Odometry
        t.header.frame_id = msg.header.frame_id 
        t.child_frame_id = msg.child_frame_id  

        # Chuyển Pose từ Odometry sang Transform
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        # Phát TF odom -> base_footprint
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
