#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

base_wheel_track = 0.213

class HardwareBridge(Node):
    
    def __init__(self):
        super().__init__("hardware_bridge")
        self.joint_state_publisher_ = self.create_publisher(
            JointState, "/joint_states", 10)
        
        self.twist_publisher_ = self.create_publisher(
            Twist, "/cmd_vel", 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to the input joint states, e.g., from a simulation or another driver
        self.joint_state_subscriber_ = self.create_subscription(
            JointState, "/hardware_states", self.joint_state_callback, qos_profile)
        self.get_logger().info("hardware_bridge has been started")

    def joint_state_callback(self, msg: JointState):
        # Update the timestamp to the current time
        msg.header.stamp = self.get_clock().now().to_msg()  
        # Republish the message on the /joint_states topic
        self.joint_state_publisher_.publish(msg)
        self.get_logger().debug(f"Relayed JointState with new timestamp: {msg.header.stamp}")

        

        left_vel = msg.velocity[0]
        right_vel = msg.velocity[1]
        linear_x = (right_vel + left_vel) / 2
        angular_z = (right_vel - left_vel) / base_wheel_track

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.twist_publisher_.publish(twist_msg)

        # self.get_logger().info(f"left_vel: {left_vel}, right_vel: {right_vel}")
        self.get_logger().info(f"linear_x: {linear_x}, angular_z: {angular_z}")

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()