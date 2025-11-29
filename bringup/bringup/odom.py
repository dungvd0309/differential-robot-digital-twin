#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
import math
import numpy as np

class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('diff_drive_odom')

        # === 1. Khai báo Tham số (Parameters) ===
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.2336)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint') # <--- SỬA THÀNH base_footprint
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value # Giá trị là 'base_footprint'
        
        # === 2. Trạng thái nội bộ (Internal State) ===
        self.last_time = None
        self.last_left_pos = None
        self.last_right_pos = None
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # === 3. Publishers & Subscribers ===
        self.odom_pub = self.create_publisher(Odometry, "/" + self.odom_frame, 10)

        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)

        self.get_logger().info("Diff-drive odom node started")

    def joint_cb(self, msg: JointState):
        current_time = self.get_clock().now()
        
        # Lấy dữ liệu vị trí bánh xe
        try:
            li = msg.name.index("left_wheel_joint")
            ri = msg.name.index("right_wheel_joint")
        except ValueError:
            self.get_logger().warn("Joint names not found. Check joint names in URDF.")
            return

        left_pos = msg.position[li]
        right_pos = msg.position[ri]

        # Khởi tạo lần đầu
        if self.last_time is None:
            self.last_time = current_time
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return

        # === 4. Tính toán Odometry (Calculation) ===
        d_left = (left_pos - self.last_left_pos) * self.wheel_radius
        d_right = (right_pos - self.last_right_pos) * self.wheel_radius

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation
        
        self.linear_velocity = d_center / dt
        self.angular_velocity = d_theta / dt
        
        # Tích phân để ra Pose (Vị trí)
        if abs(d_theta) < 1e-6:
            self.x += d_center * math.cos(self.yaw)
            self.y += d_center * math.sin(self.yaw)
        else:
            self.x += d_center * math.cos(self.yaw + d_theta / 2.0)
            self.y += d_center * math.sin(self.yaw + d_theta / 2.0)
            
        self.yaw += d_theta
        
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # === 5. Publish Odometry message (Pose + Twist + Covariance) ===
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame # Gán frame con là base_footprint

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Twist
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        
        # Covariance
        pose_diag = np.array([1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-3])
        twist_diag = np.array([1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-3])

        odom.pose.covariance = np.diag(pose_diag).flatten().tolist()
        odom.twist.covariance = np.diag(twist_diag).flatten().tolist()
        
        self.odom_pub.publish(odom)

        # === 6. Cập nhật trạng thái (Update state) ===
        self.last_time = current_time
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
