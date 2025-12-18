#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.get_logger().info('IMU Publisher node started')
        
        # 1. Subscriber: Lắng nghe dữ liệu thô (chưa có covariance)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
            
        # 2. Publisher: Xuất bản dữ liệu đã có covariance
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.imu_frame_id = "imu_link"

        # --- CẤU HÌNH COVARIANCE (QUAN TRỌNG) ---
        # Ma trận 3x3 trải phẳng thành mảng 9 phần tử: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        # Chúng ta chỉ quan tâm đường chéo chính: [0], [4], [8] (Variance của x, y, z)
        
        # A. Linear Acceleration Covariance (Gia tốc tuyến tính)
        # Giả sử cảm biến có độ nhiễu thấp, tin tưởng vừa phải
        self.linear_accel_cov = [
            1e-1, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 1e6
        ]

        # B. Angular Velocity Covariance (Vận tốc góc - Gyro)
        # Đây là dữ liệu quan trọng nhất để sửa hướng (Heading)
        # Đặt giá trị nhỏ (0.001) nghĩa là RẤT TIN TƯỞNG vào Gyro
        self.angular_vel_cov = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-1
        ]

        # C. Orientation Covariance (Góc nghiêng/hướng)
        # Vì bạn nói CHƯA CÓ dữ liệu này (chỉ có Accel + Gyro),
        # Ta đặt phần tử đầu tiên là -1.0 để báo cho EKF biết là "Đừng dùng cái này"
        self.orientation_cov = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]

    def imu_callback(self, msg_in: Imu):
        msg_out = Imu()
        
        # 1. Copy Header (Giữ nguyên timestamp và frame_id)
        msg_out.header.stamp = msg_in.header.stamp
        msg_out.header.frame_id = self.imu_frame_id
        
        # 2. Copy dữ liệu đo được
        msg_out.angular_velocity = msg_in.angular_velocity
        msg_out.linear_acceleration = msg_in.linear_acceleration
        msg_out.orientation = msg_in.orientation 

        # 3. Gán ma trận Covariance đã cấu hình
        msg_out.angular_velocity_covariance = self.angular_vel_cov
        msg_out.linear_acceleration_covariance = self.linear_accel_cov
        msg_out.orientation_covariance = self.orientation_cov

        self.get_logger().info(f'ax = {msg_out.linear_acceleration.x:.2f}, gz = {msg_out.angular_velocity.z:.2f}')

        # 4. Publish
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()