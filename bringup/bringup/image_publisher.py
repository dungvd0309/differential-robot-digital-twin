import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/esp32_cam/image/compressed',
            self.listener_callback,
            qos_sub)

        self.publisher = self.create_publisher(
            Image, 
            '/camera/image', 
            qos_pub)

        self.get_logger().info("image_publisher node started.")

    def listener_callback(self, msg):
        try:
            # Decode
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn("Empty image received!")
                return

            # Xoay anh
            rotated_img = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Dong goi anh vao msg
            rows, cols, channels = rotated_img.shape

            img_msg = Image()
            
            img_msg.header = msg.header
            img_msg.header.frame_id = "esp32_cam_link" 

            img_msg.height = rows
            img_msg.width = cols
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = 0
            img_msg.step = cols * channels # step: số byte của một hàng
            img_msg.data = rotated_img.tobytes() # Chuyển dữ liệu Numpy thành Bytes

            # Publish
            self.publisher.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()