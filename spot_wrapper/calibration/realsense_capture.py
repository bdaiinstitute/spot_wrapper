import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

class RSSingleImageCapture(Node):
    def __init__(self, topic_name: str = '/camera/image_raw'):
        super().__init__('rs_single_image_capture')
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.image_acquired = False
        self.cv_image = None
        self.mutex = threading.Lock()
        self.get_logger().info(f'Subscribed to {topic_name}, waiting for image...')

    def get_image(self) -> np.ndarray | None:
        if self.image_acquired:
            self.mutex.acquire()
            image_copy = self.cv_image.copy()
            self.image_acquired = False  # Reset flag for next capture
            self.mutex.release()
            return image_copy
        else:
            self.get_logger().warning('No image has been acquired yet.')
            return None
        
    def image_callback(self, msg):
        self.get_logger().info('Received image, saving...')
        try:
            self.mutex.acquire()
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_acquired = True
            self.mutex.release()
            self.get_logger().info('Image acquired by realsense camera')
            
        except Exception as e:
            self.get_logger().error(f'Error converting or saving image: {e}')
            self.cv_image = None
            self.image_acquired = False

""" def main(args=None):
    rclpy.init(args=args)
    node = SingleImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
"""