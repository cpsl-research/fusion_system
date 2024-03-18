import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 200)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()

        # initilize the webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            exit(1)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # change opencv to image message
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image')
        else:
            self.get_logger().error('Failed to capture image')

    def __del__(self):
        # release resources
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_driver = CameraDriver()
    rclpy.spin(camera_driver)
    camera_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()