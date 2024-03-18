# import os
# import jetson.inference
# import jetson.utils
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import yaml
# from ament_index_python.packages import get_package_share_directory
# from jetson.utils import videoOutput

# class CameraPerception(Node):
#     def __init__(self):
#         super().__init__('camera_perception')
        
#         self.bridge = CvBridge()
        
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('model_name', None),
#                 ('thresholds.low', None),
#                 ('thresholds.high', None)
#         ])

#         self.net = jetson.inference.detectNet(self.get_parameter('model_name').value, threshold=self.get_parameter('thresholds.high').value)

#         self.subscription = self.create_subscription(
#             Image,
#             'camera_image',
#             self.image_callback,
#             200)
#         self.publisher_ = self.create_publisher(Image, 'camera_dets', 200)
        
#         #### Test: for streaming the output:
#         self.output = videoOutput("/dev/video0")
        

#     def image_callback(self, msg):
#         # Convert ROS image to CUDA (use jetson.utils)
#         img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         cuda_img = jetson.utils.cudaFromNumpy(img)

#         # Use DetectNet to detect objects
#         detections = self.net.Detect(cuda_img, overlay="box,labels,conf")

#         # Check if there are any detections and process them
#         if detections:
#             for detection in detections:
#                 class_id = detection.ClassID
#                 confidence = detection.Confidence
#                 # Get the name of the detected class
#                 class_name = self.net.GetClassDesc(class_id)
#                 # Log or process the detection info
#                 self.get_logger().info(f"Detected: {class_name} with confidence {confidence}")
        
#         #### Test: Try to Render use another way
#         print(self.output)
#         self.output.Render(cuda_img)
#         # self.setStatus("{:.of} FPS".format(self.net.GetNetworkFPS()))
        
        
#         # Convert CUDA image back to ROS Image message
#         output_frame = jetson.utils.cudaToNumpy(cuda_img)
#         new_msg = self.bridge.cv2_to_imgmsg(output_frame, "bgr8")

#         # Publish the new message
#         self.publisher_.publish(new_msg)
#         self.get_logger().info('Published processed image with detections')

# def main(args=None):
#     rclpy.init(args=args)
#     camera_perception = CameraPerception()
#     rclpy.spin(camera_perception)
#     camera_perception.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# import jetson.inference
# import jetson.utils
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from fusion_system_msgs.msg import Detection, DetectionArray 

# class CameraPerception(Node):
#     def __init__(self):
#         super().__init__('camera_perception')
        
#         self.bridge = CvBridge()
        
#         self.declare_parameters(
#             namespace='',
#             parameters=[
#                 ('model_name', None),
#                 ('thresholds.low', None),
#                 ('thresholds.high', None)
#         ])

#         model_name = self.get_parameter('model_name').value
#         threshold = self.get_parameter('thresholds.high').value
#         self.net = jetson.inference.detectNet(model_name, threshold=threshold)

#         self.subscription = self.create_subscription(
#             Image,
#             'camera_image',
#             self.image_callback,
#             200)
#         self.publisher_ = self.create_publisher(DetectionArray, 'camera_dets', 200)
        
#     def image_callback(self, msg):
#         img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         cuda_img = jetson.utils.cudaFromNumpy(img)

#         detections = self.net.Detect(cuda_img, overlay="box,labels,conf")

#         detection_msg = DetectionArray()
#         detection_msg.detections = []

#         for detection in detections:
#             det = Detection()
#             det.x1 = detection.Left
#             det.y1 = detection.Top
#             det.x2 = detection.Right
#             det.y2 = detection.Bottom
#             det.confidence = detection.Confidence
#             det.class_id = detection.ClassID
#             detection_msg.detections.append(det)

#         self.publisher_.publish(detection_msg)
#         self.get_logger().info('Published detection message with detections')

# def main(args=None):
#     rclpy.init(args=args)
#     camera_perception = CameraPerception()
#     rclpy.spin(camera_perception)
#     camera_perception.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from fusion_system_msgs.msg import Detection, DetectionArray
import random

class CameraPerception(Node):
    def __init__(self):
        super().__init__('camera_perception_simulator')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(DetectionArray, 'camera_dets', 200)
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize detection positions and movement directions
        self.detections = [{
            'x1': random.random() * 640,
            'y1': random.random() * 480,
            'dx': 10,  # Movement delta in x direction
            'dy': 10,  # Movement delta in y direction
            'class_id': random.randint(0, 10)
        } for _ in range(random.randint(1, 5))]  # Initialize 1 to 5 detections
        
        self.timer = self.create_timer(1.0, self.simulate_detection_callback, callback_group=self.callback_group)
        
    def simulate_detection_callback(self):
        detection_msg = DetectionArray()
        detection_msg.detections = []

        for det in self.detections:
            # Update positions
            det['x1'] += det['dx']
            det['y1'] += det['dy']
            
            # Check for boundary collisions and reverse direction if needed
            if det['x1'] > 640 or det['x1'] < 0:
                det['dx'] = -det['dx']
            if det['y1'] > 480 or det['y1'] < 0:
                det['dy'] = -det['dy']
            
            # Create detection message
            detection = Detection()
            detection.x1 = det['x1']
            detection.y1 = det['y1']
            detection.x2 = det['x1'] + 50  # Fixed width
            detection.y2 = det['y1'] + 50  # Fixed height
            detection.confidence = random.random()
            detection.class_id = det['class_id']
            detection_msg.detections.append(detection)
        
        num_detections = len(detection_msg.detections)
        self.publisher_.publish(detection_msg)
        self.get_logger().info(f'Published {num_detections} simulated detections with periodic movement')

def main(args=None):
    rclpy.init(args=args)
    perception_simulator = CameraPerception()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(perception_simulator, executor=executor)
    
    perception_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

