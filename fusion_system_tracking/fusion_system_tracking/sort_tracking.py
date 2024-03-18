import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from fusion_system_msgs.msg import DetectionArray, Detection
import numpy as np
from .sort import Sort

class TrackNode(Node):
    def __init__(self):
        super().__init__('track_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            DetectionArray,
            'camera_dets',
            self.detection_callback,
            200)
        # initialize sort
        self.sort_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.3)

    def detection_callback(self, msg):
        # convert DetectionArray to sort structure
        detections = np.array([
            [detection.x1, detection.y1, detection.x2, detection.y2, detection.confidence]
            for detection in msg.detections
        ])

        # update 
        tracks = self.sort_tracker.update(detections)

        # print result to console
        for track in tracks:
            bbox = track[:4]  
            track_id = int(track[4])  
            self.get_logger().info(f'Track ID: {track_id}, BBox: {bbox}')

def main(args=None):
    rclpy.init(args=args)
    track_node = TrackNode()
    rclpy.spin(track_node)
    track_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
