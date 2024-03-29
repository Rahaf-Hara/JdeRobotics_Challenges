#!/usr/bin/env python3

"""
Visualizes object detections from an RGBD camera stream by overlaying bounding
boxes, class names, confidence scores, and object distances on the video feed.
The visualized images are then published as a new ROS 2 topic.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import ament_index_python.packages
from .object_detection import ObjectDetection


class DetectionVisualizer(Node):
    """
    A class to visualize detection results on images from an RGBD camera,
    including drawing bounding boxes and displaying class names, confidence
    levels, and distances from the camera.
    """
    def __init__(self):
        super().__init__('detection_visualizer')
        self.get_logger().info('Detection node started')

        # Load object detection configuration
        config_path = ament_index_python.packages.get_package_share_directory(
            'gps_nav2') + '/config/object_detection_params.yaml'
        self.object_detector = ObjectDetection(config_path)
        self.bridge = CvBridge()

        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image, '/intel_realsense_r200_depth/image_raw',
            self.image_callback, 10)
        self.depth_subscriber = self.create_subscription(
            Image, '/intel_realsense_r200_depth/depth/image_raw',
            self.depth_callback, 10)

        # Publisher
        self.image_publisher = self.create_publisher(
            Image, '/visualized_detections', 10)
        self.latest_depth_image = None

    def depth_callback(self, msg):
        """Handles new depth image messages."""
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def image_callback(self, msg):
        """Processes incoming image messages for object detection."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = self.object_detector.extract_detections(
            self.object_detector.predict(cv_image))
        visualized_image = self.visualize_detections(cv_image, detections)
        self.image_publisher.publish(
            self.bridge.cv2_to_imgmsg(visualized_image, "bgr8"))

    def visualize_detections(self, image, detections):
        """
        Draws bboxes, class names, distances, and confidence on the image.

        Args:
            image: The original image as a numpy array.
            detections: A list of detection results.

        Returns:
            A numpy array of the image with visualizations.
        """
        if self.latest_depth_image is None:
            self.get_logger().warn('Depth image not yet received.')
            return image

        color_array = self.object_detector.generate_color()
        for det in detections:
            class_name, confidence, bounds, class_id = det
            distance = self.latest_depth_image[int(bounds[1]), int(bounds[0])]
            label = f"{class_name}: {confidence*100:.2f}% Distance: {distance:.2f}m"
            cv2.rectangle(image, (int(bounds[0] - bounds[2]/2), int(bounds[1] - bounds[3]/2)),
                          (int(bounds[0] + bounds[2]/2), int(bounds[1] + bounds[3]/2)),
                          color_array[class_id], 2)
            cv2.putText(image, label, (int(bounds[0] - bounds[2]/2), int(bounds[1] - bounds[3]/2 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return image


def main(args=None):
    rclpy.init(args=args)
    detection_visualizer = DetectionVisualizer()
    rclpy.spin(detection_visualizer)
    detection_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
