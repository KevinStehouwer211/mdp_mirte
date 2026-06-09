from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory


class FlowerDetector(Node):
    def __init__(self):
        super().__init__("flower_detector")

        # Directory of the package
        package_dir = Path(get_package_share_directory("flower_perception"))

        # Root of best model
        self.declare_parameter(
            "model_path",
            str(package_dir / "models" / "best.pt"),
        )
        # Topic of gripper camera compressed image
        self.declare_parameter(
            "image_topic",
            "/gripper_camera/image_raw/compressed",
        )
        # Topic to publish detections
        self.declare_parameter(
            "detections_topic",
            "/flower_perception",
        )
        # Topic to publish debug image with detections drawn
        self.declare_parameter(
            "debug_image_topic",
            "/flower_perception/debug_image",
        )
        # Confidence threshold for detections (best value from validation set)
        self.declare_parameter(
            "confidence_threshold",
            0.43,
        )

        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Subscribe to compressed image topic
        # We use CompressedImage to save bandwidth
        self.image_subscription = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            10,
        )

        # Publisher for detections
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            detections_topic,
            10,
        )

        # Publisher for debug image with detections drawn
        # We use Image instead of CompressedImage to avoid compression artifacts
        # And computing happens locally, so bandwidth is not a concern
        self.debug_image_publisher = self.create_publisher(
            Image,
            debug_image_topic,
            10,
        )

        self.get_logger().info("Flower detector started")
        self.get_logger().info(f"Model: {model_path}")
        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info(f"Publishing detections to: {detections_topic}")
        self.get_logger().info(f"Publishing debug image to: {debug_image_topic}")

    # Callback for incoming compressed images
    def image_callback(self, msg: CompressedImage):
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Check if the image was decoded successfully
        if frame is None:
            self.get_logger().warn("Could not decode compressed image")
            return

        # Run YOLO model on the frame
        result = self.model(
            frame,
            conf=self.confidence_threshold,
            verbose=False,
        )[0]

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        # Process each detection and convert to Detection2D format
        for box in result.boxes:
            # Get confidence and class ID for the detection
            # We need index 0 since YOLO outputs as (N, ) tensors
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])

            # Get bounding box x,y center coordinates and width, height
            x, y, w, h = box.xywh[0].tolist()

            detection = Detection2D()
            detection.header = msg.header

            # Set bounding box center and size
            detection.bbox.center.position.x = float(x)
            detection.bbox.center.position.y = float(y)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)

            # Create ObjectHypothesisWithPose with class ID and confidence
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = confidence

            # Add hypothesis to results array of detection message
            detection.results.append(hypothesis)
            # Add detection to detections array of Detection2DArray message
            detections_msg.detections.append(detection)

        self.detection_publisher.publish(detections_msg)

        # Create debug image with detections drawn from YOLO result
        annotated_frame = result.plot()
        debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        debug_msg.header = msg.header

        self.debug_image_publisher.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FlowerDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()