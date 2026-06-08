#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from pupil_apriltags import Detector
    
from gh_twin_msgs.msg import Sensor


class AprilTagNode(Node):

    def __init__(self):
        super().__init__('pupil_apriltag_node')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/gripper_camera/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Publisher for teleoperation commands
        self.tagid_pub = self.create_publisher(
            Sensor,
            '/vision/scanned_tag',
            10
        )

        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1
        )

        self.get_logger().info("Pupil AprilTag node started")

        # Camera intrinsics (REPLACE with your calibration)
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        self.tag_size = 0.05  # meters

    def image_callback(self, msg):

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        if img is None:
            return

        detections = self.detector.detect(
            img,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size
        )

        for d in detections:
            
            detected_tag = Sensor()
            detected_tag.tag_id = f'{d.tag_id}'
            #detected_tag.id = d.tag_id
            print(detected_tag)
            
            self.tagid_pub.publish(detected_tag)
            
            #t = d.pose_t  # translation
            #r = d.pose_R  # rotation matrix


            #self.get_logger().info(
            #    f"Tag {d.tag_id}")



def main():
    rclpy.init()
    node = AprilTagNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()