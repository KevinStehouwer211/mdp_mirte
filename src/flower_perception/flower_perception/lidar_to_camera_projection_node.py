import math
import copy

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge

import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

from message_filters import Subscriber, ApproximateTimeSynchronizer


class LidarToCameraProjection(Node):
    def __init__(self):
        super().__init__("lidar_to_camera_projection")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("debug_image_topic", "/flower_perception/debug_image")
        self.declare_parameter(
            "projected_image_topic",
            "/flower_perception/debug_image/lidar_projected_image",
        )
        self.declare_parameter("camera_frame", "wrist_camera")


        # Your rosbag has scan timestamps about 34 s ahead of image timestamps.
        self.declare_parameter("scan_time_offset", 34.0)
        # Synchronizer settings (Cam publish at 30 Hz, LiDAR at 10 Hz)
        self.declare_parameter("sync_slop", 0.08)       # Difference in publish times for synchronization
        self.declare_parameter("sync_queue_size", 5)    # How many msg get stored while sync

        # Camera intrinsics
        self.declare_parameter("fx", 752.5599)
        self.declare_parameter("fy", 752.3901)
        self.declare_parameter("cx", 301.2110)
        self.declare_parameter("cy", 212.9964)
        self.declare_parameter("k1", -0.251118)
        self.declare_parameter("k2", 0.647895)
        self.declare_parameter("p1", -0.004571)
        self.declare_parameter("p2", -0.000315)
        self.declare_parameter("k3", 0.0)


        self.scan_topic = self.get_parameter("scan_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.projected_image_topic = self.get_parameter("projected_image_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.scan_time_offset = float(self.get_parameter("scan_time_offset").value)
        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        k1 = float(self.get_parameter("k1").value)
        k2 = float(self.get_parameter("k2").value)
        p1 = float(self.get_parameter("p1").value)
        p2 = float(self.get_parameter("p2").value)
        k3 = float(self.get_parameter("k3").value)

        # Camera intrinsics matrix and distortion coefficients
        self.K = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        
        # Publisher for corrected scan timestamp
        self.corrected_scan_pub = self.create_publisher(
            LaserScan,
            "/scan_corrected",
            10,
        )

        # Normal ROS subscription to correct /scan timestamps
        self.raw_scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_correction_callback,
            10,
        )

        # Message filter subscribers
        self.scan_sub = Subscriber(self, LaserScan, "/scan_corrected")
        self.image_sub = Subscriber(self, Image, self.debug_image_topic)

        # Direct image subscription: always forward incoming debug images
        # to the projected topic, even when LiDAR sync/projection is unavailable.
        self.image_passthrough_sub = self.create_subscription(
            Image,
            self.debug_image_topic,
            self.image_passthrough_callback,
            10,
        )

        
        # Approximate time synchronizer for corrected scans and images
        self.sync = ApproximateTimeSynchronizer(
            [self.scan_sub, self.image_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.synced_callback)

        self.image_pub = self.create_publisher(
            Image,
            self.projected_image_topic,
            10,
        )

        self.get_logger().info("LiDAR to camera projection node started")
        self.get_logger().info(f"Raw scan topic: {self.scan_topic}")
        self.get_logger().info("Corrected scan topic: /scan_corrected")
        self.get_logger().info(f"Image topic: {self.debug_image_topic}")
        self.get_logger().info(f"Output image topic: {self.projected_image_topic}")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")
        self.get_logger().info(f"Scan time offset: {self.scan_time_offset:.3f} s")
        self.get_logger().info(f"Sync slop: {self.sync_slop:.3f} s")

    def image_passthrough_callback(self, img_msg: Image):
        # Convert, overlay plane if available, and publish
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Could not convert passthrough image: {e}")
            self.image_pub.publish(img_msg)
            return
        # (No plane overlay in this version)
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = img_msg.header
        self.image_pub.publish(out_msg)

    # This callback corrects the scan timestamps and republishes them for synchronization
    def scan_correction_callback(self, scan_msg: LaserScan):
        corrected_scan = copy.deepcopy(scan_msg)
        scan_time = rclpy.time.Time.from_msg(scan_msg.header.stamp)
        corrected_time = scan_time - Duration(seconds=self.scan_time_offset)
        corrected_scan.header.stamp = corrected_time.to_msg()
        self.corrected_scan_pub.publish(corrected_scan)

    # This callback receives synchronized corrected scans and images, 
    # projects the LiDAR points onto the image, and publishes the debug image.
    def synced_callback(self, scan_msg: LaserScan, img_msg: Image):
        # Always keep the debug image stream alive, even if LiDAR projection fails.
        fallback_msg = img_msg

        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Could not convert image: {e}")
            self.image_pub.publish(fallback_msg)
            return

        # Use latest TF for this broken-time rosbag.
        # Do not use scan_msg.header.stamp here, because it is timestamp-corrected
        # only for synchronization, while the TF buffer still uses original bag TF time.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                scan_msg.header.frame_id,
                rclpy.time.Time(),
                #timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not get TF {scan_msg.header.frame_id} -> {self.camera_frame}: {e}"
            )
            self.image_pub.publish(fallback_msg)
            return

        cam_points = []

        for i, r in enumerate(scan_msg.ranges):
            if not math.isfinite(r):
                continue

            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            p_laser = PointStamped()
            p_laser.header = scan_msg.header
            p_laser.point.x = float(r * math.cos(angle))
            p_laser.point.y = float(r * math.sin(angle))
            p_laser.point.z = 0.0

            try:
                p_cam = do_transform_point(p_laser, transform)
            except Exception:
                continue

            # Camera projection frame convention:
            # X = right, Y = down, Z = forward
            if p_cam.point.z <= 0.0:
                continue

            cam_points.append(
                [
                    p_cam.point.x,
                    p_cam.point.y,
                    p_cam.point.z,
                ]
            )

        try:
            if cam_points:
                pts3d = np.array(cam_points, dtype=np.float64).reshape(-1, 1, 3)

                img_pts, _ = cv2.projectPoints(
                    pts3d,
                    np.zeros(3),
                    np.zeros(3),
                    self.K,
                    self.D,
                )

                for pt in img_pts:
                    u = int(round(pt[0][0]))
                    v = int(round(pt[0][1]))

                    if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:
                        cv2.circle(frame, (u, v), 3, (0, 0, 255), -1)

            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out_msg.header = img_msg.header
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"LiDAR projection failed, publishing original debug image: {e}")
            self.image_pub.publish(fallback_msg)

    # (plane visualization removed in this revert)


def main(args=None):
    rclpy.init(args=args)
    node = LidarToCameraProjection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()