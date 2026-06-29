import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import tf2_ros
from scipy.spatial.transform import Rotation


class FlowerHeightEstimator(Node):
    def __init__(self):
        super().__init__("flower_height_estimator")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("detections_topic", "/flower_perception")
        self.declare_parameter("camera_frame", "wrist_camera")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ground_z_in_base_link", 0.0)
        self.declare_parameter("flower_height_prior", 0.2)   # m above ground; <0 disables
        self.declare_parameter("prior_class", "3")           # bug class id; only this uses the prior
        self.declare_parameter("fx", 752.5599)
        self.declare_parameter("fy", 752.3901)
        self.declare_parameter("cx", 301.2110)
        self.declare_parameter("cy", 212.9964)
        self.declare_parameter("k1", -0.251118)
        self.declare_parameter("k2",  0.647895)
        self.declare_parameter("p1", -0.004571)
        self.declare_parameter("p2", -0.000315)
        self.declare_parameter("k3",  0.0)
        self.declare_parameter("debug_image_topic",  "/flower_perception/debug_image")
        self.declare_parameter("height_image_topic", "/flower_perception/debug_image_with_heights")

        self.scan_topic          = self.get_parameter("scan_topic").value
        self.detections_topic    = self.get_parameter("detections_topic").value
        self.camera_frame        = self.get_parameter("camera_frame").value
        self.base_frame          = self.get_parameter("base_frame").value
        self.ground_z            = float(self.get_parameter("ground_z_in_base_link").value)
        self.flower_height_prior = float(self.get_parameter("flower_height_prior").value)
        self.prior_class         = self.get_parameter("prior_class").value
        self.debug_image_topic   = self.get_parameter("debug_image_topic").value
        self.height_image_topic  = self.get_parameter("height_image_topic").value

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.D = np.array([
            self.get_parameter("k1").value, self.get_parameter("k2").value,
            self.get_parameter("p1").value, self.get_parameter("p2").value,
            self.get_parameter("k3").value,
        ], dtype=np.float64)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge      = CvBridge()

        self.latest_scan:  LaserScan | None = None
        self.latest_image: Image     | None = None

        self.create_subscription(LaserScan,        self.scan_topic,        self._scan_cb,       10)
        self.create_subscription(Detection2DArray, self.detections_topic,  self._detections_cb, 10)
        self.create_subscription(Image,            self.debug_image_topic, self._image_cb,      10)

        self.height_pub      = self.create_publisher(Float32, "/flower_perception/flower_height", 10)
        self.height_image_pub = self.create_publisher(Image, self.height_image_topic, 10)

        self.get_logger().info(f"Camera: {self.camera_frame}  Base: {self.base_frame}")

    # ------------------------------------------------------------------ #
    #  ROS callbacks                                                       #
    # ------------------------------------------------------------------ #

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _image_cb(self, msg: Image):
        self.latest_image = msg

    def _detections_cb(self, detections_msg: Detection2DArray):
        if self.latest_scan is None or self.latest_image is None:
            return

        # Project scan into the camera frame that matches this detection batch
        projected = self._project_scan(self.latest_scan, detections_msg.header.stamp)
        if not projected:
            return

        # TF: camera → base_link (for 3-D position output)
        mat = None
        try:
            tf  = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
            mat = self._tf_to_matrix(tf)
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

        results = []
        for det in detections_msg.detections:
            result = self._estimate(det, projected, mat)
            results.append(result)
            if result[0] is not None:
                self.height_pub.publish(Float32(data=float(result[0])))

        self._draw_and_publish(detections_msg, results, projected, self.latest_image)

    # ------------------------------------------------------------------ #
    #  Height estimation                                                   #
    # ------------------------------------------------------------------ #

    def _estimate(self, detection, projected, mat_cam_to_base):
        """
        Returns (height_from_ground, pos_base, anchor_uv) or (None, None, None).

        Prior method (only for `prior_class`, e.g. "bug", whose height is
        known): the target is assumed to lie on a known horizontal plane
        z = ground_z + flower_height_prior in base_link.  The bbox-centre ray
        is intersected with that plane, which fixes the depth from geometry
        alone.  This avoids the LiDAR-depth failure where a far-away floor
        point makes a distant target look tall.

        Otherwise (flowers, varying heights): match the scan point directly
        below the bbox centre and project it onto the ray to recover depth.
        """
        if mat_cam_to_base is None:
            return None, None, None

        u_box = float(detection.bbox.center.position.x)
        v_box = float(detection.bbox.center.position.y)

        # Unproject bbox centre to a unit ray in camera frame
        ray = self._unproject_pixel(u_box, v_box)

        # --- Prior: intersect the ray with the known height plane -----------
        # Only for the class whose height is known a priori (e.g. "bug").
        use_prior = (self.flower_height_prior >= 0.0
                     and str(self._detection_label(detection)) == str(self.prior_class))
        if use_prior:
            O = mat_cam_to_base[:3, 3]                 # camera origin in base
            D = mat_cam_to_base[:3, :3] @ ray          # ray direction in base
            plane_z = self.ground_z + self.flower_height_prior
            if abs(D[2]) > 1e-6:
                s = (plane_z - O[2]) / D[2]
                if s > 0.0:                            # plane is in front of cam
                    pos_base = O + s * D
                    return self.flower_height_prior, pos_base, (int(u_box), int(v_box))

        # --- Fallback: LiDAR-depth method -----------------------------------
        # Find the best scan point: must be BELOW the bbox centre (v > v_box),
        # pick the one with the smallest horizontal distance to the bbox centre.
        best_pt, best_uv, best_u_dist = None, None, float("inf")
        for u, v, pt_cam in projected:
            if v <= v_box:
                continue                    # ignore points above the detection
            u_dist = abs(u - u_box)
            if u_dist < best_u_dist:
                best_u_dist = u_dist
                best_pt  = pt_cam           # Best 3D lidar point in camera frame
                best_uv  = (int(u), int(v))

        if best_pt is None:
            return None, None, None

        # Depth along that ray = projection of the LiDAR point onto the ray
        t = float(np.dot(best_pt, ray))
        if t <= 0.0:
            return None, None, None

        # 3-D flower position in base_link
        pos_base = (mat_cam_to_base @ np.append(t * ray, 1.0))[:3]
        height   = float(pos_base[2] - self.ground_z)

        return height, pos_base, best_uv

    # ------------------------------------------------------------------ #
    #  LiDAR projection                                                    #
    # ------------------------------------------------------------------ #

    def _project_scan(self, scan_msg: LaserScan, stamp=None):
        """
        Transform all valid scan ranges into camera frame and project to pixels.
        Returns list of (u, v, point_in_camera_frame).
        Uses the timestamp of the triggering image for the TF lookup so the
        projection stays in sync with camera motion; falls back to latest TF.
        """
        tf_time = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, scan_msg.header.frame_id, tf_time
            )
        except Exception:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame, scan_msg.header.frame_id, rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().warn(str(e), throttle_duration_sec=5.0)
                return []

        mat    = self._tf_to_matrix(transform)
        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
        valid  = np.isfinite(ranges) & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        if not np.any(valid):
            return []

        idx    = np.where(valid)[0]
        r      = ranges[idx]
        angles = scan_msg.angle_min + idx * scan_msg.angle_increment

        # Build homogeneous laser-frame points (z = 0 for 2-D scan)
        pts_laser = np.column_stack([r * np.cos(angles), r * np.sin(angles),
                                     np.zeros(len(r)),   np.ones(len(r))])
        pts_cam   = (mat @ pts_laser.T).T[:, :3]

        # Keep only points in front of the camera
        front   = pts_cam[:, 2] > 0.0
        pts_cam = pts_cam[front]
        if len(pts_cam) == 0:
            return []

        # Project with full distortion model
        img_pts, _ = cv2.projectPoints(
            pts_cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.D
        )
        u = img_pts[:, 0, 0]
        v = img_pts[:, 0, 1]

        return list(zip(u.tolist(), v.tolist(), pts_cam))

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _detection_label(detection):
        """Top-scoring class label of a Detection2D, or None.

        Handles both vision_msgs layouts: results[].hypothesis.class_id
        (Humble+) and the older results[].id.
        """
        if not detection.results:
            return None
        best = max(detection.results,
                   key=lambda r: getattr(getattr(r, "hypothesis", r), "score",
                                         getattr(r, "score", 0.0)))
        hyp = getattr(best, "hypothesis", best)
        return getattr(hyp, "class_id", getattr(best, "id", None))

    def _unproject_pixel(self, u, v) -> np.ndarray:
        """Return a unit direction vector in camera frame for pixel (u, v)."""
        undist = cv2.undistortPoints(
            np.array([[[u, v]]], dtype=np.float32), self.K, self.D, P=self.K
        )
        u_u, v_u = undist[0, 0]
        d = np.array([(u_u - self.K[0, 2]) / self.K[0, 0],
                      (v_u - self.K[1, 2]) / self.K[1, 1],
                      1.0])
        return d / np.linalg.norm(d)

    def _tf_to_matrix(self, tf_stamped) -> np.ndarray:
        """Convert a TransformStamped to a 4×4 homogeneous numpy matrix."""
        t   = tf_stamped.transform.translation
        r   = tf_stamped.transform.rotation
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        mat[:3,  3] = [t.x, t.y, t.z]
        return mat

    @staticmethod
    def _put_text_bg(img, text, org, scale, color, thickness=1):
        """Draw text with a filled black background for readability."""
        (tw, th), bl = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        x, y = org
        cv2.rectangle(img, (x - 3, y - th - 3), (x + tw + 3, y + bl + 3), (0, 0, 0), -1)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)

    # ------------------------------------------------------------------ #
    #  Visualisation                                                       #
    # ------------------------------------------------------------------ #

    def _draw_and_publish(self, detections, results, projected, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(str(e))
            return

        h, w = frame.shape[:2]

        # All scan points as small red dots
        for u, v, _ in projected:
            u, v = int(u), int(v)
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(frame, (u, v), 2, (0, 0, 255), -1)

        for det, (height, pos_base, anchor_uv) in zip(detections.detections, results):
            cx = int(round(det.bbox.center.position.x))
            cy = int(round(det.bbox.center.position.y))
            hh = int(round(det.bbox.size_y / 2))
            hw = int(round(det.bbox.size_x / 2))
            tx = cx - hw + 4           # left edge of text, inside the bbox
            ty1 = cy - hh + 18         # first text line inside bbox
            ty2 = cy - hh + 38         # second text line inside bbox

            if height is None:
                self._put_text_bg(frame, "no LiDAR", (tx, ty1), 0.45, (180, 180, 180))
                continue

            # Line from bbox centre down to the matched scan point
            if anchor_uv is not None:
                ua, va = anchor_uv
                cv2.line(frame, (cx, cy), (ua, va), (0, 165, 255), 1)
                if 0 <= ua < w and 0 <= va < h:
                    cv2.circle(frame, (ua, va), 5, (0, 165, 255), -1)

            self._put_text_bg(frame, f"h:{height:.2f}m",        (tx, ty1), 0.50, (0, 255, 0),   thickness=2)
            self._put_text_bg(frame, f"x:{pos_base[0]:.2f} y:{pos_base[1]:.2f} z:{pos_base[2]:.2f}",
                              (tx, ty2), 0.40, (0, 255, 255))

        out        = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = img_msg.header
        self.height_image_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FlowerHeightEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
