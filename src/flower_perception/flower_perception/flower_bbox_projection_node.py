"""
Project triangulated 3-D detections into the camera image as 3-D boxes.

How it works
------------
The triangulation node publishes a vision_msgs/Detection3DArray (one x,y,z per
flower, in `odom`).  This node keeps the latest such array and, for every raw
camera frame, draws a wireframe 3-D bounding box around each detection so you
can *see* whether the triangulated point lands on the flower.

We draw on the CLEAN camera image (the same raw `/gripper_camera/image_raw`
stream the detector consumes), not the debug image, so the only overlay is our
3-D box.

Box size comes from the matching 2-D detection: the YOLO box pixel size, scaled
to metric at the detection's depth (size_m = size_px * Z / focal).  So a bigger
flower in the image -> a bigger 3-D box.  The box is built axis-aligned in
`odom` (height along the world up-axis, so it stays upright/parallel to the
ground regardless of camera tilt), its 8 corners transformed into the camera
and projected with the intrinsics (same K, D as the rest of the pipeline).
Each box is labelled with class, track id and the x,y,z in `odom`.

View it on:
    /flower_perception/debug_image/bbox_projected
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection3DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point


# Cube corners (unit, half-edge = 1) and the 12 edges connecting them.
# Bits of the index map to (x, y, z) sign: front face 0-3, back face 4-7.
_CORNERS = np.array([
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1],
], dtype=np.float64)
_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),     # front face
    (4, 5), (5, 6), (6, 7), (7, 4),     # back face
    (0, 4), (1, 5), (2, 6), (3, 7),     # connecting edges
]

# Class index -> (display name, box color in BGR). The color matches the flower
# so the overlay reads at a glance; the pest gets green.
_CLASS_INFO = {
    "0": ("tulip_red",   (0,   0,   255)),   # red
    "1": ("tulip_white", (255, 255, 255)),   # white
    "2": ("tulip_pink",  (203, 192, 255)),   # pink
    "3": ("bug",         (0,   255, 0)),      # green
}
_UNKNOWN_CLASS = ("?", (200, 200, 200))


class FlowerBBoxProjection(Node):
    def __init__(self):
        super().__init__("flower_bbox_projection")

        # Clean raw camera stream (same one the detector consumes).
        self.declare_parameter("image_topic", "/gripper_camera/image_raw/compressed")
        # 2-D detections, used to size the 3-D box from the YOLO box.
        self.declare_parameter("detections_2d_topic", "/flower_perception")
        # Triangulated 3-D detections (x,y,z in odom).
        self.declare_parameter("detections_3d_topic", "/flower_triangulation/detections")
        self.declare_parameter(
            "output_image_topic", "/flower_perception/debug_image/bbox_projected")
        self.declare_parameter("camera_frame", "wrist_camera")
        # Detections carry their own frame_id (odom); this is only a fallback.
        self.declare_parameter("base_frame", "odom")
        # Fallback half-edge (m) when no matching 2-D box gives a size.
        self.declare_parameter("box_size", 0.06)

        # Camera intrinsics (same calibration as the rest of the pipeline).
        self.declare_parameter("fx", 752.5599)
        self.declare_parameter("fy", 752.3901)
        self.declare_parameter("cx", 301.2110)
        self.declare_parameter("cy", 212.9964)
        self.declare_parameter("k1", -0.251118)
        self.declare_parameter("k2",  0.647895)
        self.declare_parameter("p1", -0.004571)
        self.declare_parameter("p2", -0.000315)
        self.declare_parameter("k3",  0.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.detections_2d_topic = self.get_parameter("detections_2d_topic").value
        self.detections_3d_topic = self.get_parameter("detections_3d_topic").value
        self.output_image_topic = self.get_parameter("output_image_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.base_frame   = self.get_parameter("base_frame").value
        self.box_size     = float(self.get_parameter("box_size").value)

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

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_dets3d: Detection3DArray | None = None
        self.size_by_track: dict[str, tuple[float, float]] = {}   # track id -> (w_px, h_px)

        self.create_subscription(
            Detection3DArray, self.detections_3d_topic, self._dets3d_cb, 10)
        self.create_subscription(
            Detection2DArray, self.detections_2d_topic, self._dets2d_cb, 10)
        self.create_subscription(
            CompressedImage, self.image_topic, self._image_cb, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        self.get_logger().info(
            f"Projecting 3-D boxes from {self.detections_3d_topic} onto "
            f"{self.image_topic} -> {self.output_image_topic}")

    # ------------------------------------------------------------------ #
    def _dets3d_cb(self, msg: Detection3DArray):
        self.latest_dets3d = msg

    def _dets2d_cb(self, msg: Detection2DArray):
        # Map each track id to its current 2-D box pixel size.
        sizes = {}
        for det in msg.detections:
            if not det.results:
                continue
            # class_id is "<class>_track_<id>" (see detector/tracker node).
            _, _, track = det.results[0].hypothesis.class_id.partition("_track_")
            sizes[track] = (float(det.bbox.size_x), float(det.bbox.size_y))
        self.size_by_track = sizes

    # ------------------------------------------------------------------ #
    def _image_cb(self, img_msg: CompressedImage):
        frame = cv2.imdecode(np.frombuffer(img_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Could not decode compressed image")
            return

        dets = self.latest_dets3d
        if dets is not None and dets.detections:
            dets_frame = dets.header.frame_id or self.base_frame
            try:
                # Maps points from the detection frame (odom) into the camera.
                # Latest TF: the rosbag's stamps are unreliable (see lidar node).
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame, dets_frame, rclpy.time.Time())
                for det in dets.detections:
                    self._draw_box(frame, det, dets_frame, transform)
            except Exception as e:
                self.get_logger().warn(
                    f"TF {dets_frame} -> {self.camera_frame} failed: {e}")

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = img_msg.header
        self.image_pub.publish(out_msg)

    # ------------------------------------------------------------------ #
    def _draw_box(self, frame, det, dets_frame, transform):
        c = det.bbox.center.position
        center_odom = np.array([c.x, c.y, c.z])

        # Triangulated centre -> camera frame (Z forward) for the depth.
        z = self._to_cam(center_odom, dets_frame, transform)[2]
        if z <= 0.0:                                    # behind the camera
            return

        # Box half-extents in metres, from the matching 2-D box pixel size
        # back-projected at this depth: size_m = size_px * Z / focal.
        size_px = self.size_by_track.get(str(det.id))
        if size_px is not None and size_px[0] > 0 and size_px[1] > 0:
            half_w = 0.5 * size_px[0] * z / self.K[0, 0]
            half_h = 0.5 * size_px[1] * z / self.K[1, 1]
        else:
            half_w = half_h = self.box_size
        # Upright box, axis-aligned in odom: square footprint from the 2-D
        # width (X, Y), height (Z, world up) from the 2-D height. This keeps
        # the box parallel to the ground plane regardless of camera tilt.
        half = np.array([half_w, half_w, half_h])

        # Build the 8 corners in odom, then transform each into the camera and
        # project them with the intrinsics.
        cam_pts = np.array([
            self._to_cam(center_odom + corner * half, dets_frame, transform)
            for corner in _CORNERS])
        if np.all(cam_pts[:, 2] <= 0.0):
            return
        img_pts, _ = cv2.projectPoints(
            cam_pts.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.D)
        img_pts = img_pts.reshape(-1, 2)

        cls = det.results[0].hypothesis.class_id if det.results else "?"
        name, color = _CLASS_INFO.get(cls, _UNKNOWN_CLASS)
        for a, b in _EDGES:
            if cam_pts[a, 2] <= 0.0 or cam_pts[b, 2] <= 0.0:
                continue
            pa = (int(round(img_pts[a, 0])), int(round(img_pts[a, 1])))
            pb = (int(round(img_pts[b, 0])), int(round(img_pts[b, 1])))
            cv2.line(frame, pa, pb, color, 2)

        # Label: class name, track id, and the x,y,z in the detection frame (odom).
        valid = img_pts[cam_pts[:, 2] > 0.0]
        x0 = int(round(valid[:, 0].min()))
        y0 = int(round(valid[:, 1].min()))
        self._text(frame, f"{name} #{det.id}", (x0, y0 - 22), color)
        self._text(frame, f"({c.x:.2f}, {c.y:.2f}, {c.z:.2f})", (x0, y0 - 6), color)

    # ------------------------------------------------------------------ #
    def _to_cam(self, pt, frame, transform):
        """Transform a point (np array, in `frame`) into the camera frame."""
        p = PointStamped()
        p.header.frame_id = frame
        p.point.x, p.point.y, p.point.z = float(pt[0]), float(pt[1]), float(pt[2])
        pc = do_transform_point(p, transform).point
        return np.array([pc.x, pc.y, pc.z])

    # ------------------------------------------------------------------ #
    @staticmethod
    def _text(frame, text, org, color):
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)             # outline for contrast
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = FlowerBBoxProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
