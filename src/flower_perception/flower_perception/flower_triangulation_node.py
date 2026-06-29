"""
Multi-view triangulation of detections (flowers / bugs) — no LiDAR, no height prior.

How it works
------------
Each detection gives one camera ray.  As the camera *moves* (move the arm or the
base) the same flower is seen from many positions, giving many rays in base_frame
(odom).  Where those rays best intersect is the 3-D point, height included.

This node accumulates rays continuously: every detection frame contributes one
ray per tracked flower (matched by the tracker's stable per-flower id) into a
rolling buffer, gated by how far the camera has moved so near-duplicate views
don't pile up.  For each flower with enough well-spread views it solves the
N-ray least-squares closest point and republishes a vision_msgs/Detection3DArray
(x,y,z + class + track id) in base_frame.  Bad views (a wrong FK pose, a tracker
id swap) are rejected by residual before re-solving, so the estimate is robust
and keeps improving as you sweep the arm.
"""

from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from vision_msgs.msg import (Detection2DArray, Detection3DArray, Detection3D,
                             ObjectHypothesisWithPose)
from geometry_msgs.msg import Pose
from gh_twin_msgs.msg import Flower, Pest
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from scipy.spatial.transform import Rotation


# gh_twin_msgs mapping: detection class index -> Flower.color. The bug class is
# published as a Pest instead of a Flower.
_FLOWER_COLOR = {"0": "red", "1": "white", "2": "pink"}
_PEST_CLASS = "3"


class FlowerTriangulation(Node):
    def __init__(self):
        super().__init__("flower_triangulation")

        self.declare_parameter("detections_topic", "/flower_perception")
        self.declare_parameter("camera_frame", "wrist_camera")
        self.declare_parameter("base_frame", "odom")    # triangulate & publish here
        self.declare_parameter("robot_frame", "base_link")  # robot body, for robot_pose
        self.declare_parameter("max_gap", 0.05)         # m; reject a ray missing the point by more
        self.declare_parameter("min_view_separation", 0.05)  # m; camera motion to log a new view
        self.declare_parameter("max_views", 150)        # rolling buffer length per flower
                                                        # (150 * min_view_separation = full sweep)
        self.declare_parameter("min_views", 2)          # min rays before we publish a point
        self.declare_parameter("min_parallax", 0.01)    # min mean eigenvalue of sum A_i
        self.declare_parameter("publish_rate", 5.0)     # Hz; how often to re-solve & publish
        self.declare_parameter("min_z", 0.16)           # m; reject points below this height in base_frame
        self.declare_parameter("bloom_min_z", 0.26)     # m; flowers above this z are reported bloomed
        self.declare_parameter("fx", 752.5599)
        self.declare_parameter("fy", 752.3901)
        self.declare_parameter("cx", 301.2110)
        self.declare_parameter("cy", 212.9964)
        self.declare_parameter("k1", -0.251118)
        self.declare_parameter("k2",  0.647895)
        self.declare_parameter("p1", -0.004571)
        self.declare_parameter("p2", -0.000315)
        self.declare_parameter("k3",  0.0)

        self.detections_topic    = self.get_parameter("detections_topic").value
        self.camera_frame        = self.get_parameter("camera_frame").value
        self.base_frame          = self.get_parameter("base_frame").value
        self.robot_frame         = self.get_parameter("robot_frame").value
        self.max_gap             = float(self.get_parameter("max_gap").value)
        self.min_view_separation = float(self.get_parameter("min_view_separation").value)
        self.max_views           = int(self.get_parameter("max_views").value)
        self.min_views           = int(self.get_parameter("min_views").value)
        self.min_parallax        = float(self.get_parameter("min_parallax").value)
        self.min_z               = float(self.get_parameter("min_z").value)
        self.bloom_min_z         = float(self.get_parameter("bloom_min_z").value)
        self.active              = False    # idle until /start
        publish_rate             = float(self.get_parameter("publish_rate").value)

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

        self.latest_detections: Detection2DArray | None = None
        # label -> deque[(origin_base, ray_base)] rolling ray buffer.
        self.tracks: dict[str, deque] = {}
        self.last_view_origin: np.ndarray | None = None   # camera pos of last logged view

        self.create_subscription(Detection2DArray, self.detections_topic,
                                 self._detections_cb, 10)
        # Live estimate (best-effort) for visualization during an episode.
        self.det3d_pub = self.create_publisher(
            Detection3DArray, "/flower_triangulation/detections", 10)
        # Final result of an episode, latched so a downstream node that subscribes
        # late still receives the last spot's detections.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.result_pub = self.create_publisher(
            Detection3DArray, "/flower_triangulation/result", latched)
        # Final per-entity results for the task scheduler: flowers (class 0-2) and
        # pests (class 3), published once per episode on finish.
        self.flower_pub = self.create_publisher(Flower, "gui_flower_data", 10)
        self.pest_pub   = self.create_publisher(Pest, "gui_pest_data", 10)

        self.create_service(Trigger, "/flower_triangulation/start", self._start_cb)
        self.create_service(Trigger, "/flower_triangulation/finish", self._finish_cb)
        self.create_service(Trigger, "/flower_triangulation/reset", self._reset_cb)
        self.create_service(Trigger, "/flower_triangulation/snapshot", self._snapshot_cb)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            "Ready (idle). Call /flower_triangulation/start, sweep the camera, then "
            "/flower_triangulation/finish to publish the result on "
            "/flower_triangulation/result.")

    # ------------------------------------------------------------------ #
    # Callback: every time we get a new Detection2DArray, capture the rays and add
    # them to the track buffers gated by camera motion.
    def _detections_cb(self, msg: Detection2DArray):
        self.latest_detections = msg
        if not self.active:
            return                  # idle between episodes: don't accumulate
        snap = self._capture(msg)
        if snap is not None:
            self._accumulate(snap)

    # ------------------------------------------------------------------ #
    # Helper: add one ray per detection to its track buffer, gated by camera motion.
    def _accumulate(self, snap):
        """Add one ray per detection to its track buffer, gated by camera motion."""
        origin = snap[0][1]
        if (self.last_view_origin is not None and
                np.linalg.norm(origin - self.last_view_origin) < self.min_view_separation):
            return   # camera hasn't moved enough; this view adds no parallax
        self.last_view_origin = origin
        for label, O, d in snap:
            if label is None:
                continue
            buf = self.tracks.get(label)
            if buf is None:
                buf = self.tracks[label] = deque(maxlen=self.max_views)
            buf.append((O, d))

    # ------------------------------------------------------------------ #
    # Timer callback: every tick, solve and publish the current best estimate for
    # every track with enough views.
    def _on_timer(self):
        if not self.active:
            return                  # idle: nothing to solve or publish
        # Every flower seen anywhere along the sweep must survive to the final
        # solve, so we never prune mid-episode (/start already cleared the buffers).
        det_array, _ = self._solve_all()
        # Publish every tick so the visualization reflects the current estimate.
        self.det3d_pub.publish(det_array)

    # ------------------------------------------------------------------ #
    # Function to solve every track with enough views, returning a Detection3DArray.
    def _solve_all(self):
        """Solve every track with enough views; return (Detection3DArray, lines)."""
        det_array = Detection3DArray()
        det_array.header.frame_id = self.base_frame
        det_array.header.stamp = self.get_clock().now().to_msg()
        lines = []
        for label, buf in self.tracks.items():
            if len(buf) < self.min_views:
                continue
            origins = np.array([o for o, _ in buf])
            dirs    = np.array([d for _, d in buf])
            point, residual, n_in = self._solve_rays(origins, dirs)
            if point is None:
                continue
            if point[2] < self.min_z:            # below the floor -> spurious
                continue
            det_array.detections.append(
                self._to_detection3d(det_array.header, label, point))
            lines.append(f"    {label:<16} {point[0]:6.3f} {point[1]:6.3f} "
                         f"{point[2]:6.3f}  views={n_in}/{len(buf)} res={residual:.3f}")
        return det_array, lines

    # ------------------------------------------------------------------ #
    # Function to solve N rays robustly by iteratively rejecting outliers with large
    # residuals. Returns (point, mean_residual, n_inliers), or (None, ...) if 
    # there isn't enough parallax or too few inliers survive.
    def _solve_rays(self, origins, dirs):
        """Robust N-ray closest point."""
        idx = np.arange(len(origins))
        # Iterate 5 times at most, which is enough for the few outliers we expect. 
        # In each loop we solve all the current rays, compute residuals, 
        # drop outliers, and re-solve.
        for _ in range(5):                       # solve / reject / re-solve
            O = origins[idx]
            D = dirs[idx]
            # A gives the squared distance from the point to each ray: d_i is the ray direction,
            # so A_i = I - d_i d_i^T projects onto the plane normal to the ray, and 
            # ||A_i (point - O_i)|| is the distance from the point to that ray.
            A = np.eye(3)[None] - D[:, :, None] * D[:, None, :]

            # Least squares parameters for the point that minimizes the mean 
            # squared distance to the rays:
            M = A.sum(0)
            b = np.einsum("nij,nj->i", A, O)

            # Smallest eigenvalue of the (mean) normal matrix = parallax health.
            # If it's too small, the rays are nearly parallel and the solution is unreliable.
            lam_min = float(np.linalg.eigvalsh(M)[0])
            if lam_min / len(idx) < self.min_parallax:
                return None, float("inf"), 0

            # Minimize mean squared distance to the rays: M point = b.
            point = np.linalg.solve(M, b)

            # Per-ray residual = distance from the point to that ray.
            # Keep rays with residual below the threshold; if too few survive, give up.
            perp = np.einsum("nij,nj->ni", A, point[None] - O)
            res = np.linalg.norm(perp, axis=1)
            keep = res <= self.max_gap
            if keep.all():
                return point, float(res.mean()), len(idx)
            if keep.sum() < self.min_views:
                return None, float("inf"), 0
            idx = idx[keep]                      # drop outliers, solve again
        return point, float(res[keep].mean()), int(keep.sum())

    # ------------------------------------------------------------------ #
    # All the service callbacks: start, finish, reset, and snapshot. Start/finish toggle
    # the active state for accumulation; finish also publishes the final result; reset
    # clears buffers and goes idle without publishing; snapshot logs the current estimates.
    def _start_cb(self, request, response):
        """Begin a fresh accumulation episode at a new spot."""
        self._clear()
        self.active = True
        response.success = True
        response.message = ("Started: accumulating views. Sweep the camera, then "
                            "call /flower_triangulation/finish.")
        self.get_logger().info(response.message)
        return response

    def _finish_cb(self, request, response):
        """Solve once, publish the final result (latched), and go idle."""
        det_array, lines = self._solve_all()
        self.active = False
        n = len(det_array.detections)
        # Publish on both: result (latched, for the downstream node) and the live
        # topic (so the visualization shows the final estimate).
        self.result_pub.publish(det_array)
        self.det3d_pub.publish(det_array)
        n_flowers, n_pests = self._publish_entities(det_array)
        if lines:
            self.get_logger().info(
                "\n  final estimates (label  x  y  z  views  res):\n" + "\n".join(lines))
        response.success = n > 0
        response.message = (f"Finished: {n} detection(s) on /flower_triangulation/result; "
                            f"{n_flowers} on flower_data, {n_pests} on pest_data.")
        self.get_logger().info(response.message)
        return response

    def _reset_cb(self, request, response):
        """Abort the current episode: clear buffers, go idle, publish nothing."""
        self._clear()
        self.active = False
        response.success = True
        response.message = "Aborted/cleared; idle."
        self.get_logger().info(response.message)
        return response

    def _snapshot_cb(self, request, response):
        """Debug: log the current per-flower x,y,z table mid-episode."""
        det_array, lines = self._solve_all()
        n = len(det_array.detections)
        if n == 0:
            response.success = False
            response.message = ("No flowers solved yet. Sweep so each flower is seen "
                                "from a few separated viewpoints.")
            self.get_logger().warn(response.message)
            return response
        self.get_logger().info(
            "\n  current multi-view estimates (label  x  y  z  views  res):\n"
            + "\n".join(lines))
        response.success = True
        response.message = f"{n} flower(s) solved; see log for the x,y,z table."
        return response

    # Helper: clear all tracks and views, ready for a new episode.
    def _clear(self):
        self.tracks.clear()
        self.last_view_origin = None

    # ------------------------------------------------------------------ #
    # Helper: capture detections as (label, origin_base, ray_base) list, 
    # where the ray is in base_frame (odom) so it's ready for triangulation. 
    def _capture(self, det_msg: Detection2DArray):
        """Snapshot detections as (label, origin_base, ray_base) list."""
        if det_msg is None or not det_msg.detections:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time.from_msg(det_msg.header.stamp))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")
                return None

        mat = self._tf_to_matrix(tf)
        origin = mat[:3, 3]
        snap = []
        for det in det_msg.detections:
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)
            ray_cam = self._unproject_pixel(u, v)
            ray_base = mat[:3, :3] @ ray_cam
            snap.append((self._label(det), origin, ray_base))
        return snap

    # ------------------------------------------------------------------ #
    @staticmethod
    def _label(detection):
        # class_id is "<class>_track_<id>" (see flower_detector_tracker_node). The
        # tracker keeps the id stable across frames, so matching on the full id
        # pins each ray to the same physical flower across all views.
        return detection.results[0].hypothesis.class_id if detection.results else None

    # ------------------------------------------------------------------ #
    @staticmethod
    def _to_detection3d(header, label, point) -> Detection3D:
        """Build a Detection3D with only the fields the pipeline consumes:
        position in bbox.center, class + score in the hypothesis, track id in
        Detection3D.id. label is "<class>_track_<id>"."""
        cls, _, track = label.partition("_track_")

        det = Detection3D()
        det.header = header
        det.id = track                          # track id, e.g. "2"
        det.bbox.center.position.x = float(point[0])
        det.bbox.center.position.y = float(point[1])
        det.bbox.center.position.z = float(point[2])
        # Identity orientation: the box axes are aligned with the odom frame.
        det.bbox.center.orientation.w = 1.0

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = cls           # class, e.g. "0"
        hyp.hypothesis.score = 1.0
        # Carry the odom coordinates in the hypothesis pose as well.
        hyp.pose.pose.position = det.bbox.center.position
        hyp.pose.pose.orientation.w = 1.0       # identity: aligned with odom
        det.results.append(hyp)
        return det

    # ------------------------------------------------------------------ #
    def _publish_entities(self, det_array):
        """Publish the final detections as gh_twin_msgs: flowers (class 0-2) on
        flower_data, pests (class 3) on pest_data. Returns (n_flowers, n_pests)."""
        robot_pose = self._robot_pose()
        n_flowers = n_pests = 0
        for det in det_array.detections:
            cls = det.results[0].hypothesis.class_id if det.results else ""
            p = det.bbox.center.position
            try:
                track_id = int(det.id)
            except (TypeError, ValueError):
                track_id = 0

            if cls in _FLOWER_COLOR:
                msg = Flower()
                msg.id = track_id
                msg.location.x, msg.location.y, msg.location.z = p.x, p.y, p.z
                msg.color = _FLOWER_COLOR[cls]
                msg.bloomed = p.z > self.bloom_min_z   # taller flowers are bloomed
                if robot_pose is not None:
                    msg.robot_pose = robot_pose
                self.flower_pub.publish(msg)
                n_flowers += 1
            elif cls == _PEST_CLASS:
                msg = Pest()
                msg.id = track_id
                msg.location.x, msg.location.y, msg.location.z = p.x, p.y, p.z
                msg.sprayed = False
                if robot_pose is not None:
                    msg.robot_pose = robot_pose
                self.pest_pub.publish(msg)
                n_pests += 1
        return n_flowers, n_pests

    def _robot_pose(self) -> Pose | None:
        """Current robot body pose in base_frame (odom), for the msgs' robot_pose."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.robot_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f"robot_pose TF {self.base_frame}->{self.robot_frame} failed: {e}")
            return None
        pose = Pose()
        t, r = tf.transform.translation, tf.transform.rotation
        pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
        pose.orientation = r
        return pose

    # Helper: unproject pixel (u,v) in the camera frame to a unit ray.
    # A unit ray is a direction vector in 3D space with length 1.
    def _unproject_pixel(self, u, v) -> np.ndarray:
        undist = cv2.undistortPoints(
            np.array([[[u, v]]], dtype=np.float32), self.K, self.D, P=self.K)
        u_u, v_u = undist[0, 0]
        d = np.array([(u_u - self.K[0, 2]) / self.K[0, 0],
                      (v_u - self.K[1, 2]) / self.K[1, 1],
                      1.0])
        return d / np.linalg.norm(d)

    # Helper: convert a TFStamped to a 4x4 homogeneous transformation matrix.
    @staticmethod
    def _tf_to_matrix(tf_stamped) -> np.ndarray:
        t   = tf_stamped.transform.translation
        r   = tf_stamped.transform.rotation
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        mat[:3,  3] = [t.x, t.y, t.z]
        return mat


def main(args=None):
    rclpy.init(args=args)
    node = FlowerTriangulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
