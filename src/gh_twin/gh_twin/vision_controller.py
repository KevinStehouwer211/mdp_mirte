#!/usr/bin/env python3

import time
from pathlib import Path
from typing import Optional, Set

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray, Detection3DArray


class VisionController(Node):
    START_SERVICE  = "/flower_triangulation/start"
    FINISH_SERVICE = "/flower_triangulation/finish"
    RESET_SERVICE  = "/flower_triangulation/reset"

    def __init__(self, config_file: str = None):
        super().__init__("vision_controller_node", use_global_arguments=False)

        self._pest_class_ids: Set[str] = set()
        self._latest_detections: Optional[Detection2DArray] = None
        self._latest_3d_result: Optional[Detection3DArray] = None
        self._pests_detected = False

        if config_file:
            self._load_config(config_file)

        self._start_client  = self.create_client(Trigger, self.START_SERVICE)
        self._finish_client = self.create_client(Trigger, self.FINISH_SERVICE)
        self._reset_client  = self.create_client(Trigger, self.RESET_SERVICE)
        self.create_subscription(Detection2DArray, "/flower_perception", self._detections_cb, 10)
        self.create_subscription(Detection3DArray, "/flower_triangulation/result", self._result_cb, 10)
        self._pest_pub = self.create_publisher(Bool, "/pest_detected", 10)

        self.get_logger().info("Vision controller initialization complete")

    # ------------------------------------------------------------------ #
    def _detections_cb(self, msg: Detection2DArray):
        self._latest_detections = msg

    def _result_cb(self, msg: Detection3DArray):
        self._latest_3d_result = msg

    def _load_config(self, config_file: str):
        path = Path(config_file)
        if not path.exists():
            self.get_logger().error(f"Vision config not found: {config_file}")
            return

        self.get_logger().info(f"Loading vision config from '{config_file}'")

        with open(path, "r") as f:
            config = yaml.safe_load(f)

        pest_ids = config.get("pest_class_ids", [])
        self._pest_class_ids = {str(pid) for pid in pest_ids}

        self.get_logger().info(f"Loaded vision config: pest class IDs: {self._pest_class_ids}")

    # ------------------------------------------------------------------ #
    def _call_service(self, client, service_name: str, timeout_sec: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service not available: {service_name}")
            return False

        future = client.call_async(Trigger.Request())
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout_sec:
                self.get_logger().warn(f"Service call timed out: {service_name}")
                return False

        result = future.result()
        if result and result.success:
            self.get_logger().info(f"{service_name}: {result.message}")
            return True
        self.get_logger().warn(
            f"{service_name} failed: {result.message if result else 'no response'}"
        )
        return False

    def start_triangulation(self) -> bool:
        """Clear buffers and begin accumulating camera views for this spot."""
        self._latest_3d_result = None
        return self._call_service(self._start_client, self.START_SERVICE)

    def finish_triangulation(self) -> bool:
        """Solve and publish the final 3-D result; node goes idle until next start."""
        return self._call_service(self._finish_client, self.FINISH_SERVICE)

    def reset_triangulation(self) -> bool:
        """Abort the current episode without publishing."""
        return self._call_service(self._reset_client, self.RESET_SERVICE)

    def check_and_publish_pests(self):
        if not self._pest_class_ids:
            self.get_logger().debug("No pest_class_ids configured; skipping pest check")
            return

        msg = self._latest_detections
        if msg is None:
            self.get_logger().warn("No detections received yet; cannot check for pests")
            return

        self._pests_detected = False
        for det in msg.detections:
            for hyp in det.results:
                # class_id format from detector: "{yolo_class_id}_track_{track_id}"
                yolo_class = hyp.hypothesis.class_id.split("_track_")[0]
                if yolo_class in self._pest_class_ids:
                    self._pests_detected = True
                    self.get_logger().warn(
                        f"Pest detected: class '{hyp.hypothesis.class_id}' "
                        f"(score={hyp.hypothesis.score:.2f})"
                    )
                    break
            if self._pests_detected:
                break

        pest_msg = Bool()
        pest_msg.data = self._pests_detected
        self._pest_pub.publish(pest_msg)

        if self._pests_detected:
            self.get_logger().warn("Pest flag published on /pest_detected")
        else:
            self.get_logger().info("No pests detected in latest frame")

    @property
    def pests_detected(self) -> bool:
        return self._pests_detected
