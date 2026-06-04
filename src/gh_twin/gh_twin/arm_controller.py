#!/usr/bin/env python3

import enum
import time
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class ArmExitCode(enum.IntEnum):
    INIT_COMPLETE  = 0x60
    INIT_FAILED    = 0x61
    GOAL_SET       = 0x62
    MOVING         = 0x63
    SUCCEEDED      = 0x64
    FAILED         = 0x65
    INVALID_GOAL   = 0x66
    INVALID_JOINTS = 0x67
    UNKNOWN_POSE   = 0x68
    TIMEOUT        = 0x69


class ArmController(Node):
    ACTION_NAME = "/mirte_master_arm_controller/follow_joint_trajectory"

    def __init__(self, config_file: str = None):
        super().__init__("arm_controller_node")

        self._poses: Dict[str, Dict] = {}
        self._joint_names: List[str] = []
        self._motion_duration_sec: float = 3.0
        self._init_complete = False
        self.ARM_TIMEOUT = 30.0

        if config_file:
            self._load_config(config_file)

        self._action_client = ActionClient(self, FollowJointTrajectory, self.ACTION_NAME)

        self.get_logger().info(f"Waiting for arm action server '{self.ACTION_NAME}'...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Arm action server not available: '{self.ACTION_NAME}'")
            return

        self._init_complete = True
        self.get_logger().info("Arm controller initialization complete")

    def _load_config(self, config_file: str):
        path = Path(config_file)
        if not path.exists():
            self.get_logger().error(f"Arm config not found: {config_file}")
            return

        self.get_logger().info(f"Loading arm config from '{config_file}'")

        with open(path, "r") as f:
            config = yaml.safe_load(f)

        self._joint_names = config.get("joint_names", [])
        self._motion_duration_sec = float(config.get("motion_duration_sec", 3.0))

        self.get_logger().info(
            f"Loaded arm config: {len(self._joint_names)} joints, default motion duration {self._motion_duration_sec:.1f} sec"
        )

        for pose_name, pose_data in config.get("poses", {}).items():
            self._poses[pose_name] = pose_data

        self.get_logger().info(
            f"Loaded {len(self._poses)} arm poses: {list(self._poses.keys())}"
        )

    def move_arm_to_pose(self, pose_name: str) -> ArmExitCode:
        if not self._init_complete:
            self.get_logger().error("Arm controller not initialized")
            return ArmExitCode.INIT_FAILED

        if pose_name not in self._poses:
            self.get_logger().error(
                f"Unknown arm pose '{pose_name}'. Available: {list(self._poses.keys())}"
            )
            return ArmExitCode.UNKNOWN_POSE

        self.get_logger().info(f"Sending arm to pose '{pose_name}'")
        goal_msg = self._build_goal(pose_name)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )

        # Spin until goal acceptance, mirroring the nav_to_pose spin-once loop.
        start = time.monotonic()
        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > self.ARM_TIMEOUT:
                self.get_logger().error("Timed out waiting for goal acceptance")
                return ArmExitCode.TIMEOUT

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for pose '{pose_name}'")
            return ArmExitCode.INVALID_GOAL

        self.get_logger().info(f"Goal accepted, executing '{pose_name}'")

        result_future = goal_handle.get_result_async()

        start = time.monotonic()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > self.ARM_TIMEOUT:
                self.get_logger().error(f"Timed out waiting for result for pose '{pose_name}'")
                return ArmExitCode.TIMEOUT

        return self._interpret_result(pose_name, result_future.result().result)

    def _build_goal(self, pose_name: str) -> FollowJointTrajectory.Goal:
        pose_data = self._poses[pose_name]
        duration_float = float(pose_data.get("duration_sec", self._motion_duration_sec))
        duration_sec = int(duration_float)
        duration_nsec = int((duration_float - duration_sec) * 1e9)

        point = JointTrajectoryPoint()
        point.positions = [math.radians(float(p)) for p in pose_data["positions"]]
        point.time_from_start = Duration(sec=duration_sec, nanosec=duration_nsec)

        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def execute_spray_sequence(self) -> ArmExitCode:
        """
        Execute the spray sweep: sequ_home → P1 → P2 → P1 → P2 → P1 → sequ_home.
        Returns on the first failure; otherwise returns SUCCEEDED.
        """
        sequence = ["sequ_home", "sequ_p1", "sequ_p2", "sequ_p1", "sequ_p2", "sequ_p1", "sequ_home"]
        for pose_name in sequence:
            self.get_logger().info(f"Spray sequence: moving to '{pose_name}'")
            exit_code = self.move_arm_to_pose(pose_name)
            if exit_code != ArmExitCode.SUCCEEDED:
                self.get_logger().error(
                    f"Spray sequence aborted at '{pose_name}': {exit_code.name}"
                )
                return exit_code
        self.get_logger().info("Spray sequence complete")
        return ArmExitCode.SUCCEEDED

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.actual.positions:
            positions_str = ", ".join(f"{p:.3f}" for p in feedback.actual.positions)
            self.get_logger().debug(f"Arm positions: [{positions_str}]")

    def _interpret_result(self, pose_name: str, result) -> ArmExitCode:
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"Arm reached pose '{pose_name}'")
            return ArmExitCode.SUCCEEDED
        elif result.error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            self.get_logger().error(
                f"Invalid goal for '{pose_name}': {result.error_string}"
            )
            return ArmExitCode.INVALID_GOAL
        elif result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            self.get_logger().error(
                f"Invalid joints for '{pose_name}': {result.error_string}"
            )
            return ArmExitCode.INVALID_JOINTS
        else:
            self.get_logger().error(
                f"Arm motion failed for '{pose_name}' (code {result.error_code}): {result.error_string}"
            )
            return ArmExitCode.FAILED
