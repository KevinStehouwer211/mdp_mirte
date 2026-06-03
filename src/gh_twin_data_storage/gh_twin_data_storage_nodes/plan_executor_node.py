#!/usr/bin/env python3

import argparse
import math
import re
from pathlib import Path
from typing import Dict, List
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Bool
from typedb.driver import SessionType, TransactionType

from gh_twin.nav_to_pose import ExitCode, NavToPose
from gh_twin_data_storage_nodes.pddl_planner_node import PddlPlannerNode
from gh_twin_data_storage_nodes.arm_controller import ArmController, ArmExitCode

class PlanAborted(Exception):
    pass


def quaternion_from_yaw(yaw: float):
    half_yaw = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


class PlanExecutorNode(Node):
    def __init__(
        self,
        domain_file: str = "DomainGreenhouse.pddl",
        problem_file: str = "problem_generated.pddl",
    ):
        super().__init__("plan_executor")

        self.declare_parameter("domain_file", domain_file)
        self.declare_parameter("problem_file", problem_file)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("execute_on_start", True)
        self.declare_parameter("battery_topic", "/power/power_watcher")
        self.declare_parameter("battery_threshold", 0.25)
        self.declare_parameter("battery_check_period", 10.0)
        self.declare_parameter("recharge_waypoint", "wp0") #Placeholder waypoint for now
        self.declare_parameter("typedb_address", "localhost:1729")
        self.declare_parameter("database_name", "greenhouse")
        self.declare_parameter("operating_mode_topic", "/operating_mode")
        self.declare_parameter("robot_heartbeat_topic", "/robot_heartbeat")
        self.declare_parameter("arm_poses_file", "")
        
        self.domain_file = Path(self.get_parameter("domain_file").value)
        self.problem_file = Path(self.get_parameter("problem_file").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.execute_on_start = bool(self.get_parameter("execute_on_start").value)
        
        self.battery_topic = self.get_parameter("battery_topic").value
        self.battery_threshold = float(self.get_parameter("battery_threshold").value)
        self.recharge_waypoint = self.get_parameter("recharge_waypoint").value
        self.typedb_address = self.get_parameter("typedb_address").value
        self.database_name = self.get_parameter("database_name").value
        self.operating_mode_topic = self.get_parameter("operating_mode_topic").value
        self.robot_heartbeat_topic = self.get_parameter("robot_heartbeat_topic").value

        self.navigator = None
        self.pddl_manager = PddlPlannerNode(
            domain_file=str(self.domain_file),
            problem_file=str(self.problem_file),
            typedb_address=self.typedb_address,
            database_name=self.database_name,
            node_name="pddl_planner_helper",
            use_global_arguments=False,
        )

        self._started = False
        self.plan_active = False
        self.abort_requested = False
        self.returning_to_recharge = False
        self.current_goal_handle = None
        self.latest_battery = None
        self.robot_heartbeat = True
        self.operating_mode = 1 #default to manual mode
        self.arm_controller = None

        self.create_subscription(BatteryState, self.battery_topic, self.battery_callback, 10)
        self.create_subscription(Int32, self.operating_mode_topic, self.operating_mode_callback, 10)

        self.robot_heartbeat_publisher = self.create_publisher(Bool, self.robot_heartbeat_topic, 10)
        self.create_timer(1.0, self.publish_robot_heartbeat)

        self.robot_heartbeat = True

        self.create_timer(
            float(self.get_parameter("battery_check_period").value),
            self.check_battery_level,
        )
        
    def destroy_node(self):
        self.robot_heartbeat = False
        self.publish_robot_heartbeat()

        try:
            if self.navigator is not None:
                self.navigator.destroy_node()
            self.pddl_manager.close()
            self.pddl_manager.destroy_node()
        finally:
            return super().destroy_node()

    def operating_mode_callback(self, msg: Int32):
        self.operating_mode = msg.data
        self.set_operating_mode()
        self.get_logger().info(f"Operating mode changed to: {self.operating_mode}")

    def set_operating_mode(self):
        # this function subscribes/reads the mode selection topic from the HMI (\operatingmode) and updates the mode parameter

        if self.operating_mode == 0: #data collection
            self.abort_requested = False
            if not self.plan_active:
                self._execute_plan_once()
                self.get_logger().info("Data collection mode; executing current plan")
            else:
                return
 
        elif self.operating_mode == 1: #manual
            self.abort_requested = True
            self.get_logger().info ("Manual mode; aborting current plan and task scheduler is on standby untill mode change")

        elif self.operating_mode == 2  :  #rebase
            self.abort_requested = False
            self.return_to_recharge()
            self.get_logger().info("Rebase mode; returning to recharge waypoint")

        else:
            self.get_logger().info("Mode not implemented/known")
        
    def _execute_plan_once(self):
        self._started = True
        try:
            self.execute_current_plan()
        except Exception as exc:
            self.get_logger().error(f"Plan execution failed: {exc}")

    def execute_current_plan(self):
        self.pddl_manager.connect_to_typedb()
        self.pddl_manager.generate_pddl_problem(str(self.problem_file))
        planner_output = self.pddl_manager.run_planner(
            domain_file=str(self.domain_file),
            problem_file=str(self.problem_file),
        )

        if not planner_output:
            raise RuntimeError("Planner did not return a plan.")

        plan = self.parse_popf_plan(planner_output)
        if not plan:
            raise RuntimeError("Planner output did not contain executable steps.")

        self.plan_active = True
        try:
            self.get_logger().info(f"Executing {len(plan)} plan steps")
            for index, action in enumerate(plan, start=1):
                self.raise_if_aborted()
                self.get_logger().info(
                    f"Step {index}/{len(plan)}: {action['name']} {action['raw_args']}"
                )
                if action["name"] == "move":
                    self.execute_move(action)
                elif action["name"] in {"scan", "spray"}:
                    self.execute_arm_action(action)
                else:
                    self.get_logger().warn(f"Skipping unsupported PDDL action: {action['raw']}")
        except PlanAborted:
            self.get_logger().warn(f"Plan aborted by battery monitor. Returning to {self.recharge_waypoint}")
            self.return_to_recharge()
            return
        finally:
            self.plan_active = False

        self.get_logger().info("Plan execution complete")

    def battery_callback(self, msg: BatteryState):
        self.latest_battery = msg

    def check_battery_level(self):
        if self.latest_battery is None:
            self.get_logger().warn(f"No battery messages received yet on {self.battery_topic}")
            return

        percentage = float(self.latest_battery.percentage)
        self.get_logger().info(f"Battery level: {percentage * 100:.0f}%")
        
        if percentage >= self.battery_threshold:
            return

        if not self.plan_active or self.returning_to_recharge:
            return

        self.get_logger().warn(f"Battery below threshold ({percentage:.2f} < {self.battery_threshold:.2f})")
        self.abort_requested = True

    def publish_robot_heartbeat(self):
        msg = Bool()
        msg.data = self.robot_heartbeat
        self.robot_heartbeat_publisher.publish(msg)

    def parse_popf_plan(self, planner_output: str) -> List[Dict]:
        """Turn POPF text into simple action dictionaries."""
        pattern = re.compile(
            r"^\s*(?P<start>[0-9]+(?:\.[0-9]+)?)\s*:\s*"
            r"\((?P<body>[^)]+)\)"
            r"(?:\s*\[(?P<duration>[0-9]+(?:\.[0-9]+)?)\])?"
        )
        steps = []

        for line in planner_output.splitlines():
            match = pattern.match(line)
            if not match:
                continue

            tokens = match.group("body").lower().split()
            if not tokens:
                continue

            action = {
                "start_time": float(match.group("start")),
                "name": tokens[0],
                "args": tokens[1:],
                "raw_args": " ".join(tokens[1:]),
                "duration": float(match.group("duration") or 0.0),
                "raw": line.strip(),
            }

            if action["name"] == "move" and len(action["args"]) >= 4:
                action["from_waypoint"] = action["args"][2]
                action["to_waypoint"] = action["args"][3]

            steps.append(action)

        return sorted(steps, key=lambda action: action["start_time"])

    def execute_move(self, action: Dict):
        self.raise_if_aborted()
        if "to_waypoint" not in action:
            raise RuntimeError(f"Move step has unexpected arguments: {action['raw']}")

        waypoint_name = action["to_waypoint"]
        pose = self.query_waypoint_pose(waypoint_name)
        self.navigate_to_waypoint(waypoint_name, pose)
        self.replace_current_location(waypoint_name)

    def execute_wait(self, action: Dict):
        self.raise_if_aborted()
        duration = action["duration"]

        if duration <= 0.0:
            self.get_logger().warn(f"duration is 0 or missing for action {action['raw']}")

        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.raise_if_aborted()
            time.sleep(0.1)

    def get_navigator(self) -> NavToPose:
        if self.navigator is None:
            initial_pose = Pose()
            initial_pose.orientation.w = 1.0
            self.get_logger().info("Initializing navigation subsystem")
            self.navigator = NavToPose(initial_pose)
        return self.navigator

    def query_waypoint_pose(self, waypoint_pddl_name: str) -> Dict[str, float]:
        query = """
        match
          $wp isa waypoint, has id $wp_id, has x $x, has y $y, has yaw $yaw;
        get $wp_id, $x, $y, $yaw;
        """
        rows = self.pddl_manager._read_query(query)

        for row in rows:
            waypoint_id = self.pddl_manager._read_attr(row, "wp_id")
            if self.pddl_manager._to_readable_pddl_name(waypoint_id) != waypoint_pddl_name:
                continue

            return {
                "id": waypoint_id,
                "x": float(self.pddl_manager._read_attr(row, "x")),
                "y": float(self.pddl_manager._read_attr(row, "y")),
                "yaw": float(self.pddl_manager._read_attr(row, "yaw")),
            }

        raise RuntimeError(f"No TypeDB pose found for waypoint '{waypoint_pddl_name}'")

    def navigate_to_waypoint(self, waypoint_pddl_name: str, pose: Dict[str, float]):
        navigator = self.get_navigator()
        goal_pose = self.build_nav_pose(pose)

        self.get_logger().info(
            f"Navigating to {pose['id']} ({pose['x']:.3f}, {pose['y']:.3f}, yaw={pose['yaw']:.3f})"
        )

        exit_code = navigator.nav_set_goal(goal_pose)
        if exit_code != ExitCode.PATH_VALID_GOAL_SET:
            raise RuntimeError(
                f"Navigation subsystem rejected {waypoint_pddl_name}: {exit_code.name}"
            )

        terminal_codes = {
            ExitCode.GOAL_SUCCEEDED,
            ExitCode.GOAL_FAILED,
            ExitCode.TIMEOUT,
            ExitCode.PATH_INVALID,
            ExitCode.INIT_FAILED,
            ExitCode.GOAL_CANCELED,
        }

        while rclpy.ok():
            exit_code = navigator.nav_update_status()
            rclpy.spin_once(navigator, timeout_sec=0.1)
            if self.abort_requested and not self.returning_to_recharge:
                self.cancel_current_nav_goal()
            self.raise_if_aborted()
            if exit_code in terminal_codes:
                break

        if exit_code != ExitCode.GOAL_SUCCEEDED:
            raise RuntimeError(
                f"Navigation to {waypoint_pddl_name} failed: {exit_code.name}"
            )

        self.get_logger().info(f"Reached {pose['id']}")

    def cancel_current_nav_goal(self):
        if self.navigator is None:
            return

        self.navigator.nav_cancel_goal()

    def raise_if_aborted(self):
        if self.abort_requested and not self.returning_to_recharge:
            raise PlanAborted()

    def return_to_recharge(self):
        self.returning_to_recharge = True
        self.abort_requested = False
        try:
            recharge_pose = self.query_waypoint_pose(self.recharge_waypoint)
            self.navigate_to_waypoint(self.recharge_waypoint, recharge_pose)
            self.replace_current_location(self.recharge_waypoint)
            self.get_logger().info(f"Robot returned to recharge waypoint {self.recharge_waypoint}")
        finally:
            self.returning_to_recharge = False

    def build_nav_pose(self, pose: Dict[str, float]) -> Pose:
        """Convert a TypeDB waypoint pose into the Pose message the navigation subsystem needs."""
        pose_msg = Pose()
        pose_msg.position.x = pose["x"]
        pose_msg.position.y = pose["y"]
        pose_msg.position.z = 0.0

        quat = quaternion_from_yaw(pose["yaw"])
        pose_msg.orientation.x = quat["x"]
        pose_msg.orientation.y = quat["y"]
        pose_msg.orientation.z = quat["z"]
        pose_msg.orientation.w = quat["w"]
        return pose_msg

    def replace_current_location(self, waypoint_pddl_name: str):
        waypoint_id = self.query_waypoint_pose(waypoint_pddl_name)["id"]
        robot_id = self.pddl_manager.robot_id

        delete_query = f'''
        match
          $robot isa robot, has id "{robot_id}";
          $old_loc (located: $robot, location: $old_wp) isa current-location;
        delete
          $old_loc isa current-location;
        '''

        insert_query = f'''
        match
          $robot isa robot, has id "{robot_id}";
          $wp isa waypoint, has id "{waypoint_id}";
        insert
          $loc (located: $robot, location: $wp) isa current-location;
        '''

        with self.pddl_manager.driver.session(self.pddl_manager.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as transaction:
                transaction.query.delete(delete_query)
                transaction.commit()

        self.pddl_manager._write_query(insert_query)
        self.get_logger().info(f"Updated TypeDB current-location to {waypoint_id}")

    def execute_arm_action(self, action: Dict):
        arm_controller = self.get_arm_controller()

        if action["name"] == "scan":
            self.get_logger().info("Moving arm to scan position.")
            exit_code = arm_controller.move_arm_to_pose("scan")
            if exit_code != ArmExitCode.SUCCEEDED:
                raise RuntimeError(f"Arm failed to reach scan pose: {exit_code.name}")
            self.replace_current_arm_pose("scan")

            self.execute_wait(action)

            exit_code = arm_controller.move_arm_to_pose("base")
            if exit_code != ArmExitCode.SUCCEEDED:
                raise RuntimeError(f"Arm failed to return to base: {exit_code.name}")
            self.replace_current_arm_pose("base")

        elif action["name"] == "spray":
            self.get_logger().info("Executing spray sequence.")
            exit_code = arm_controller.execute_spray_sequence()
            if exit_code != ArmExitCode.SUCCEEDED:
                raise RuntimeError(f"Spray sequence failed: {exit_code.name}")
            self.replace_current_arm_pose("base")

        else:
            self.get_logger().warn(f"Skipping unknown arm action: {action['name']}")

    def get_arm_controller(self) -> ArmController:
        if self.arm_controller is None:
            config_file = self.get_parameter("arm_poses_file").value or None
            self.get_logger().info("Initializing arm controller")
            self.arm_controller = ArmController(config_file=config_file)
        return self.arm_controller

    def replace_current_arm_pose(self, arm_pose):
        arm_id = self.pddl_manager.arm_id
        
        delete_query = f'''
        match
          $arm isa arm, has id "{arm_id}", has arm-state $old-arm-state;
        delete
          $arm has $old-arm-state;
        '''

        insert_query = f'''
        match
          $arm isa arm, has id "{arm_id}", has arm-state "{arm_pose}";
        insert
          $arm has arm-state "{arm_pose}";
        '''

        with self.pddl_manager.driver.session(self.pddl_manager.database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as transaction:
                transaction.query.delete(delete_query)
                transaction.commit()

        self.pddl_manager._write_query(insert_query)
        self.get_logger().info(f"Updated TypeDB arm-state to {arm_pose}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Execute greenhouse PDDL plans in simulation.")
    parser.add_argument("--domain-file", default="DomainGreenhouse.pddl")
    parser.add_argument("--problem-file", default="problem_generated.pddl")
    parsed, unknown = parser.parse_known_args(args=args)

    rclpy.init(args=unknown)
    node = PlanExecutorNode(
        domain_file=parsed.domain_file,
        problem_file=parsed.problem_file,
    )

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
