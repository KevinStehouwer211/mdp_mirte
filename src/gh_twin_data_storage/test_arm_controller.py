#!/usr/bin/env python3
"""
Unit tests for ArmController.

All ROS 2 dependencies are mocked — no live ROS 2 environment is needed.

Run with:
    python3 test_arm_controller.py
"""

import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Stub every ROS 2 / control_msgs dependency before importing the module.
# ---------------------------------------------------------------------------

def _make_logger():
    logger = MagicMock()
    logger.info  = lambda msg: print(f"  [INFO]  {msg}")
    logger.error = lambda msg: print(f"  [ERROR] {msg}")
    logger.warn  = lambda msg: print(f"  [WARN]  {msg}")
    logger.debug = lambda msg: None
    return logger


class _MockNode:
    def __init__(self, name):
        self._log = _make_logger()
    def get_logger(self):
        return self._log


rclpy_mod = MagicMock()
rclpy_mod.ok.return_value = True
rclpy_mod.spin_once = MagicMock()

sys.modules.setdefault("rclpy",                  rclpy_mod)
sys.modules.setdefault("rclpy.node",             MagicMock(Node=_MockNode))
sys.modules.setdefault("rclpy.action",           MagicMock())
sys.modules.setdefault("control_msgs",           MagicMock())
sys.modules.setdefault("control_msgs.action",    MagicMock())
sys.modules.setdefault("trajectory_msgs",        MagicMock())
sys.modules.setdefault("trajectory_msgs.msg",    MagicMock())
sys.modules.setdefault("builtin_interfaces",     MagicMock())
sys.modules.setdefault("builtin_interfaces.msg", MagicMock())

# Put the package on the path so the import below works regardless of how
# the script is invoked.
_pkg_root = Path(__file__).resolve().parent / "gh_twin_data_storage_nodes"
sys.path.insert(0, str(_pkg_root))

from arm_controller import ArmController, ArmExitCode  # noqa: E402

# ---------------------------------------------------------------------------
# Set FollowJointTrajectory result constants on the mock so comparisons work.
# ---------------------------------------------------------------------------
from control_msgs.action import FollowJointTrajectory as _FJT  # noqa: E402

_FJT.Result.SUCCESSFUL     = 0
_FJT.Result.INVALID_GOAL   = -1
_FJT.Result.INVALID_JOINTS = -2


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

VALID_CONFIG = """\
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_joint

motion_duration_sec: 3.0

poses:
  home:
    positions: [0, 0, 0, 0]
  base:
    positions: [0, 0, 0, 0]
  scan:
    positions: [0, -60, 30, 0]
    duration_sec: 3
  spray:
    positions: [0, -45, 70, 0]
    duration_sec: 3
"""


def _write_config(text: str = VALID_CONFIG) -> str:
    f = tempfile.NamedTemporaryFile(mode="w", suffix=".yml", delete=False)
    f.write(text)
    f.close()
    return f.name


def _make_arm(server_available: bool = True, config: str = VALID_CONFIG) -> ArmController:
    """Create an ArmController with a mocked ActionClient."""
    from unittest.mock import patch
    config_path = _write_config(config)
    with patch("arm_controller.ActionClient") as MockAC:
        MockAC.return_value.wait_for_server.return_value = server_available
        arm = ArmController(config_file=config_path)
        arm._action_client = MockAC.return_value
    return arm


def _immediate_future(result=None):
    """Future that reports done() == True immediately."""
    f = MagicMock()
    f.done.return_value = True
    f.result.return_value = result
    return f


def _never_done_future():
    """Future that never becomes done (used to exercise timeouts)."""
    f = MagicMock()
    f.done.return_value = False
    return f


def _goal_handle(accepted: bool = True, error_code: int = 0, error_string: str = ""):
    handle = MagicMock()
    handle.accepted = accepted
    result_wrapper = MagicMock()
    result_wrapper.result.error_code = error_code
    result_wrapper.result.error_string = error_string
    handle.get_result_async.return_value = _immediate_future(result_wrapper)
    return handle


def _wire_action(arm: ArmController, accepted: bool = True,
                 error_code: int = 0, error_string: str = ""):
    """
    Configure arm._action_client so send_goal_async resolves in a single
    spin cycle: goal accepted/rejected, then result returned.
    """
    handle = _goal_handle(accepted=accepted, error_code=error_code, error_string=error_string)
    arm._action_client.send_goal_async.return_value = _immediate_future(handle)


# ---------------------------------------------------------------------------
# Test classes
# ---------------------------------------------------------------------------

class TestInit(unittest.TestCase):

    def test_init_no_config_succeeds(self):
        """Controller initialises with no poses when no config is given."""
        from unittest.mock import patch
        with patch("arm_controller.ActionClient") as MockAC:
            MockAC.return_value.wait_for_server.return_value = True
            arm = ArmController()
        self.assertTrue(arm._init_complete)
        self.assertEqual(arm._poses, {})

    def test_init_loads_all_poses(self):
        """All four poses in the config are loaded correctly."""
        arm = _make_arm()
        self.assertTrue(arm._init_complete)
        self.assertEqual(sorted(arm._poses.keys()), ["base", "home", "scan", "spray"])

    def test_init_loads_joint_names(self):
        """Joint names from the config are stored on the controller."""
        arm = _make_arm()
        self.assertEqual(arm._joint_names, [
            "shoulder_pan_joint", "shoulder_lift_joint",
            "elbow_joint", "wrist_joint",
        ])

    def test_init_loads_global_duration(self):
        arm = _make_arm()
        self.assertEqual(arm._motion_duration_sec, 3.0)

    def test_init_missing_config_file_does_not_crash(self):
        """A missing config path logs an error but init still completes."""
        from unittest.mock import patch
        with patch("arm_controller.ActionClient") as MockAC:
            MockAC.return_value.wait_for_server.return_value = True
            arm = ArmController(config_file="/nonexistent/arm_poses.yml")
        self.assertTrue(arm._init_complete)
        self.assertEqual(arm._poses, {})

    def test_init_server_unavailable_sets_flag_false(self):
        """_init_complete is False when the action server is unreachable."""
        arm = _make_arm(server_available=False)
        self.assertFalse(arm._init_complete)


class TestMoveArmToPose(unittest.TestCase):

    def test_not_initialized_returns_init_failed(self):
        arm = _make_arm(server_available=False)
        self.assertEqual(arm.move_arm_to_pose("scan"), ArmExitCode.INIT_FAILED)

    def test_unknown_pose_returns_unknown_pose(self):
        arm = _make_arm()
        self.assertEqual(arm.move_arm_to_pose("levitate"), ArmExitCode.UNKNOWN_POSE)

    def test_successful_move_returns_succeeded(self):
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=0)
        self.assertEqual(arm.move_arm_to_pose("scan"), ArmExitCode.SUCCEEDED)

    def test_goal_rejected_returns_invalid_goal(self):
        arm = _make_arm()
        _wire_action(arm, accepted=False)
        self.assertEqual(arm.move_arm_to_pose("spray"), ArmExitCode.INVALID_GOAL)

    def test_result_invalid_goal_code(self):
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=-1, error_string="bad goal")
        self.assertEqual(arm.move_arm_to_pose("scan"), ArmExitCode.INVALID_GOAL)

    def test_result_invalid_joints_code(self):
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=-2, error_string="bad joints")
        self.assertEqual(arm.move_arm_to_pose("home"), ArmExitCode.INVALID_JOINTS)

    def test_result_generic_failure(self):
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=-99, error_string="controller fault")
        self.assertEqual(arm.move_arm_to_pose("base"), ArmExitCode.FAILED)

    def test_all_configured_poses_succeed(self):
        """Every pose from the config file can be sent without error."""
        arm = _make_arm()
        for pose_name in arm._poses:
            with self.subTest(pose=pose_name):
                _wire_action(arm, accepted=True, error_code=0)
                self.assertEqual(arm.move_arm_to_pose(pose_name), ArmExitCode.SUCCEEDED)

    def test_send_goal_async_called_with_correct_joint_names(self):
        """The goal trajectory uses joint names from the config."""
        import math
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=0)
        arm.move_arm_to_pose("scan")

        # Retrieve the goal that was sent
        call_args = arm._action_client.send_goal_async.call_args
        goal_msg = call_args[0][0]
        # trajectory is set on the goal; joint_names must match config
        self.assertEqual(
            list(goal_msg.trajectory.joint_names),
            ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"],
        )

    def test_positions_converted_to_radians(self):
        """Degree values in the config are converted to radians before sending."""
        import math
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=0)
        arm.move_arm_to_pose("scan")

        call_args = arm._action_client.send_goal_async.call_args
        goal_msg = call_args[0][0]
        sent_positions = list(goal_msg.trajectory.points[0].positions)
        expected = [math.radians(float(p)) for p in arm._poses["scan"]["positions"]]
        self.assertEqual(sent_positions, expected)

    def test_per_pose_duration_used(self):
        """A pose with its own duration_sec uses that value, not the global default."""
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=0)
        arm.move_arm_to_pose("scan")  # scan has duration_sec: 3

        call_args = arm._action_client.send_goal_async.call_args
        goal_msg = call_args[0][0]
        self.assertEqual(goal_msg.trajectory.points[0].time_from_start.sec, 3)

    def test_global_duration_used_when_pose_has_none(self):
        """A pose without duration_sec falls back to motion_duration_sec."""
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=0)
        arm.move_arm_to_pose("home")  # home has no duration_sec

        call_args = arm._action_client.send_goal_async.call_args
        goal_msg = call_args[0][0]
        self.assertEqual(
            goal_msg.trajectory.points[0].time_from_start.sec,
            int(arm._motion_duration_sec),
        )


class TestTimeout(unittest.TestCase):

    def setUp(self):
        rclpy_mod.ok.return_value = True

    def test_goal_acceptance_timeout(self):
        """TIMEOUT is returned when the goal-acceptance future never resolves."""
        arm = _make_arm()
        arm.ARM_TIMEOUT = 0.05

        arm._action_client.send_goal_async.return_value = _never_done_future()
        self.assertEqual(arm.move_arm_to_pose("scan"), ArmExitCode.TIMEOUT)

    def test_result_timeout(self):
        """TIMEOUT is returned when the result future never resolves."""
        arm = _make_arm()
        arm.ARM_TIMEOUT = 0.05

        handle = _goal_handle(accepted=True, error_code=0)
        handle.get_result_async.return_value = _never_done_future()
        arm._action_client.send_goal_async.return_value = _immediate_future(handle)

        self.assertEqual(arm.move_arm_to_pose("scan"), ArmExitCode.TIMEOUT)


class TestFeedbackCallback(unittest.TestCase):

    def test_feedback_with_positions_does_not_raise(self):
        arm = _make_arm()
        msg = MagicMock()
        msg.feedback.actual.positions = [0.1, -1.0, 0.5, 0.0]
        arm._feedback_callback(msg)   # must not raise

    def test_feedback_empty_positions_does_not_raise(self):
        arm = _make_arm()
        msg = MagicMock()
        msg.feedback.actual.positions = []
        arm._feedback_callback(msg)   # must not raise


class TestConfigEdgeCases(unittest.TestCase):

    def test_empty_poses_block(self):
        """A config with no poses section initialises cleanly."""
        config = "joint_names: [j1, j2, j3, j4]\nmotion_duration_sec: 2.0\n"
        arm = _make_arm(config=config)
        self.assertEqual(arm._poses, {})
        self.assertTrue(arm._init_complete)

    def test_partial_pose_entry(self):
        """A pose with only positions (no per-pose duration) still works."""
        arm = _make_arm()
        _wire_action(arm, accepted=True, error_code=0)
        # 'base' has no duration_sec in the default config
        self.assertEqual(arm.move_arm_to_pose("base"), ArmExitCode.SUCCEEDED)


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    unittest.main(verbosity=2)
