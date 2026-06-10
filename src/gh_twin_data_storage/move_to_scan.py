#!/usr/bin/env python3
"""
Standalone arm-move + snapshot test.

Run against real hardware:
    python3 test_arm_controller_local_backup.py

Sequence: scan_a → snapshot A → scan_b → snapshot B → base
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock

_pkg_root = Path(__file__).resolve().parent
sys.path.insert(0, str(_pkg_root))


if __name__ == "__main__":
    import rclpy
    from gh_twin.arm_controller import ArmController, ArmExitCode
    # from gh_twin.vision_controller import VisionController

    rclpy.init()

    config_file_path = str(_pkg_root / "config/arm_poses.yml")
    print(f"Using config file: {config_file_path}")

    arm_ctrl = ArmController(config_file=config_file_path)
    # vision_ctrl = VisionController(config_file=config_file_path)

    try:


      # ── scan_a ─────────────────────────────────────────────────────
        print("[scan] moving arm → scan_a")
        code = arm_ctrl.move_arm_to_pose("sequ_home")
        if code != ArmExitCode.SUCCEEDED:
            print(f"[scan] ERROR: scan_a failed ({code.name})")

        arm_ctrl.execute_spray_sequence()

        code = arm_ctrl.move_arm_to_pose("scan_a")

        # # ── start triangulation ────────────────────────────────────────
        # print("[scan] starting triangulation accumulation")
        # vision_ctrl.start_triangulation()

  
        # # ── scan_b ─────────────────────────────────────────────────────
        # print("[scan] moving arm → scan_b")
        # code = arm_ctrl.move_arm_to_pose("scan_b")
        # if code != ArmExitCode.SUCCEEDED:
        #     print(f"[scan] ERROR: scan_b failed ({code.name})")

        # # ── finish triangulation ───────────────────────────────────────
        # print("[scan] finishing triangulation → publishing 3-D result")
        # vision_ctrl.finish_triangulation()

        # # ── return to base ─────────────────────────────────────────────
        # print("[scan] returning arm → base")
        # code = arm_ctrl.move_arm_to_pose("base")
        # if code != ArmExitCode.SUCCEEDED:
        #     print(f"[scan] ERROR: failed to reach base ({code.name})")

        # print("[scan] done")

    finally:
        arm_ctrl.destroy_node()
        # vision_ctrl.destroy_node()
        rclpy.shutdown()

else:
    # Imported as a module — stub ROS 2 dependencies so tests run without
    # a live ROS 2 environment.

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

    sys.modules["rclpy"]                  = rclpy_mod
    sys.modules["rclpy.node"]             = MagicMock(Node=_MockNode)
    sys.modules["rclpy.action"]           = MagicMock()
    sys.modules["control_msgs"]           = MagicMock()
    sys.modules["control_msgs.action"]    = MagicMock()
    sys.modules["trajectory_msgs"]        = MagicMock()
    sys.modules["trajectory_msgs.msg"]    = MagicMock()
    sys.modules["builtin_interfaces"]     = MagicMock()
    sys.modules["builtin_interfaces.msg"] = MagicMock()

    from gh_twin.arm_controller import ArmController  # noqa: E402