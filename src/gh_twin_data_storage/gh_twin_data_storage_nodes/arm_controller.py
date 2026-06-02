class ArmController:
    """Interface used by PlanExecutorNode for arm pose actions."""

    def move_arm_to_pose(self, pose_name: str):
        raise NotImplementedError(
            f"Arm pose '{pose_name}' was requested, but ArmController is not implemented yet."
        )
