
from .models import RobotPose, OperationResult


class MockRobot:
    """Simulates robot movement without hardware."""

    def __init__(self):
        self.current_pose = None
        self.movement_history = []
        self.should_fail = False

    def move_to(self, pose: RobotPose) -> OperationResult:
        if self.should_fail:
            return OperationResult(
                success=False,
                message="Simulated robot movement failure",
                error_code="MOVE_FAILED"
            )

        self.current_pose = pose
        self.movement_history.append(pose)

        return OperationResult(
            success=True,
            message="Mock robot reached target"
        )


class MockGripper:
    """Simulates gripper activation and deactivation."""

    def __init__(self):
        self.is_active = False
        self.should_fail = False

    def activate_tool(self) -> OperationResult:
        if self.should_fail:
            return OperationResult(
                success=False,
                message="Simulated gripper activation failure",
                error_code="GRIPPER_FAILED"
            )

        self.is_active = True

        return OperationResult(
            success=True,
            message="Mock gripper activated"
        )

    def deactivate_tool(self) -> OperationResult:
        if self.should_fail:
            return OperationResult(
                success=False,
                message="Simulated gripper deactivation failure",
                error_code="GRIPPER_FAILED"
            )

        self.is_active = False

        return OperationResult(
            success=True,
            message="Mock gripper deactivated"
        )