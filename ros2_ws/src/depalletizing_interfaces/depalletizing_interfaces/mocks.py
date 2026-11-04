
"""Mock robot and gripper implementations for hardware-free testing."""

from .models import RobotPose, OperationResult


class MockRobot:
    """Simulate robot movement without physical hardware."""

    def __init__(self):
        """Initialize robot state and movement history."""
        self.current_pose = None
        self.movement_history = []
        self.should_fail = False

    def move_to(self, pose: RobotPose) -> OperationResult:
        """Simulate moving the robot to a target pose."""
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
    """Simulate gripper activation and deactivation."""

    def __init__(self):
        """Initialize the gripper state and failure flag."""
        self.is_active = False
        self.should_fail = False

    def activate_tool(self) -> OperationResult:
        """Simulate activating the gripper."""
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
        """Simulate deactivating the gripper."""
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
