
"""Pick-and-place primitives for the depalletizing robot."""

from depalletizing_interfaces.models import RobotPose, OperationResult


class PickPlace:
    """Implement pick and place operations using robot and gripper interfaces."""

    def __init__(self, robot, gripper):
        """Initialize with robot and gripper instances."""
        self.robot = robot
        self.gripper = gripper

    def pick_object(self, pose: RobotPose) -> OperationResult:
        """Pick an object from the given pose."""
        approach_pose = RobotPose(
            x=pose.x,
            y=pose.y,
            z=pose.z + 50.0,
            r=pose.r,
            frame_id=pose.frame_id,
        )

        result = self.robot.move_to(approach_pose)
        if not result.success:
            return result

        result = self.robot.move_to(pose)
        if not result.success:
            return result

        result = self.gripper.activate_tool()
        if not result.success:
            return result

        result = self.robot.move_to(approach_pose)
        if not result.success:
            return result

        return OperationResult(
            success=True,
            message="Object picked successfully",
        )

    def place_object(self, pose: RobotPose) -> OperationResult:
        """Place an object at the given pose."""
        approach_pose = RobotPose(
            x=pose.x,
            y=pose.y,
            z=pose.z + 50.0,
            r=pose.r,
            frame_id=pose.frame_id,
        )

        result = self.robot.move_to(approach_pose)
        if not result.success:
            return result

        result = self.robot.move_to(pose)
        if not result.success:
            return result

        result = self.gripper.deactivate_tool()
        if not result.success:
            return result

        result = self.robot.move_to(approach_pose)
        if not result.success:
            return result

        return OperationResult(
            success=True,
            message="Object placed successfully",
        )
