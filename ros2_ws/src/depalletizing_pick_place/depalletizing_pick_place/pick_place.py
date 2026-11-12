
"""Pick-and-place primitives for the depalletizing robot."""

from rclpy.task import Future

from depalletizing_interfaces.models import RobotPose, OperationResult


class PickPlace:
    """Implement pick and place operations using robot and gripper interfaces."""

    def __init__(self, robot, gripper, approach_height=50.0):
        """Initialize with robot, gripper, and approach height in millimeters."""
        self.robot = robot
        self.gripper = gripper
        self.approach_height = approach_height

    @staticmethod
    def _failed_result(message: str, error_code: str) -> OperationResult:
        """Build a failed result using a shared error code."""
        return OperationResult(
            success=False,
            message=message,
            error_code=error_code,
        )

    def _completed_future(self, result: OperationResult) -> Future:
        """Wrap a finished OperationResult in a Future."""
        future = Future()
        future.set_result(result)
        return future

    def _as_future(self, operation):
        """Normalize synchronous and asynchronous operations."""

        try:
            result = operation()

            if isinstance(result, Future):
                return result

            if isinstance(result, OperationResult):
                return self._completed_future(result)

            raise TypeError(
                "Operation must return Future or OperationResult."
            )

        except Exception as exc:
            return self._completed_future(
                self._failed_result(
                    f"Operation error: {exc}",
                    "WORKFLOW_ERROR",
                )
            )

    def _run_sequence(self, steps, success_message: str) -> Future:
        """
        Execute operations sequentially without blocking.

        Each step may return a Future or an OperationResult.
        The next step starts only after the previous one succeeds.
        Failure, cancellation, and timeout all stop the sequence.
        """
        workflow_future = Future()

        def execute_step(index):
            if workflow_future.done():
                return

            # All steps completed successfully.
            if index >= len(steps):
                workflow_future.set_result(
                    OperationResult(
                        success=True,
                        message=success_message
                    )
                )
                return

            try:
                operation_future = self._as_future(steps[index])

                operation_future.add_done_callback(
                    lambda future: handle_step_result(future, index)
                )

            except Exception as exc:
                workflow_future.set_result(
                    self._failed_result(
                        f"Workflow execution error: {exc}",
                        "WORKFLOW_ERROR",
                    )
                )

        def handle_step_result(future, index):
            if workflow_future.done():
                return

            try:
                result = future.result()

                if not isinstance(result, OperationResult):
                    raise TypeError(
                        "Operation did not return an OperationResult."
                    )

                if not result.success:
                    workflow_future.set_result(result)
                    return

                # Continue with the next operation.
                execute_step(index + 1)

            except Exception as exc:
                workflow_future.set_result(
                    self._failed_result(
                        f"Operation failed: {exc}",
                        "WORKFLOW_ERROR",
                    )
                )

        execute_step(0)

        return workflow_future

    def _approach_pose(self, pose: RobotPose) -> RobotPose:
        """Return the pose used before and after the object pose."""
        return RobotPose(
            x=pose.x,
            y=pose.y,
            z=pose.z + self.approach_height,
            r=pose.r,
            frame_id=pose.frame_id,
        )

    def pick_object(self, pose: RobotPose) -> Future:
        """Pick an object from the given pose."""
        approach_pose = self._approach_pose(pose)

        steps = [
            lambda: self.robot.move_to(approach_pose),
            lambda: self.robot.move_to(pose),
            self.gripper.activate_tool,
            lambda: self.robot.move_to(approach_pose),
        ]

        return self._run_sequence(
            steps,
            "Object picked successfully"
        )

    def place_object(self, pose: RobotPose) -> Future:
        """Place an object at the given pose."""
        approach_pose = self._approach_pose(pose)

        steps = [
            lambda: self.robot.move_to(approach_pose),
            lambda: self.robot.move_to(pose),
            self.gripper.deactivate_tool,
            lambda: self.robot.move_to(approach_pose),
        ]

        return self._run_sequence(
            steps,
            "Object placed successfully"
        )
