
"""Pick-and-place primitives for the depalletizing robot."""

from collections.abc import Callable

from rclpy.task import Future

from depalletizing_interfaces.errors import ERROR_CANCELLED, ERROR_WORKFLOW_ERROR
from depalletizing_interfaces.models import RobotPose, OperationResult


# Dobot PointToPoint motion types. P1 passes these through to the action goal.
MOVJ = 1
MOVL = 2

HELD_NOTE = "Object is still held; gripper remains closed."


class PickPlace:
    """Implement pick and place operations using robot and gripper interfaces."""

    def __init__(
        self,
        robot,
        gripper,
        approach_height=50.0,
        is_cancelled: Callable[[], bool] | None = None,
    ):
        """Initialize with robot, gripper, approach height, and cancellation."""
        self.robot = robot
        self.gripper = gripper
        self.approach_height = approach_height
        self.holding_object = False
        self._is_cancelled = (
            is_cancelled if is_cancelled is not None else (lambda: False)
        )

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
                    ERROR_WORKFLOW_ERROR,
                )
            )

    def _run_sequence(self, steps, success_message: str) -> Future:
        """
        Execute operations sequentially without blocking.

        Each step may return a Future or an OperationResult.
        Cancellation is checked before every step. An active movement is
        stopped by the robot driver, not by this sequence.
        Failure, cancellation, and timeout all stop the sequence.
        """
        workflow_future = Future()

        def finish(result):
            if not workflow_future.done():
                workflow_future.set_result(self._with_safe_state(result))

        def execute_step(index):
            if workflow_future.done():
                return

            if index >= len(steps):
                workflow_future.set_result(
                    OperationResult(
                        success=True,
                        message=success_message
                    )
                )
                return

            try:
                if self._is_cancelled():
                    finish(self._failed_result(
                        "Cancelled before the next step",
                        ERROR_CANCELLED,
                    ))
                    return
            except Exception as exc:
                finish(self._failed_result(
                    f"Cancellation check failed: {exc}",
                    ERROR_WORKFLOW_ERROR,
                ))
                return

            try:
                _kind, operation = steps[index]
                operation_future = self._as_future(operation)

                operation_future.add_done_callback(
                    lambda future: handle_step_result(future, index)
                )

            except Exception as exc:
                finish(self._failed_result(
                    f"Workflow execution error: {exc}",
                    ERROR_WORKFLOW_ERROR,
                ))

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
                    finish(result)
                    return

                kind, _operation = steps[index]
                if kind == "grasp":
                    self.holding_object = True
                elif kind == "release":
                    self.holding_object = False

                execute_step(index + 1)

            except Exception as exc:
                finish(self._failed_result(
                    f"Operation failed: {exc}",
                    ERROR_WORKFLOW_ERROR,
                ))

        execute_step(0)

        return workflow_future

    def _with_safe_state(self, result: OperationResult) -> OperationResult:
        """Keep the gripper closed when a failure happens while holding."""
        if result.success or not self.holding_object:
            return result
        if HELD_NOTE in result.message:
            return result
        message = f"{result.message} {HELD_NOTE}".strip()
        return OperationResult(
            success=False,
            message=message,
            error_code=result.error_code,
        )

    def _move(self, pose: RobotPose, motion_type: int):
        """Return a step that moves with the given Dobot motion type."""
        return lambda: self.robot.move_to(pose, motion_type=motion_type)

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
        """Pick an object from the given pose.

        Travel to the approach pose with MOVJ. Descend and lift with MOVL.
        """
        if self.holding_object:
            return self._completed_future(self._failed_result(
                "Cannot pick while an object is still held",
                ERROR_WORKFLOW_ERROR,
            ))

        approach_pose = self._approach_pose(pose)
        steps = [
            ("approach", self._move(approach_pose, MOVJ)),
            ("descend", self._move(pose, MOVL)),
            ("grasp", self.gripper.activate_tool),
            ("lift", self._move(approach_pose, MOVL)),
        ]

        return self._run_sequence(
            steps,
            "Object picked successfully"
        )

    def place_object(self, pose: RobotPose) -> Future:
        """Place an object at the given pose.

        Travel to the approach pose with MOVJ. Descend and lift with MOVL.
        The object stays grasped until the gripper opens successfully.
        """
        self.holding_object = True
        approach_pose = self._approach_pose(pose)
        steps = [
            ("approach", self._move(approach_pose, MOVJ)),
            ("descend", self._move(pose, MOVL)),
            ("release", self.gripper.deactivate_tool),
            ("lift", self._move(approach_pose, MOVL)),
        ]

        return self._run_sequence(
            steps,
            "Object placed successfully"
        )
