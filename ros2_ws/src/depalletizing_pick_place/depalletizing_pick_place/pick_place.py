
"""Pick-and-place primitives for the depalletizing robot."""

from rclpy.task import Future

from depalletizing_interfaces.models import RobotPose, OperationResult


class PickPlace:
    """Implement pick and place operations using robot and gripper interfaces."""

    def __init__(self, robot, gripper):
        """Initialize with robot and gripper instances."""
        self.robot = robot
        self.gripper = gripper

    def _completed_future(self, result: OperationResult) -> Future:
        """Wrap synchronous gripper results in a Future."""
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
                OperationResult(
                    success=False,
                    message=f"Operation error: {exc}"
                )
            )

    def _run_sequence(self, steps, success_message: str) -> Future:
        """
        Execute operations sequentially without blocking.

        Each step returns a Future[OperationResult].
        The next step starts only after the previous one succeeds.
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
                operation_future = steps[index]()

                operation_future.add_done_callback(
                    lambda future: handle_step_result(future, index)
                )

            except Exception as exc:
                workflow_future.set_result(
                    OperationResult(
                        success=False,
                        message=f"Workflow execution error: {exc}"
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
                    OperationResult(
                        success=False,
                        message=f"Operation failed: {exc}"
                    )
                )

        execute_step(0)

        return workflow_future

    def pick_object(self, pose: RobotPose) -> Future:
        """Pick an object from the given pose."""
        approach_pose = RobotPose(
            x=pose.x,
            y=pose.y,
            z=pose.z + 50.0,
            r=pose.r,
            frame_id=pose.frame_id,
        )

        steps = [
            lambda: self.robot.move_to(approach_pose),
            lambda: self.robot.move_to(pose),
            lambda: self._completed_future(
                self.gripper.activate_tool()
            ),
            lambda: self.robot.move_to(approach_pose),
        ]

        return self._run_sequence(
            steps,
            "Object picked successfully"
        )

    def place_object(self, pose: RobotPose) -> Future:
        """Place an object at the given pose."""
        approach_pose = RobotPose(
            x=pose.x,
            y=pose.y,
            z=pose.z + 50.0,
            r=pose.r,
            frame_id=pose.frame_id,
        )

        steps = [
            lambda: self.robot.move_to(approach_pose),
            lambda: self.robot.move_to(pose),
            lambda: self._completed_future(
                self.gripper.deactivate_tool()
            ),
            lambda: self.robot.move_to(approach_pose),
        ]

        return self._run_sequence(
            steps,
            "Object placed successfully"
        )

    def run_full_workflow(self) -> Future:
        """
        Execute the complete depalletizing workflow asynchronously.

        Replace example coordinates with calibrated poses.
        """

        pick_pose = RobotPose(
            x=200.0,
            y=0.0,
            z=50.0,
            r=0.0,
            frame_id="base_link"
        )

        place_pose = RobotPose(
            x=250.0,
            y=100.0,
            z=50.0,
            r=0.0,
            frame_id="base_link"
        )

        steps = [
            lambda: self.pick_object(pick_pose),
            lambda: self.place_object(place_pose),
        ]

        return self._run_sequence(
            steps,
            "Depalletizing workflow completed successfully."
        )
