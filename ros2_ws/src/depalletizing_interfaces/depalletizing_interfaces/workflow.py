"""High-level asynchronous pick-and-place workflow orchestration."""

from collections.abc import Callable
from threading import Lock

from rclpy.task import Future

from .models import OperationResult, RobotPose


class PickAndPlaceWorkflow:
    """
    Coordinate operations without blocking the ROS executor.
    
    Operations return a Future containing OperationResult. Direct results are
    accepted for legacy mocks. Cancellation is checked between operations;
    stopping an active physical operation belongs to the robot driver.
    """

    def __init__(
        self,
        pick_object: Callable[[RobotPose], Future],
        place_object: Callable[[RobotPose], Future],
        is_cancelled: Callable[[], bool] = lambda: False,
    ):
        """Initialize operation and cancellation dependencies."""
        self._pick_object = pick_object
        self._place_object = place_object
        self._is_cancelled = is_cancelled
        self._lock = Lock()
        self._running = False
        self.status = 'idle'

    def execute_pick_and_place(
        self, source: RobotPose, target: RobotPose,
    ) -> Future:
        """Return a Future resolved after picking and then placing finish."""
        workflow_future = Future()
        with self._lock:
            if self._running:
                workflow_future.set_result(OperationResult(
                    False, 'A workflow is already running', 'WORKFLOW_BUSY',
                ))
                return workflow_future
            self._running = True
            self.status = 'running'

        def finish(result, status):
            with self._lock:
                self.status = status
                self._running = False
            workflow_future.set_result(result)

        def fail(error):
            finish(OperationResult(
                False, f'Workflow error: {error}', 'WORKFLOW_ERROR',
            ), 'failed')

        def start(operation, pose, phase):
            self.status = phase
            result = operation(pose)
            if isinstance(result, OperationResult):
                future = Future()
                future.set_result(result)
            elif isinstance(result, Future):
                future = result
            else:
                raise TypeError('Operation must return Future or OperationResult')
            future.add_done_callback(lambda done: completed(done, phase))

        def completed(future, phase):
            try:
                result = future.result()
                if not isinstance(result, OperationResult):
                    raise TypeError('Operation Future must contain OperationResult')
                if not result.success:
                    finish(OperationResult(
                        False, f'{phase.capitalize()} failed: {result.message}',
                        result.error_code,
                    ), 'cancelled' if result.error_code == 'CANCELLED' else 'failed')
                elif phase == 'picking':
                    if self._is_cancelled():
                        finish(OperationResult(
                            False, 'Workflow cancelled after picking; '
                            'object may still be held', 'CANCELLED',
                        ), 'cancelled')
                    else:
                        start(self._place_object, target, 'placing')
                else:
                    finish(OperationResult(
                        True, 'Pick-and-place completed successfully',
                    ), 'completed')
            except Exception as error:
                fail(error)

        try:
            if self._is_cancelled():
                finish(OperationResult(
                    False, 'Workflow cancelled before picking', 'CANCELLED',
                ), 'cancelled')
            else:
                start(self._pick_object, source, 'picking')
        except Exception as error:
            fail(error)
        return workflow_future
