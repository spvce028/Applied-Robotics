"""Asynchronous sequential waypoint execution."""

from collections.abc import Callable, Iterable

from rclpy.task import Future

from .models import OperationResult, RobotPose


class WaypointExecutor:
    """Execute robot movements through an ordered sequence of poses."""

    def __init__(
        self,
        move_to: Callable[[RobotPose], Future],
        is_cancelled: Callable[[], bool] = lambda: False,
    ):
        """Initialize movement and cancellation dependencies."""
        self._move_to = move_to
        self._is_cancelled = is_cancelled

    @staticmethod
    def _completed_future(result: OperationResult) -> Future:
        """Return an already-completed Future."""
        future = Future()
        future.set_result(result)
        return future

    def _as_future(self, result) -> Future:
        """Normalize asynchronous and legacy synchronous results."""
        if isinstance(result, Future):
            return result

        if isinstance(result, OperationResult):
            return self._completed_future(result)

        raise TypeError(
            "move_to() must return Future or OperationResult."
        )

    def move_through_waypoints(
        self,
        poses: Iterable[RobotPose],
    ) -> Future:
        """Move through every pose sequentially without blocking."""
        sequence_future = Future()
        pose_iterator = iter(poses)

        def finish(result):
            if not sequence_future.done():
                sequence_future.set_result(result)

        def handle_movement_result(movement_future):
            if sequence_future.done():
                return

            try:
                result = movement_future.result()

                if not isinstance(result, OperationResult):
                    raise TypeError(
                        "Movement Future must contain OperationResult."
                    )

                if not result.success:
                    finish(result)
                    return

                move_next()

            except Exception as exc:
                finish(
                    OperationResult(
                        success=False,
                        message=(
                            "Waypoint movement failed: "
                            f"{exc}"
                        ),
                        error_code="MOVE_FAILED",
                    )
                )

        def move_next():
            if sequence_future.done():
                return

            try:
                if self._is_cancelled():
                    finish(
                        OperationResult(
                            success=False,
                            message="Waypoint sequence cancelled",
                            error_code="CANCELLED",
                        )
                    )
                    return

            except Exception as exc:
                finish(
                    OperationResult(
                        success=False,
                        message=f"Cancellation check failed: {exc}",
                        error_code="WORKFLOW_ERROR",
                    )
                )
                return

            try:
                pose = next(pose_iterator)

            except StopIteration:
                finish(
                    OperationResult(
                        success=True,
                        message="Waypoint sequence completed",
                    )
                )
                return

            except Exception as exc:
                finish(
                    OperationResult(
                        success=False,
                        message=f"Could not read next waypoint: {exc}",
                        error_code="WORKFLOW_ERROR",
                    )
                )
                return

            try:
                movement_result = self._move_to(pose)
                movement_future = self._as_future(movement_result)
                movement_future.add_done_callback(
                    handle_movement_result
                )

            except Exception as exc:
                finish(
                    OperationResult(
                        success=False,
                        message=(
                            "Could not start waypoint movement: "
                            f"{exc}"
                        ),
                        error_code="MOVE_FAILED",
                    )
                )

        move_next()
        return sequence_future
