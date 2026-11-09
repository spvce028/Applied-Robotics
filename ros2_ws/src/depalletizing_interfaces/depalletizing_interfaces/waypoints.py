"""Sequential waypoint execution."""

from collections.abc import Callable, Iterable

from .models import OperationResult, RobotPose


class WaypointExecutor:
    """Execute robot movements through an ordered sequence of poses."""

    def __init__(
        self,
        move_to: Callable[[RobotPose], OperationResult],
        is_cancelled: Callable[[], bool] = lambda: False,
    ):
        """Initialize the executor with movement and cancellation functions."""
        self._move_to = move_to
        self._is_cancelled = is_cancelled

    def move_through_waypoints(
        self,
        poses: Iterable[RobotPose],
    ) -> OperationResult:
        """Move through every pose sequentially."""
        for pose in poses:
            if self._is_cancelled():
                return OperationResult(
                    success=False,
                    message="Waypoint sequence cancelled",
                    error_code="CANCELLED",
                )

            result = self._move_to(pose)

            if not result.success:
                return result

        return OperationResult(
            success=True,
            message="Waypoint sequence completed",
        )
