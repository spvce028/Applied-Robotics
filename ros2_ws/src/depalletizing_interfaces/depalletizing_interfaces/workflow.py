"""High-level pick-and-place workflow orchestration."""

from collections.abc import Callable

from .models import OperationResult, RobotPose


class PickAndPlaceWorkflow:
    """Coordinate picking and placing an object."""

    def __init__(
        self,
        pick_object: Callable[[RobotPose], OperationResult],
        place_object: Callable[[RobotPose], OperationResult],
        is_cancelled: Callable[[], bool] = lambda: False,
    ):
        """Initialize the workflow with its dependencies."""
        self._pick_object = pick_object
        self._place_object = place_object
        self._is_cancelled = is_cancelled
        self.status = 'idle'

    def execute_pick_and_place(
        self,
        source: RobotPose,
        target: RobotPose,
    ) -> OperationResult:
        """Execute picking followed by placing."""
        self.status = 'running'

        if self._is_cancelled():
            self.status = 'cancelled'
            return OperationResult(
                success=False,
                message='Workflow cancelled before picking',
                error_code='CANCELLED',
            )

        try:
            self.status = 'picking'
            pick_result = self._pick_object(source)

            if not pick_result.success:
                self.status = 'failed'
                return OperationResult(
                    success=False,
                    message=f'Picking failed: {pick_result.message}',
                    error_code=pick_result.error_code,
                )

            if self._is_cancelled():
                self.status = 'cancelled'
                return OperationResult(
                    success=False,
                    message='Workflow cancelled after picking; '
                            'object may still be held',
                    error_code='CANCELLED',
                )

            self.status = 'placing'
            place_result = self._place_object(target)

            if not place_result.success:
                self.status = 'failed'
                return OperationResult(
                    success=False,
                    message=f'Placing failed: {place_result.message}',
                    error_code=place_result.error_code,
                )

            self.status = 'completed'
            return OperationResult(
                success=True,
                message='Pick-and-place completed successfully',
            )

        except Exception as error:
            self.status = 'failed'
            return OperationResult(
                success=False,
                message=f'Workflow error: {error}',
                error_code='WORKFLOW_ERROR',
            )
