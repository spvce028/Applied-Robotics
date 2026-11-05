"""Tests for sequential waypoint execution."""

from unittest.mock import Mock, call

from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_interfaces.waypoints import WaypointExecutor


def make_poses():
    """Create a standard three-pose test sequence."""
    return [
        RobotPose(100, 0, 50, 0),
        RobotPose(150, 0, 50, 0),
        RobotPose(200, 0, 50, 0),
    ]


def test_all_waypoints_succeed():
    """Execute every waypoint when all movements succeed."""
    poses = make_poses()
    move_to = Mock(
        return_value=OperationResult(success=True)
    )
    executor = WaypointExecutor(move_to)

    result = executor.move_through_waypoints(poses)

    assert result.success is True
    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
        call(poses[2]),
    ]


def test_failure_stops_remaining_movements():
    """Stop without executing later waypoints after a failure."""
    poses = make_poses()
    failure = OperationResult(
        success=False,
        message="Movement failed",
        error_code="MOVE_FAILED",
    )
    move_to = Mock(
        side_effect=[
            OperationResult(success=True),
            failure,
        ]
    )
    executor = WaypointExecutor(move_to)

    result = executor.move_through_waypoints(poses)

    assert result is failure
    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
    ]


def test_cancelled_movement_stops_sequence():
    """Stop without executing later waypoints after cancellation."""
    poses = make_poses()
    cancelled = OperationResult(
        success=False,
        message="Movement cancelled",
        error_code="CANCELLED",
    )
    move_to = Mock(
        side_effect=[
            OperationResult(success=True),
            cancelled,
        ]
    )
    executor = WaypointExecutor(move_to)

    result = executor.move_through_waypoints(poses)

    assert result is cancelled
    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
    ]


def test_cancellation_before_next_waypoint():
    """Stop before starting the next waypoint when cancelled."""
    poses = make_poses()
    move_to = Mock(
        return_value=OperationResult(success=True)
    )
    is_cancelled = Mock(side_effect=[False, True])
    executor = WaypointExecutor(move_to, is_cancelled)

    result = executor.move_through_waypoints(poses)

    assert result.success is False
    assert result.error_code == "CANCELLED"
    move_to.assert_called_once_with(poses[0])


def test_empty_sequence_succeeds():
    """Return success without movement for an empty sequence."""
    move_to = Mock()
    executor = WaypointExecutor(move_to)

    result = executor.move_through_waypoints([])

    assert result.success is True
    move_to.assert_not_called()
