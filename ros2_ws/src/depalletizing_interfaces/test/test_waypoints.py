"""Tests for asynchronous sequential waypoint execution."""

from unittest.mock import Mock, call

from rclpy.task import Future

from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_interfaces.waypoints import WaypointExecutor


def make_poses():
    """Create a standard three-pose sequence."""
    return [
        RobotPose(100, 0, 50, 0),
        RobotPose(150, 0, 50, 0),
        RobotPose(200, 0, 50, 0),
    ]


def completed_future(result):
    """Create a Future containing the supplied result."""
    future = Future()
    future.set_result(result)
    return future


def successful_result():
    """Create a successful movement result."""
    return OperationResult(success=True)


def test_waypoints_execute_strictly_sequentially():
    """Start each movement only after the previous one succeeds."""
    poses = make_poses()
    movement_futures = [Future(), Future(), Future()]
    move_to = Mock(side_effect=movement_futures)
    executor = WaypointExecutor(move_to)

    sequence_future = executor.move_through_waypoints(poses)

    assert move_to.call_args_list == [call(poses[0])]
    assert sequence_future.done() is False

    movement_futures[0].set_result(successful_result())

    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
    ]

    movement_futures[1].set_result(successful_result())

    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
        call(poses[2]),
    ]

    movement_futures[2].set_result(successful_result())

    assert sequence_future.done() is True
    assert sequence_future.result().success is True


def test_failure_stops_remaining_movements():
    """Stop without starting later waypoints after failure."""
    poses = make_poses()
    first_movement = Future()
    second_movement = Future()
    move_to = Mock(
        side_effect=[first_movement, second_movement]
    )
    executor = WaypointExecutor(move_to)

    sequence_future = executor.move_through_waypoints(poses)

    first_movement.set_result(successful_result())

    failure = OperationResult(
        success=False,
        message="Movement failed",
        error_code="MOVE_FAILED",
    )
    second_movement.set_result(failure)

    assert sequence_future.result() is failure
    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
    ]


def test_cancelled_movement_stops_sequence():
    """Stop after a movement returns cancellation."""
    poses = make_poses()
    first_movement = Future()
    second_movement = Future()
    move_to = Mock(
        side_effect=[first_movement, second_movement]
    )
    executor = WaypointExecutor(move_to)

    sequence_future = executor.move_through_waypoints(poses)

    first_movement.set_result(successful_result())

    cancelled = OperationResult(
        success=False,
        message="Movement cancelled",
        error_code="CANCELLED",
    )
    second_movement.set_result(cancelled)

    assert sequence_future.result() is cancelled
    assert move_to.call_args_list == [
        call(poses[0]),
        call(poses[1]),
    ]


def test_cancellation_before_next_waypoint():
    """Stop before the next waypoint when cancellation is requested."""
    poses = make_poses()
    first_movement = Future()
    move_to = Mock(return_value=first_movement)
    is_cancelled = Mock(side_effect=[False, True])
    executor = WaypointExecutor(move_to, is_cancelled)

    sequence_future = executor.move_through_waypoints(poses)

    first_movement.set_result(successful_result())

    result = sequence_future.result()

    assert result.success is False
    assert result.error_code == "CANCELLED"
    move_to.assert_called_once_with(poses[0])


def test_timeout_stops_remaining_movements():
    """Propagate a movement timeout without starting another pose."""
    poses = make_poses()
    timeout = OperationResult(
        success=False,
        message="Movement timed out",
        error_code="TIMEOUT",
    )
    move_to = Mock(return_value=completed_future(timeout))
    executor = WaypointExecutor(move_to)

    sequence_future = executor.move_through_waypoints(poses)

    assert sequence_future.result() is timeout
    move_to.assert_called_once_with(poses[0])


def test_move_to_exception_stops_sequence():
    """Convert a movement exception into a failure result."""
    poses = make_poses()
    move_to = Mock(
        side_effect=RuntimeError("Action client unavailable")
    )
    executor = WaypointExecutor(move_to)

    sequence_future = executor.move_through_waypoints(poses)
    result = sequence_future.result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    move_to.assert_called_once_with(poses[0])


def test_empty_sequence_succeeds():
    """Return success without requesting movement."""
    move_to = Mock()
    executor = WaypointExecutor(move_to)

    sequence_future = executor.move_through_waypoints([])
    result = sequence_future.result()

    assert result.success is True
    move_to.assert_not_called()
