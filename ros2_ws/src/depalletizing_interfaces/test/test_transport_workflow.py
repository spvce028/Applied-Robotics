"""Tests for waypoint transport between picking and placing."""

import pytest

from rclpy.task import Future

from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_interfaces.workflow import PickAndPlaceWorkflow


def test_transport_runs_between_pick_and_place():
    """Wait for transport before starting placement."""
    calls = []
    pick_future = Future()
    transport_future = Future()
    place_future = Future()

    source = RobotPose(150, 0, 50, 0)
    target = RobotPose(250, 100, 50, 0)
    waypoints = (
        RobotPose(150, 0, 120, 0),
        RobotPose(250, 100, 120, 0),
    )

    def pick(pose):
        calls.append(('pick', pose))
        return pick_future

    def transport(poses):
        calls.append(('transport', tuple(poses)))
        return transport_future

    def place(pose):
        calls.append(('place', pose))
        return place_future

    workflow = PickAndPlaceWorkflow(
        pick_object=pick,
        place_object=place,
        move_through_waypoints=transport,
    )

    result_future = workflow.execute_pick_and_place(
        source,
        target,
        waypoints,
    )

    assert calls == [('pick', source)]
    assert workflow.status == 'picking'
    assert result_future.done() is False

    pick_future.set_result(OperationResult(success=True))

    assert calls == [
        ('pick', source),
        ('transport', waypoints),
    ]
    assert workflow.status == 'transporting'
    assert result_future.done() is False

    transport_future.set_result(OperationResult(success=True))

    assert calls == [
        ('pick', source),
        ('transport', waypoints),
        ('place', target),
    ]
    assert workflow.status == 'placing'
    assert result_future.done() is False

    place_future.set_result(OperationResult(success=True))

    assert result_future.result().success is True
    assert workflow.status == 'completed'


@pytest.mark.parametrize(
    'error_code',
    ['MOVE_FAILED', 'TIMEOUT', 'CANCELLED'],
)
def test_transport_failure_prevents_placement(error_code):
    """Do not place after a failed transport operation."""
    place_calls = []

    source = RobotPose(150, 0, 50, 0)
    target = RobotPose(250, 100, 50, 0)
    waypoints = [RobotPose(200, 50, 120, 0)]

    def transport(poses):
        return OperationResult(
            success=False,
            message='Transport did not complete',
            error_code=error_code,
        )

    workflow = PickAndPlaceWorkflow(
        pick_object=lambda pose: OperationResult(success=True),
        place_object=lambda pose: place_calls.append(pose),
        move_through_waypoints=transport,
    )

    result = workflow.execute_pick_and_place(
        source,
        target,
        waypoints,
    ).result()

    assert result.success is False
    assert result.error_code == error_code
    assert place_calls == []

    expected_status = (
        'cancelled'
        if error_code == 'CANCELLED'
        else 'failed'
    )
    assert workflow.status == expected_status


def test_cancellation_after_transport_prevents_placement():
    """Do not place when cancellation occurs during transport."""
    cancelled = False
    place_calls = []

    source = RobotPose(150, 0, 50, 0)
    target = RobotPose(250, 100, 50, 0)
    waypoints = [RobotPose(200, 50, 120, 0)]

    def transport(poses):
        nonlocal cancelled
        cancelled = True
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=lambda pose: OperationResult(success=True),
        place_object=lambda pose: place_calls.append(pose),
        is_cancelled=lambda: cancelled,
        move_through_waypoints=transport,
    )

    result = workflow.execute_pick_and_place(
        source,
        target,
        waypoints,
    ).result()

    assert result.success is False
    assert result.error_code == 'CANCELLED'
    assert workflow.status == 'cancelled'
    assert place_calls == []


def test_empty_waypoints_preserve_pick_and_place():
    """Skip transport when no waypoints are configured."""
    calls = []

    source = RobotPose(150, 0, 50, 0)
    target = RobotPose(250, 100, 50, 0)

    def transport(poses):
        calls.append(('transport', tuple(poses)))
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=lambda pose: (
            calls.append(('pick', pose))
            or OperationResult(success=True)
        ),
        place_object=lambda pose: (
            calls.append(('place', pose))
            or OperationResult(success=True)
        ),
        move_through_waypoints=transport,
    )

    result = workflow.execute_pick_and_place(
        source,
        target,
    ).result()

    assert result.success is True
    assert calls == [
        ('pick', source),
        ('place', target),
    ]


def test_waypoints_require_transport_operation():
    """Reject waypoints when no transport operation is configured."""
    place_calls = []

    source = RobotPose(150, 0, 50, 0)
    target = RobotPose(250, 100, 50, 0)
    waypoints = [RobotPose(200, 50, 120, 0)]

    workflow = PickAndPlaceWorkflow(
        pick_object=lambda pose: OperationResult(success=True),
        place_object=lambda pose: place_calls.append(pose),
    )

    result = workflow.execute_pick_and_place(
        source,
        target,
        waypoints,
    ).result()

    assert result.success is False
    assert result.error_code == 'WORKFLOW_ERROR'
    assert workflow.status == 'failed'
    assert place_calls == []
