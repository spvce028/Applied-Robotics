"""Tests for the pick-and-place workflow."""

from depalletizing_interfaces.mocks import MockGripper, MockRobot
from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_interfaces.workflow import PickAndPlaceWorkflow


def test_successful_workflow():
    """Verify that picking happens before placing."""
    calls = []

    def mock_pick(source):
        calls.append(('pick', source))
        return OperationResult(success=True)

    def mock_place(target):
        calls.append(('place', target))
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    source = RobotPose(150, 0, 100, 0)
    target = RobotPose(200, 0, 100, 0)

    result = workflow.execute_pick_and_place(source, target)

    assert result.success is True
    assert workflow.status == 'completed'
    assert calls == [
        ('pick', source),
        ('place', target),
    ]


def test_pick_failure_stops_workflow():
    """Verify that placing is skipped when picking fails."""
    calls = []

    def mock_pick(source):
        calls.append('pick')
        return OperationResult(
            success=False,
            message='Object could not be picked',
            error_code='PICK_FAILED',
        )

    def mock_place(target):
        calls.append('place')
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'PICK_FAILED'
    assert workflow.status == 'failed'
    assert calls == ['pick']


def test_place_failure_stops_workflow():
    """Verify that a placing failure is reported correctly."""
    calls = []

    def mock_pick(source):
        calls.append('pick')
        return OperationResult(
            success=True,
            message='Object picked successfully',
        )

    def mock_place(target):
        calls.append('place')
        return OperationResult(
            success=False,
            message='Object could not be placed',
            error_code='PLACE_FAILED',
        )

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'PLACE_FAILED'
    assert workflow.status == 'failed'
    assert calls == ['pick', 'place']


def test_cancellation_before_picking():
    """Verify that cancellation prevents all operations."""
    calls = []

    def mock_pick(source):
        calls.append('pick')
        return OperationResult(success=True)

    def mock_place(target):
        calls.append('place')
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
        is_cancelled=lambda: True,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'CANCELLED'
    assert workflow.status == 'cancelled'
    assert calls == []


def test_cancellation_after_picking():
    """Verify that cancellation after picking prevents placing."""
    calls = []
    cancelled = False

    def mock_pick(source):
        nonlocal cancelled
        calls.append('pick')

        # Simulate cancellation during the picking operation.
        cancelled = True

        return OperationResult(
            success=True,
            message='Object picked successfully',
        )

    def mock_place(target):
        calls.append('place')
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
        is_cancelled=lambda: cancelled,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'CANCELLED'
    assert workflow.status == 'cancelled'
    assert calls == ['pick']
    assert 'object may still be held' in result.message


def test_exception_during_picking():
    """Verify that exceptions stop the workflow."""
    calls = []

    def mock_pick(source):
        calls.append('pick')
        raise RuntimeError('Robot connection lost')

    def mock_place(target):
        calls.append('place')
        return OperationResult(success=True)

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'WORKFLOW_ERROR'
    assert workflow.status == 'failed'
    assert 'Robot connection lost' in result.message
    assert calls == ['pick']


def test_exception_during_placing():
    """Verify that placing exceptions are handled correctly."""
    calls = []

    def mock_pick(source):
        calls.append('pick')
        return OperationResult(
            success=True,
            message='Object picked successfully',
        )

    def mock_place(target):
        calls.append('place')
        raise RuntimeError('Robot connection lost during placing')

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'WORKFLOW_ERROR'
    assert workflow.status == 'failed'
    assert 'Robot connection lost during placing' in result.message
    assert calls == ['pick', 'place']


def test_workflow_with_shared_mocks():
    """Verify workflow integration with shared robot and gripper mocks."""
    robot = MockRobot()
    gripper = MockGripper()

    source = RobotPose(150, 0, 100, 0)
    target = RobotPose(200, 0, 100, 0)

    def mock_pick(pose):
        result = robot.move_to(pose)

        if not result.success:
            return result

        return gripper.activate_tool()

    def mock_place(pose):
        result = robot.move_to(pose)

        if not result.success:
            return result

        return gripper.deactivate_tool()

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    result = workflow.execute_pick_and_place(source, target)

    assert result.success is True
    assert workflow.status == 'completed'
    assert robot.movement_history == [source, target]
    assert gripper.is_active is False


def test_workflow_with_robot_movement_failure():
    """Verify that robot movement failure stops the workflow."""
    robot = MockRobot()
    gripper = MockGripper()

    robot.should_fail = True

    calls = []

    def mock_pick(pose):
        calls.append('pick')

        result = robot.move_to(pose)

        if not result.success:
            return result

        return gripper.activate_tool()

    def mock_place(pose):
        calls.append('place')

        result = robot.move_to(pose)

        if not result.success:
            return result

        return gripper.deactivate_tool()

    workflow = PickAndPlaceWorkflow(
        pick_object=mock_pick,
        place_object=mock_place,
    )

    result = workflow.execute_pick_and_place(
        source=RobotPose(150, 0, 100, 0),
        target=RobotPose(200, 0, 100, 0),
    )

    assert result.success is False
    assert result.error_code == 'MOVE_FAILED'
    assert workflow.status == 'failed'
    assert calls == ['pick']
    assert robot.movement_history == []
    assert gripper.is_active is False
