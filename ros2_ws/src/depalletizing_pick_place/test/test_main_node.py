"""Exercise the real ROS start service with mocked hardware."""

import time
from unittest.mock import Mock, patch

import pytest

pytest.importorskip('rclpy.node')
pytest.importorskip('dobot_msgs.action')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_srvs.srv import Trigger

from depalletizing_interfaces.models import OperationResult
from depalletizing_pick_place.main_node import DepalletizingMainNode


# Help-Function for Param-Testing
def make_test_parameters():
    """Return a complete configuration for the test node."""
    values = {
        'pick_x': 200.0,
        'pick_y': 0.0,
        'pick_z': 50.0,
        'pick_r': 0.0,
        'place_x': 250.0,
        'place_y': 100.0,
        'place_z': 50.0,
        'place_r': 0.0,
        'velocity_ratio': 0.6,
        'acceleration_ratio': 0.3,
        'movement_timeout': 12.0,
        'approach_height': 50.0,
        'gripper_service_name': '/test/gripper',
        'gripper_timeout': 2.0,
        'keep_compressor_running': True,
    }

    return [
        Parameter(name, value=value)
        for name, value in values.items()
    ]


@pytest.mark.parametrize('fail_pick', [False, True])
def test_start_service_runs_async_workflow(fail_pick):
    """Drive the service, P5 workflow and P4 primitives through real ROS."""
    class Robot:
        is_faulted = False
        is_busy = False

        def __init__(
            self,
            node,
            *,
            velocity_ratio,
            acceleration_ratio,
            movement_timeout,
            workspace_limits=None,
        ):
            self.velocity_ratio = velocity_ratio
            self.acceleration_ratio = acceleration_ratio
            self.movement_timeout = movement_timeout
            self.movements = []
            self.pending = None

        def server_is_ready(self):
            return True

        def move_to(self, pose, motion_type=1):
            self.movements.append(pose)
            self.pending = Future()
            return self.pending

    class Gripper:
        def __init__(
            self,
            node,
            *,
            service_name,
            timeout_sec,
            keep_compressor_running,
        ):
            self.service_name = service_name
            self.timeout_sec = timeout_sec
            self.keep_compressor_running = keep_compressor_running

        def service_is_ready(self):
            return True

        def activate_tool(self):
            return OperationResult(success=True, message="Gripper activated")

        def deactivate_tool(self):
            return OperationResult(success=True, message="Gripper deactivated")

    rclpy.init()
    executor = SingleThreadedExecutor()
    node = None
    client_node = None

    try:
        with patch('depalletizing_pick_place.main_node.DobotActionRobot', Robot), patch('depalletizing_pick_place.main_node.GripperClient', Gripper):
            node = DepalletizingMainNode(
                parameter_overrides=make_test_parameters(),
            )

        assert node.robot.velocity_ratio == 0.6
        assert node.robot.acceleration_ratio == 0.3
        assert node.robot.movement_timeout == 12.0

        assert node.gripper.service_name == '/test/gripper'
        assert node.gripper.timeout_sec == 2.0
        assert node.gripper.keep_compressor_running is True

        assert node.pick_place.approach_height == 50.0

        client_node = rclpy.create_node('p5_workflow_test_client')
        executor.add_node(node)
        executor.add_node(client_node)

        assert not node._homing_confirmed
        assert node.robot.movements == []

        start_client = client_node.create_client(
            Trigger,
            '/start_depalletizing',
        )
        assert start_client.wait_for_service(timeout_sec=5.0)

        rejected_start = start_client.call_async(Trigger.Request())
        executor.spin_until_future_complete(
            rejected_start,
            timeout_sec=5.0,
        )

        assert rejected_start.done()
        assert not rejected_start.result().success
        assert 'Homing' in rejected_start.result().message
        assert not node._workflow_running
        assert node.robot.movements == []

        homing_client = client_node.create_client(
            Trigger,
            '/confirm_homing',
        )
        assert homing_client.wait_for_service(timeout_sec=5.0)

        for state in ('is_busy', 'is_faulted'):
            setattr(node.robot, state, True)

            rejected_confirmation = homing_client.call_async(Trigger.Request())
            executor.spin_until_future_complete(
                rejected_confirmation,
                timeout_sec=5.0,
            )

            assert rejected_confirmation.done()
            assert not rejected_confirmation.result().success
            assert not node._homing_confirmed
            assert node.robot.movements == []

            setattr(node.robot, state, False)

        confirmation = homing_client.call_async(Trigger.Request())
        executor.spin_until_future_complete(
            confirmation,
            timeout_sec=5.0,
        )

        assert confirmation.done()
        assert confirmation.result().success
        assert node._homing_confirmed
        assert node.robot.movements == []

        client = client_node.create_client(Trigger, '/start_depalletizing')
        assert client.wait_for_service(timeout_sec=5.0)

        for state, expected_message in (
            ('is_faulted', 'faulted'),
            ('is_busy', 'busy'),
        ):
            setattr(node.robot, state, True)

            rejected = client.call_async(Trigger.Request())
            executor.spin_until_future_complete(rejected, timeout_sec=5.0)

            assert rejected.done()
            assert not rejected.result().success
            assert expected_message in rejected.result().message
            assert not node._workflow_running
            assert node.robot.movements == []

            setattr(node.robot, state, False)

        with patch.object(node.robot, 'server_is_ready', return_value=False):
            rejected = client.call_async(Trigger.Request())
            executor.spin_until_future_complete(rejected, timeout_sec=5.0)

            assert rejected.done()
            assert not rejected.result().success
            assert 'Action Server' in rejected.result().message
            assert not node._workflow_running
            assert node.robot.movements == []

        response = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(response, timeout_sec=5.0)
        assert response.done()
        assert response.result().success
        assert node.workflow.status == 'picking'
        assert node._workflow_running

        movements_before = list(node.robot.movements)

        confirmation_while_running = homing_client.call_async(Trigger.Request())
        executor.spin_until_future_complete(
            confirmation_while_running,
            timeout_sec=5.0,
        )

        assert confirmation_while_running.done()
        assert not confirmation_while_running.result().success
        assert node._homing_confirmed
        assert node._workflow_running
        assert node.robot.movements == movements_before

        duplicate = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(duplicate, timeout_sec=5.0)
        assert duplicate.done()
        assert not duplicate.result().success

        deadline = time.monotonic() + 5.0
        while node._workflow_running and time.monotonic() < deadline:
            pending = node.robot.pending
            if pending is not None and not pending.done():
                pending.set_result(OperationResult(
                    not fail_pick, 'mock movement',
                    'MOVE_FAILED' if fail_pick else None,
                ))
            executor.spin_once(timeout_sec=0.01)
        assert not node._workflow_running
        assert node.workflow.status == ('failed' if fail_pick else 'completed')
        assert len(node.robot.movements) == (1 if fail_pick else 6)
        if not fail_pick:
            assert [(p.x, p.y, p.z) for p in node.robot.movements] == [
                (200, 0, 100), (200, 0, 50), (200, 0, 100),
                (250, 100, 100), (250, 100, 50), (250, 100, 100),
            ]
    finally:
        executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_start_refused_when_gripper_unavailable():
    class Robot: 
        is_faulted = False
        is_busy = False

        def __init__(
            self,
            node,
            *,
            velocity_ratio,
            acceleration_ratio,
            movement_timeout,
            workspace_limits=None,
        ):
            self.velocity_ratio = velocity_ratio
            self.acceleration_ratio = acceleration_ratio
            self.movement_timeout = movement_timeout

        def server_is_ready(self):
            return True

    class Gripper:
        def __init__(
            self,
            node,
            *,
            service_name,
            timeout_sec,
            keep_compressor_running,
        ):
            self.service_name = service_name
            self.timeout_sec = timeout_sec
            self.keep_compressor_running = keep_compressor_running

        def service_is_ready(self):
            return False

    rclpy.init()
    node = None
    try:
        with patch('depalletizing_pick_place.main_node.DobotActionRobot', Robot), patch('depalletizing_pick_place.main_node.GripperClient', Gripper):
            node = DepalletizingMainNode(
                parameter_overrides=make_test_parameters(),
            )

        confirmation = node.handle_confirm_homing(
            Trigger.Request(),
            Trigger.Response(),
        )
        assert confirmation.success

        response = node.handle_start_service(Trigger.Request(), Trigger.Response())

        assert not response.success
        assert 'Gripper' in response.message
        assert not node._workflow_running
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_missing_parameter_prevents_driver_creation():
    """Reject missing configuration before creating hardware clients."""
    parameters = [
        parameter
        for parameter in make_test_parameters()
        if parameter.name != 'pick_x'
    ]

    rclpy.init()
    try:
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot'
        ) as robot_class, patch(
            'depalletizing_pick_place.main_node.GripperClient'
        ) as gripper_class:
            with pytest.raises(
                RuntimeError,
                match='Missing required parameters: pick_x',
            ):
                DepalletizingMainNode(
                    parameter_overrides=parameters,
                )

            robot_class.assert_not_called()
            gripper_class.assert_not_called()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize(
    'name, value',
    [
        ('pick_x', float('nan')),
        ('place_z', float('inf')),
        ('velocity_ratio', 0.0),
        ('acceleration_ratio', 1.1),
        ('movement_timeout', -1.0),
        ('gripper_timeout', float('nan')),
        ('approach_height', 0.0),
        ('gripper_service_name', '   '),
    ],
)
def test_invalid_parameter_prevents_driver_creation(name, value):
    """Reject invalid configuration before creating hardware clients."""
    parameters = [
        parameter
        for parameter in make_test_parameters()
        if parameter.name != name
    ]
    parameters.append(Parameter(name, value=value))

    rclpy.init()
    try:
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot'
        ) as robot_class, patch(
            'depalletizing_pick_place.main_node.GripperClient'
        ) as gripper_class:
            with pytest.raises(RuntimeError, match=name):
                DepalletizingMainNode(
                    parameter_overrides=parameters,
                )

            robot_class.assert_not_called()
            gripper_class.assert_not_called()
    finally:
        rclpy.shutdown()


def test_invalid_workspace_limits_prevent_driver_creation():
    """Reject malformed workspace bounds before creating hardware clients."""
    parameters = make_test_parameters() + [
        Parameter('workspace_x_min', value=400.0),
    ]

    rclpy.init()
    try:
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot'
        ) as robot_class, patch(
            'depalletizing_pick_place.main_node.GripperClient'
        ) as gripper_class:
            with pytest.raises(RuntimeError, match='workspace'):
                DepalletizingMainNode(
                    parameter_overrides=parameters,
                )

            robot_class.assert_not_called()
            gripper_class.assert_not_called()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('scenario', ['idle', 'moving', 'driver_error'])
def test_cancel_service(scenario):
    """Acknowledge cancellation before completion and stop subsequent steps."""
    # Only the hardware is replaced; services, workflow and primitives are real.
    robot = Mock()
    robot.is_faulted = False
    robot.is_busy = False
    robot.server_is_ready.return_value = True
    gripper = Mock()
    gripper.service_is_ready.return_value = True

    rclpy.init()
    executor = SingleThreadedExecutor()
    node = None
    client_node = None
    try:
        movement = Future()
        robot.move_to.return_value = movement
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot',
            return_value=robot,
        ), patch(
            'depalletizing_pick_place.main_node.GripperClient',
            return_value=gripper,
        ):
            node = DepalletizingMainNode(
                parameter_overrides=make_test_parameters(),
            )

        client_node = rclpy.create_node('p5_cancel_test_client')
        executor.add_node(node)
        executor.add_node(client_node)
        client = client_node.create_client(Trigger, '/cancel_depalletizing')
        assert client.wait_for_service(timeout_sec=5.0)

        if scenario != 'idle':
            confirmation = node.handle_confirm_homing(
                Trigger.Request(), Trigger.Response(),
            )
            assert confirmation.success
            started = node.handle_start_service(
                Trigger.Request(), Trigger.Response(),
            )
            assert started.success
            robot.move_to.assert_called_once()
            robot.is_busy = True

            # Cancellation must remain available even if prerequisites are lost.
            node._homing_confirmed = False
            robot.server_is_ready.return_value = False
            gripper.service_is_ready.return_value = False

        if scenario == 'driver_error':
            robot.cancel_current_goal.side_effect = RuntimeError('connection lost')

        reply = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(reply, timeout_sec=5.0)
        assert reply.done()
        assert not node._cancel_dispatching

        if scenario == 'idle':
            assert not reply.result().success
            assert 'No active operation' in reply.result().message
            robot.cancel_current_goal.assert_not_called()
            robot.move_to.assert_not_called()
            assert not node._cancel_requested.is_set()
        else:
            assert reply.result().success == (scenario == 'moving')
            robot.cancel_current_goal.assert_called_once()
            assert node._cancel_requested.is_set()

            # A response is not a terminal movement result: keep the run occupied.
            assert not movement.done()
            assert node._workflow_running
            assert node.workflow.status == 'picking'

            # Even if the movement finishes successfully after the cancel request,
            # the next descent/grasp/place must not start (also on dispatch error).
            robot.is_busy = False
            movement.set_result(OperationResult(True, 'Movement ended'))
            deadline = time.monotonic() + 5.0
            while node._workflow_running and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert not node._workflow_running
            assert node.workflow.status == 'cancelled'
            robot.move_to.assert_called_once()

        gripper.activate_tool.assert_not_called()
        gripper.deactivate_tool.assert_not_called()
    finally:
        executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    'scenario',
    ['reset', 'no_fault', 'workflow', 'busy', 'cancel', 'refused', 'error'],
)
def test_reset_fault_service(scenario):
    """Reset only an idle driver and require homing confirmation afterward."""
    robot = Mock()
    robot.is_faulted = scenario != 'no_fault'
    robot.is_busy = scenario == 'busy'
    gripper = Mock()

    def reset_driver(*, force):
        assert force is False
        if scenario == 'error':
            raise RuntimeError('recovery unavailable')
        if scenario == 'refused':
            return False
        robot.is_faulted = False
        return True

    robot.reset_fault.side_effect = reset_driver
    rclpy.init()
    executor = SingleThreadedExecutor()
    node = None
    client_node = None
    try:
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot',
            return_value=robot,
        ), patch(
            'depalletizing_pick_place.main_node.GripperClient',
            return_value=gripper,
        ):
            node = DepalletizingMainNode(
                parameter_overrides=make_test_parameters(),
            )

        # Simulate a prior reference confirmation and an earlier cancel request.
        node._homing_confirmed = True
        node._cancel_requested.set()
        node._workflow_running = scenario == 'workflow'
        node._cancel_dispatching = scenario == 'cancel'
        client_node = rclpy.create_node('p5_reset_test_client')
        executor.add_node(node)
        executor.add_node(client_node)
        client = client_node.create_client(Trigger, '/reset_fault')
        assert client.wait_for_service(timeout_sec=5.0)
        reply = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(reply, timeout_sec=5.0)
        assert reply.done()
        assert reply.result().success == (scenario in ('reset', 'no_fault'))

        if scenario in ('workflow', 'busy', 'cancel', 'no_fault'):
            robot.reset_fault.assert_not_called()
            assert node._homing_confirmed
        else:
            robot.reset_fault.assert_called_once_with(force=False)
            assert not node._homing_confirmed

        if scenario == 'reset':
            assert not robot.is_faulted
            # Reset alone must not enable a new workflow.
            rejected = node.handle_start_service(
                Trigger.Request(), Trigger.Response(),
            )
            assert not rejected.success
            assert 'Homing' in rejected.message
        elif scenario in ('refused', 'error'):
            assert robot.is_faulted

        assert node._workflow_running == (scenario == 'workflow')
        assert node._cancel_dispatching == (scenario == 'cancel')
        assert node._cancel_requested.is_set()
        robot.move_to.assert_not_called()
        robot.cancel_current_goal.assert_not_called()
        gripper.activate_tool.assert_not_called()
        gripper.deactivate_tool.assert_not_called()
    finally:
        executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def manual_node(request):
    """Provide a real ROS node and executor with fake hardware clients."""
    robot = Mock()
    robot.is_faulted = False
    robot.is_busy = False
    robot.server_is_ready.return_value = True
    gripper = Mock()
    gripper.service_is_ready.return_value = True
    gripper.activate_tool.return_value = OperationResult(True, 'Closed')
    gripper.deactivate_tool.return_value = OperationResult(True, 'Opened')

    rclpy.init()
    executor = SingleThreadedExecutor()
    node = None
    client_node = None
    try:
        # Use a non-default height to verify that manual moves use configuration.
        parameters = [
            p for p in make_test_parameters() if p.name != 'approach_height'
        ]
        parameters.append(Parameter('approach_height', value=65.0))
        # Individual tests can supply additional configuration before node startup.
        for name, value in getattr(request, 'param', {}).items():
            parameters = [p for p in parameters if p.name != name]
            parameters.append(Parameter(name, value=value))
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot',
            return_value=robot,
        ), patch(
            'depalletizing_pick_place.main_node.GripperClient',
            return_value=gripper,
        ):
            node = DepalletizingMainNode(parameter_overrides=parameters)
        client_node = rclpy.create_node('p5_manual_test_client')
        executor.add_node(node)
        executor.add_node(client_node)
        yield node, client_node, executor, robot, gripper
    finally:
        executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def call_trigger(client_node, executor, service_name):
    """Call a real Trigger service with a bounded wait in the test only."""
    client = client_node.create_client(Trigger, service_name)
    try:
        assert client.wait_for_service(timeout_sec=5.0)
        reply = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(reply, timeout_sec=5.0)
        assert reply.done()
        return reply.result()
    finally:
        client_node.destroy_client(client)


@pytest.mark.parametrize('action', ['pick', 'place'])
@pytest.mark.parametrize('outcome', ['success', 'failure', 'cancel'])
def test_manual_pick_place_execution(manual_node, action, outcome):
    """Execute configured poses, reject overlap and release after completion."""
    node, client_node, executor, robot, gripper = manual_node
    movements = []

    def move_to(pose, motion_type=1):
        pending = Future()
        movements.append((pose, motion_type, pending))
        robot.is_busy = True
        return pending

    robot.move_to.side_effect = move_to
    assert not node._homing_confirmed
    assert movements == []
    assert call_trigger(client_node, executor, '/confirm_homing').success
    assert call_trigger(client_node, executor, f'/manual/{action}').success

    # Acceptance must arrive while the first movement is still pending.
    assert len(movements) == 1
    assert not movements[0][2].done()
    assert node._workflow_running
    assert not call_trigger(
        client_node, executor, '/start_depalletizing',
    ).success
    other = 'place' if action == 'pick' else 'pick'
    assert not call_trigger(
        client_node, executor, f'/manual/{other}',
    ).success
    assert len(movements) == 1

    if outcome == 'cancel':
        assert call_trigger(
            client_node, executor, '/cancel_depalletizing',
        ).success
        robot.cancel_current_goal.assert_called_once()
        assert node._workflow_running
        assert not movements[0][2].done()

    completed = 0
    deadline = time.monotonic() + 5.0
    while node._workflow_running and time.monotonic() < deadline:
        if completed < len(movements):
            pending = movements[completed][2]
            robot.is_busy = False
            # Also cover a late successful result after cancellation: no next step.
            pending.set_result(OperationResult(
                outcome != 'failure', 'Fake movement ended',
                'MOVE_FAILED' if outcome == 'failure' else None,
            ))
            completed += 1
        executor.spin_once(timeout_sec=0.01)
    assert not node._workflow_running

    x, y = (200.0, 0.0) if action == 'pick' else (250.0, 100.0)
    if outcome == 'success':
        assert [(p.x, p.y, p.z, p.r, motion) for p, motion, _ in movements] == [
            (x, y, 115.0, 0.0, 1),
            (x, y, 50.0, 0.0, 2),
            (x, y, 115.0, 0.0, 2),
        ]
        if action == 'pick':
            gripper.activate_tool.assert_called_once_with()
            gripper.deactivate_tool.assert_not_called()
            assert node.pick_place.holding_object
        else:
            gripper.deactivate_tool.assert_called_once_with()
            gripper.activate_tool.assert_not_called()
            assert not node.pick_place.holding_object
    else:
        assert len(movements) == 1
        gripper.activate_tool.assert_not_called()
        gripper.deactivate_tool.assert_not_called()
    if outcome != 'cancel':
        robot.cancel_current_goal.assert_not_called()


@pytest.mark.parametrize('action', ['pick', 'place', 'open_gripper', 'close_gripper'])
@pytest.mark.parametrize(
    'condition, message',
    [
        ('homing', 'Homing'),
        ('fault', 'faulted'),
        ('busy', 'busy'),
        ('server', 'Action Server'),
        ('gripper', 'Gripper'),
        ('workflow', 'already running'),
        ('cancel', 'already running'),
    ],
)
def test_manual_operation_preconditions(manual_node, action, condition, message):
    """Apply the shared start guards before any manual hardware command."""
    node, client_node, executor, robot, gripper = manual_node
    assert call_trigger(client_node, executor, '/confirm_homing').success
    node._homing_confirmed = condition != 'homing'
    node._workflow_running = condition == 'workflow'
    node._cancel_dispatching = condition == 'cancel'
    robot.is_faulted = condition == 'fault'
    robot.is_busy = condition == 'busy'
    robot.server_is_ready.return_value = condition != 'server'
    gripper.service_is_ready.return_value = condition != 'gripper'

    reply = call_trigger(client_node, executor, f'/manual/{action}')
    assert not reply.success
    assert message in reply.message
    assert node._workflow_running == (condition == 'workflow')
    assert node._cancel_dispatching == (condition == 'cancel')
    robot.move_to.assert_not_called()
    gripper.activate_tool.assert_not_called()
    gripper.deactivate_tool.assert_not_called()


@pytest.mark.parametrize('closed', [False, True], ids=['open', 'close'])
@pytest.mark.parametrize(
    'outcome', ['success', 'failure', 'timeout', 'exception', 'invalid', 'cancel'],
)
def test_manual_gripper_execution(manual_node, closed, outcome):
    """Wait for gripper completion, preserve state on failure and block overlap."""
    node, client_node, executor, robot, gripper = manual_node
    command = gripper.activate_tool if closed else gripper.deactivate_tool
    other_command = gripper.deactivate_tool if closed else gripper.activate_tool
    pending = Future()
    command.return_value = pending
    node.pick_place.holding_object = not closed
    assert call_trigger(client_node, executor, '/confirm_homing').success

    action = 'close_gripper' if closed else 'open_gripper'
    assert call_trigger(client_node, executor, f'/manual/{action}').success
    command.assert_called_once_with()
    other_command.assert_not_called()
    assert node._workflow_running
    assert not pending.done()
    assert node.pick_place.holding_object == (not closed)

    # All manual actions and the full workflow share the same execution slot.
    for service in (
        '/manual/pick', '/manual/place', '/manual/open_gripper',
        '/manual/close_gripper', '/start_depalletizing',
    ):
        assert not call_trigger(client_node, executor, service).success
    command.assert_called_once_with()
    other_command.assert_not_called()
    robot.move_to.assert_not_called()

    if outcome == 'cancel':
        assert call_trigger(
            client_node, executor, '/cancel_depalletizing',
        ).success
        assert node._cancel_requested.is_set()
        assert node._workflow_running
        assert not pending.done()
        # A sent service command cannot be undone by cancelling robot motion.
        # Its eventual successful result must still update the holding state.

    if outcome == 'exception':
        pending.set_exception(RuntimeError('gripper connection lost'))
    elif outcome == 'invalid':
        pending.set_result(None)
    else:
        succeeded = outcome in ('success', 'cancel')
        code = None if succeeded else (
            'TIMEOUT' if outcome == 'timeout' else 'GRIPPER_FAILED'
        )
        pending.set_result(OperationResult(succeeded, 'Fake gripper reply', code))

    deadline = time.monotonic() + 5.0
    while node._workflow_running and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert not node._workflow_running
    expected_holding = closed if outcome in ('success', 'cancel') else not closed
    assert node.pick_place.holding_object == expected_holding
    command.assert_called_once_with()
    other_command.assert_not_called()
    robot.move_to.assert_not_called()

    # Completion frees the slot; a later command can be explicitly accepted.
    command.return_value = OperationResult(True, 'Explicit retry')
    assert call_trigger(client_node, executor, f'/manual/{action}').success
    deadline = time.monotonic() + 5.0
    while node._workflow_running and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert not node._workflow_running
    assert node.pick_place.holding_object == closed
    assert not node._cancel_requested.is_set()
    assert command.call_count == 2


@pytest.mark.parametrize('closed', [False, True], ids=['open', 'close'])
def test_manual_gripper_accepts_synchronous_fakes(manual_node, closed):
    """Keep direct OperationResult support for existing gripper fakes."""
    node, client_node, executor, robot, gripper = manual_node
    node.pick_place.holding_object = not closed
    assert call_trigger(client_node, executor, '/confirm_homing').success
    action = 'close_gripper' if closed else 'open_gripper'
    assert call_trigger(client_node, executor, f'/manual/{action}').success
    deadline = time.monotonic() + 5.0
    while node._workflow_running and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert not node._workflow_running
    assert node.pick_place.holding_object == closed
    command = gripper.activate_tool if closed else gripper.deactivate_tool
    command.assert_called_once_with()
    robot.move_to.assert_not_called()


@pytest.mark.parametrize(
    'values',
    [
        [200.0, 0.0, 150.0],
        [float('nan'), 0.0, 150.0, 0.0],
        [200.0, 0.0, float('inf'), 0.0],
        [999.0, 0.0, 150.0, 0.0],
        [200.0, 0.0, 999.0, 0.0],
        [True, False, True, False],
        ['x', 'y', 'z', 'r'],
        'not a list',
        1.0,
    ],
)
def test_invalid_transport_prevents_driver_creation(values):
    """Reject an invalid complete route before any hardware client exists."""
    parameters = make_test_parameters()
    parameters.append(Parameter('transport_waypoints', value=values))
    rclpy.init()
    try:
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot',
        ) as robot_class, patch(
            'depalletizing_pick_place.main_node.GripperClient',
        ) as gripper_class:
            with pytest.raises(RuntimeError, match='transport_waypoints'):
                DepalletizingMainNode(parameter_overrides=parameters)
            robot_class.assert_not_called()
            gripper_class.assert_not_called()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('outcome', ['success', 'failure', 'timeout', 'cancel'])
@pytest.mark.parametrize(
    'manual_node',
    [{'transport_waypoints': [
        210.0, 20.0, 150.0, 0.0,
        240.0, 80.0, 150.0, 15.0,
    ]}],
    indirect=True,
)
def test_full_workflow_transport(manual_node, outcome):
    """Insert sequential transport between pick and place and stop on errors."""
    node, client_node, executor, robot, gripper = manual_node
    movements = []
    events = []

    def move_to(pose, motion_type=1):
        pending = Future()
        movements.append((pose, motion_type, pending))
        events.append('move')
        robot.is_busy = True
        return pending

    def close():
        events.append('close')
        return OperationResult(True, 'Closed')

    def open_gripper():
        events.append('open')
        return OperationResult(True, 'Opened')

    robot.move_to.side_effect = move_to
    gripper.activate_tool.side_effect = close
    gripper.deactivate_tool.side_effect = open_gripper
    assert call_trigger(client_node, executor, '/confirm_homing').success
    assert call_trigger(client_node, executor, '/start_depalletizing').success
    assert len(movements) == 1
    assert not movements[0][2].done()

    completed = 0
    deadline = time.monotonic() + 5.0
    while node._workflow_running and time.monotonic() < deadline:
        if completed < len(movements):
            # First three moves are pick; index 3 is the first transport move.
            transporting = completed == 3
            if transporting:
                assert node.workflow.status == 'transporting'
                assert node.pick_place.holding_object
                assert gripper.activate_tool.call_count == 1
                gripper.deactivate_tool.assert_not_called()
                if outcome == 'cancel':
                    assert call_trigger(
                        client_node, executor, '/cancel_depalletizing',
                    ).success
                    robot.cancel_current_goal.assert_called_once()
                    assert node._workflow_running

            success = not (transporting and outcome in ('failure', 'timeout'))
            code = None if success else (
                'TIMEOUT' if outcome == 'timeout' else 'MOVE_FAILED'
            )
            robot.is_busy = False
            # A late successful reply must not continue transport after cancel.
            movements[completed][2].set_result(
                OperationResult(success, 'Fake movement reply', code)
            )
            completed += 1
        executor.spin_once(timeout_sec=0.01)

    assert not node._workflow_running
    assert completed == len(movements)
    assert [(p.x, p.y, p.z, p.r, motion) for p, motion, _ in movements[:4]] == [
        (200.0, 0.0, 115.0, 0.0, 1),
        (200.0, 0.0, 50.0, 0.0, 2),
        (200.0, 0.0, 115.0, 0.0, 2),
        (210.0, 20.0, 150.0, 0.0, 2),
    ]
    if outcome == 'success':
        assert node.workflow.status == 'completed'
        assert [(p.x, p.y, p.z, p.r, motion) for p, motion, _ in movements[4:]] == [
            (240.0, 80.0, 150.0, 15.0, 2),
            (250.0, 100.0, 115.0, 0.0, 1),
            (250.0, 100.0, 50.0, 0.0, 2),
            (250.0, 100.0, 115.0, 0.0, 2),
        ]
        assert events == [
            'move', 'move', 'close', 'move', 'move',
            'move', 'move', 'move', 'open', 'move',
        ]
        assert not node.pick_place.holding_object
    else:
        assert len(movements) == 4
        assert node.workflow.status == (
            'cancelled' if outcome == 'cancel' else 'failed'
        )
        gripper.deactivate_tool.assert_not_called()
        assert node.pick_place.holding_object


@pytest.mark.parametrize('manual_node', [{'transport_waypoints': []}], indirect=True)
def test_empty_transport_is_optional(manual_node):
    """An explicit empty ROS array produces no extra transport movements."""
    node, _, _, _, _ = manual_node
    assert node._read_transport_waypoints() == ()


def test_transport_revalidated_before_start(manual_node):
    """Reject invalid route updates before even the first pick movement."""
    node, client_node, executor, robot, gripper = manual_node
    result = node.set_parameters([
        Parameter('transport_waypoints', value=[999.0, 0.0, 150.0, 0.0]),
    ])
    assert result[0].successful
    assert call_trigger(client_node, executor, '/confirm_homing').success
    reply = call_trigger(client_node, executor, '/start_depalletizing')
    assert not reply.success
    assert 'transport_waypoints' in reply.message
    assert not node._workflow_running
    robot.move_to.assert_not_called()
    gripper.activate_tool.assert_not_called()
