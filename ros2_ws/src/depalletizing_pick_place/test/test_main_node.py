"""Exercise the real ROS start service with mocked hardware."""

import time
from unittest.mock import patch

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