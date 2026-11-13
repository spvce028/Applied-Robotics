"""Exercise the real ROS start service with mocked hardware."""

import time
from unittest.mock import patch

import pytest

pytest.importorskip('rclpy.node')
pytest.importorskip('dobot_msgs.action')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_srvs.srv import Trigger

from depalletizing_interfaces.models import OperationResult
from depalletizing_pick_place.main_node import DepalletizingMainNode


@pytest.mark.parametrize('fail_pick', [False, True])
def test_start_service_runs_async_workflow(fail_pick):
    """Drive the service, P5 workflow and P4 primitives through real ROS."""
    class Robot:
        is_faulted = False

        def __init__(self, node):
            self.movements = []
            self.pending = None

        def server_is_ready(self):
            return True

        def move_to(self, pose):
            self.movements.append(pose)
            self.pending = Future()
            return self.pending

    rclpy.init()
    executor = SingleThreadedExecutor()
    node = None
    client_node = None
    try:
        with patch(
            'depalletizing_pick_place.main_node.DobotActionRobot', Robot,
        ):
            node = DepalletizingMainNode()
        client_node = rclpy.create_node('p5_workflow_test_client')
        executor.add_node(node)
        executor.add_node(client_node)
        client = client_node.create_client(Trigger, '/start_depalletizing')
        assert client.wait_for_service(timeout_sec=5.0)
        response = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(response, timeout_sec=5.0)
        assert response.done()
        assert response.result().success
        assert node.workflow.status == 'picking'
        assert node._workflow_running

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
