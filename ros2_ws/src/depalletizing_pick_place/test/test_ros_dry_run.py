"""Hardware-free ROS dry run using the official P1 server and real clients.

The gripper endpoint below is a TEST-ONLY stand-in until P3 supplies its official
fake service. This suite does not claim final P1/P3 simulation acceptance.
"""

import threading
import time
import uuid
from pathlib import Path
from unittest.mock import patch

import pytest

pytest.importorskip('rclpy.node')
pytest.importorskip('dobot_msgs.action')
yaml = pytest.importorskip('yaml')

import rclpy
from dobot_msgs.srv import GripperControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from depalletizing_pick_place.fake_ptp_action_server import FakePTPActionServer
from depalletizing_pick_place.main_node import DepalletizingMainNode


def wait_for(predicate, timeout=10.0):
    """Bound a wait while the background executor processes real ROS traffic."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(0.01)
    raise AssertionError('Timed out waiting for ROS integration condition')


@pytest.fixture
def ros_dry_run():
    """Run actual P5/P4/P1/P3 clients against isolated simulated endpoints."""
    prefix = '/p5_dry_run_' + uuid.uuid4().hex
    service_names = [
        '/PTP_action', '/start_depalletizing', '/confirm_homing',
        '/cancel_depalletizing', '/reset_fault', '/manual/pick', '/manual/place',
        '/manual/open_gripper', '/manual/close_gripper',
    ]
    args = ['--ros-args']
    for name in service_names:
        args.extend(['-r', f'{name}:={prefix}{name}'])
    # Remapping keeps the tests away from hardware endpoints on the ROS network.
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=6)
    nodes = []
    thread = None
    requests = []
    try:
        server = FakePTPActionServer(
            movement_duration=0.04, feedback_interval=0.005,
        )
        nodes.append(server)
        gripper_node = rclpy.create_node('p5_test_only_gripper')
        nodes.append(gripper_node)

        def gripper_reply(request, response):
            requests.append((
                request.gripper_state, request.keep_compressor_running,
            ))
            response.success = True
            response.message = 'Test-only gripper command completed'
            return response

        gripper_name = prefix + '/gripper'
        gripper_node.create_service(GripperControl, gripper_name, gripper_reply)
        config_file = Path(__file__).resolve().parents[1] / 'config/depalletizing.yaml'
        config = yaml.safe_load(config_file.read_text(encoding='utf-8'))
        values = config['depalletizing_main_node']['ros__parameters']
        values['gripper_service_name'] = gripper_name
        values['transport_waypoints'] = [
            210.0, 20.0, 150.0, 0.0, 240.0, 80.0, 150.0, 0.0,
        ]
        node = DepalletizingMainNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in values.items()
        ])
        nodes.append(node)
        caller = rclpy.create_node('p5_dry_run_caller')
        nodes.append(caller)
        for item in nodes:
            executor.add_node(item)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        wait_for(lambda: node.robot.server_is_ready() and node.gripper.service_is_ready())

        def call(name):
            client = caller.create_client(Trigger, prefix + name)
            try:
                assert client.wait_for_service(timeout_sec=5.0)
                reply = client.call_async(Trigger.Request())
                wait_for(reply.done)
                return reply.result()
            finally:
                caller.destroy_client(client)

        yield node, server, requests, call, values
    finally:
        # This test runs spin() on another thread: never spin the executor here.
        if len(nodes) >= 3:
            try:
                nodes[2].handle_cancel_service(Trigger.Request(), Trigger.Response())
                wait_for(
                    lambda: not nodes[2]._workflow_running and not nodes[2].robot.is_busy,
                    timeout=2.0,
                )
            except Exception:
                pass
        executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=2.0)
        for item in reversed(nodes):
            item.destroy_node()
        rclpy.try_shutdown()


def test_dry_run_refuses_start_without_homing(ros_dry_run):
    """Start the real node without commanding motion and reject an unhomed run."""
    node, server, requests, call, _ = ros_dry_run
    before = server.current_pose
    assert not node._homing_confirmed
    assert not node.robot.is_busy
    assert not node._workflow_running
    reply = call('/start_depalletizing')
    assert not reply.success
    assert 'Homing' in reply.message
    assert server.current_pose == before
    assert requests == []


def test_two_complete_runs_with_real_clients(ros_dry_run):
    """Exercise actual action/service messages, transport and repeatability."""
    node, server, requests, call, values = ros_dry_run
    assert call('/confirm_homing').success
    feedback_samples = []
    real_feedback_callback = node.robot._feedback_callback

    def record_feedback(*args, **kwargs):
        # Observe actual received feedback AFTER the real driver processes it.
        # last_feedback belongs to the active goal and is cleared at completion.
        real_feedback_callback(*args, **kwargs)
        pose = node.robot.last_feedback
        if pose is not None:
            feedback_samples.append(tuple(pose))

    with patch.object(node.robot, '_feedback_callback', side_effect=record_feedback):
        for run in range(2):
            samples_before = len(feedback_samples)
            assert call('/start_depalletizing').success
            wait_for(lambda: not node._workflow_running)
            assert node.workflow.status == 'completed'
            assert not node.robot.is_busy
            assert not node.pick_place.holding_object
            assert requests == [
                ('close', values['keep_compressor_running']),
                ('open', values['keep_compressor_running']),
            ] * (run + 1)
            assert server.current_pose == pytest.approx([
                values['place_x'], values['place_y'],
                values['place_z'] + values['approach_height'], values['place_r'],
            ])
            assert len(feedback_samples) > samples_before
            assert node.robot.last_feedback is None
