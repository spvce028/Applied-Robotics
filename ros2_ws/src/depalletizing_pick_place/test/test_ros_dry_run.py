"""Hardware-free ROS integration using packaged fake servers and real clients."""

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
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from depalletizing_pick_place.fake_ptp_action_server import FakePTPActionServer
from depalletizing_pick_place.fake_gripper_service import FakeGripperService
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
    """Run actual clients and workflow against isolated packaged fake servers."""
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
    command_log = []
    observation_lock = threading.Lock()

    class ObservedPTPServer(FakePTPActionServer):
        """Observe received goals without replacing the packaged server logic."""

        @property
        def observed_commands(self):
            with observation_lock:
                return list(command_log)

        def goal_callback(self, goal_request):
            response = super().goal_callback(goal_request)
            if response == GoalResponse.ACCEPT:
                with observation_lock:
                    command_log.append((
                        'move', tuple(goal_request.target_pose),
                        goal_request.motion_type,
                    ))
            return response

    class ObservedGripperService(FakeGripperService):
        """Observe completed gripper commands using the real fake-service body."""

        def handle_request(self, request, response):
            response = super().handle_request(request, response)
            if response.success:
                with observation_lock:
                    command_log.append(('gripper', request.gripper_state))
            return response

    try:
        server = ObservedPTPServer(
            movement_duration=0.04, feedback_interval=0.005,
        )
        nodes.append(server)
        gripper_name = prefix + '/gripper'
        gripper_node = ObservedGripperService(
            service_name=gripper_name, response_delay=0.01, no_response_delay=0.3,
        )
        nodes.append(gripper_node)
        # Integration scenarios must not depend on measured hardware coordinates.
        config_file = Path(__file__).resolve().parents[1] / 'config/simulation.yaml'
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

        yield node, server, gripper_node, call, values
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
    node, server, gripper, call, _ = ros_dry_run
    before = server.current_pose
    assert not node._homing_confirmed
    assert not node.robot.is_busy
    assert not node._workflow_running
    reply = call('/start_depalletizing')
    assert not reply.success
    assert 'Homing' in reply.message
    assert server.current_pose == before
    assert gripper.requests == []


def test_two_complete_runs_with_real_clients(ros_dry_run):
    """Exercise actual action/service messages, transport and repeatability."""
    node, server, gripper, call, values = ros_dry_run
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
            assert gripper.requests == [
                ('close', values['keep_compressor_running']),
                ('open', values['keep_compressor_running']),
            ] * (run + 1)
            assert server.current_pose == pytest.approx([
                values['place_x'], values['place_y'],
                values['place_z'] + values['approach_height'], values['place_r'],
            ])
            assert len(feedback_samples) > samples_before
            assert node.robot.last_feedback is None
            assert not gripper.is_closed


def test_real_action_cancellation_stops_workflow(ros_dry_run):
    """Process cancel concurrently with the packaged fake server's execute loop."""
    node, server, gripper, call, _ = ros_dry_run
    server.movement_duration = 0.5
    assert call('/confirm_homing').success
    assert call('/start_depalletizing').success
    wait_for(lambda: node.robot.is_busy)
    assert call('/cancel_depalletizing').success
    wait_for(lambda: not node._workflow_running and not node.robot.is_busy)
    assert node.workflow.status == 'cancelled'
    assert gripper.requests == []


def test_official_gripper_timeout_recovery(ros_dry_run):
    """Exercise timeout cleanup, retry and a late reply over real ROS services."""
    node, _, gripper, call, _ = ros_dry_run
    node.gripper.timeout_sec = 0.05
    gripper.set_parameters([Parameter('mode', value='no_response')])
    assert call('/confirm_homing').success
    assert call('/manual/close_gripper').success
    wait_for(lambda: not node._workflow_running)
    assert node.gripper._pending is None
    assert node.gripper._timer is None
    # Keep an observer of the timed-out client's response; it must stay TIMEOUT.
    gripper.set_parameters([Parameter('mode', value='success')])
    assert call('/manual/open_gripper').success
    wait_for(lambda: not node._workflow_running)
    assert node.gripper._pending is None
    assert not node.pick_place.holding_object
    # Allow the first request's delayed response to reach the client.
    threading.Event().wait(0.35)
    assert node.gripper._pending is None
    assert node.gripper._timer is None
    assert not node.pick_place.holding_object
    assert gripper.requests == [('close', False), ('open', False)]


def test_official_gripper_rejection_stops_pick(ros_dry_run):
    """A real fake-service failure stops the workflow before transport/place."""
    node, _, gripper, call, _ = ros_dry_run
    gripper.set_parameters([Parameter('mode', value='fail')])
    assert call('/confirm_homing').success
    assert call('/start_depalletizing').success
    wait_for(lambda: not node._workflow_running)
    assert node.workflow.status == 'failed'
    assert gripper.requests == [('close', False)]
    assert not node.pick_place.holding_object
    assert node.gripper._pending is None
    assert node.gripper._timer is None


def test_official_gripper_delayed_reply_within_timeout(ros_dry_run):
    """A delayed reply inside the timeout completes the command normaly."""
    node, _, gripper, _, _ = ros_dry_run
    client = node.gripper
    client.timeout_sec = 1.0
    gripper.set_parameters([Parameter('response_delay', value=0.2)])
    start = time.monotonic()
    future = client.activate_tool()
    wait_for(future.done)
    assert future.result().success
    assert time.monotonic() - start >= 0.2
    assert gripper.is_closed
    assert client._pending is None
    assert client._timer is None
    assert gripper.requests == [('close', False)]


def test_official_gripper_late_reply_cannot_finish_newer_request(ros_dry_run):
    """A late success for a timed-out command must not complete a pending one."""
    node, _, gripper, _, _ = ros_dry_run
    client = node.gripper
    client.timeout_sec = 0.05
    gripper.set_parameters([Parameter('mode', value='no_response')])
    old = client.activate_tool()
    wait_for(old.done)
    assert old.result().error_code == 'TIMEOUT'

    gripper.set_parameters([
        Parameter('mode', value='fail'), Parameter('response_delay', value=0.6),
    ])
    client.timeout_sec = 2.0
    new = client.deactivate_tool()
    wait_for(lambda: gripper.is_closed)
    threading.Event().wait(0.05)
    assert not new.done()

    wait_for(new.done)
    assert new.result().error_code == 'GRIPPER_FAILED'
    assert old.result().error_code == 'TIMEOUT'
    assert client._pending is None
    assert client._timer is None
    assert gripper.requests == [('close', False), ('open', False)]


def configure_oriented_placement(node):
    """Use explicit simulation poses and rotate at a separate raised waypoint."""
    values = {
        'pick_x': 200.0, 'pick_y': 0.0, 'pick_z': 50.0, 'pick_r': 15.0,
        'place_x': 250.0, 'place_y': 100.0, 'place_z': 50.0, 'place_r': -60.0,
        'transport_waypoints': [
            200.0, 0.0, 150.0, 15.0,    # Raise before changing orientation.
            200.0, 0.0, 150.0, -60.0,   # Align at fixed x/y/z.
            250.0, 100.0, 150.0, -60.0, # Transport already aligned.
        ],
    }
    results = node.set_parameters([
        Parameter(name, value=value) for name, value in values.items()
    ])
    assert all(result.successful for result in results)
    return [
        ('move', (200.0, 0.0, 100.0, 15.0), 1),
        ('move', (200.0, 0.0, 50.0, 15.0), 2),
        ('gripper', 'close'),
        ('move', (200.0, 0.0, 100.0, 15.0), 2),
        ('move', (200.0, 0.0, 150.0, 15.0), 2),
        ('move', (200.0, 0.0, 150.0, -60.0), 2),
        ('move', (250.0, 100.0, 150.0, -60.0), 2),
        ('move', (250.0, 100.0, 100.0, -60.0), 1),
        ('move', (250.0, 100.0, 50.0, -60.0), 2),
        ('gripper', 'open'),
        ('move', (250.0, 100.0, 100.0, -60.0), 2),
    ]


def test_m3_oriented_placement_two_complete_cycles(ros_dry_run):
    """Verify actual server-received commands, angles and release order twice."""
    node, server, gripper, call, values = ros_dry_run
    expected = configure_oriented_placement(node)
    assert server.observed_commands == []
    assert call('/confirm_homing').success
    for cycle in range(2):
        assert call('/start_depalletizing').success
        # The operation is still active when acknowledgement reaches the caller.
        assert node._workflow_running
        for service in (
            '/start_depalletizing', '/manual/pick', '/manual/place',
            '/manual/open_gripper', '/manual/close_gripper',
        ):
            assert not call(service).success
        wait_for(lambda: not node._workflow_running and not node.robot.is_busy)
        assert node.workflow.status == 'completed'
        assert server.observed_commands == expected * (cycle + 1)
        assert server.current_pose == pytest.approx([250.0, 100.0, 100.0, -60.0])
        assert gripper.requests == [
            ('close', values['keep_compressor_running']),
            ('open', values['keep_compressor_running']),
        ] * (cycle + 1)
        assert not gripper.is_closed
        assert not node.pick_place.holding_object


def test_m3_cancel_during_alignment_prevents_placement(ros_dry_run):
    """Cancel the rotation action itself, leaving later transport/place unstarted."""
    node, server, gripper, call, values = ros_dry_run
    expected = configure_oriented_placement(node)
    # Allow time to issue a real cancel while the alignment action is active.
    server.movement_duration = 0.3
    assert call('/confirm_homing').success
    assert call('/start_depalletizing').success
    alignment = ('move', (200.0, 0.0, 150.0, -60.0), 2)
    wait_for(lambda: alignment in server.observed_commands)
    assert node.workflow.status == 'transporting'
    assert node.robot.is_busy
    assert node.pick_place.holding_object
    assert gripper.is_closed
    assert call('/cancel_depalletizing').success
    # Wait for terminal action handling, not merely cancellation acknowledgement.
    wait_for(lambda: not node._workflow_running and not node.robot.is_busy)
    assert node.workflow.status == 'cancelled'
    assert server.observed_commands == expected[:6]
    assert gripper.requests == [('close', values['keep_compressor_running'])]
    assert gripper.is_closed
    assert node.pick_place.holding_object
    assert node._cancel_requested.is_set()
