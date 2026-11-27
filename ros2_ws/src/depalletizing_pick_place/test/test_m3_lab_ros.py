"""Exercise the interactive M3 runner over real ROS communication with fakes."""

import copy
import uuid

import pytest

pytest.importorskip('rclpy.node')
from test_ros_dry_run import ros_dry_run  # noqa: F401
from depalletizing_pick_place.m3_lab_test import M3LabDriver, record_summary, run_trials


@pytest.fixture
def isolated_dry_run(monkeypatch, request):
    """Separate logs and parameter services from an operator's running simulation."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + uuid.uuid4().int % 70))
    return request.getfixturevalue('ros_dry_run')


def test_m3_runner_preserves_live_route_and_completes_two_cycles(isolated_dry_run):
    node, server, gripper, _, values = isolated_dry_run
    # The reusable fixture's generic route is replaced explicitly by the test
    # setup, never by the operator tool under test.
    from rclpy.parameter import Parameter
    route = [
        200.0, 0.0, 150.0, 15.0,
        200.0, 0.0, 150.0, -60.0,
        250.0, 100.0, 150.0, -60.0,
    ]
    node.set_parameters([
        Parameter('pick_r', value=15.0), Parameter('place_r', value=-60.0),
        Parameter('transport_waypoints', value=route),
    ])
    runner = M3LabDriver()
    executor = node.executor
    executor.add_node(runner)
    report = {'mode': 'simulation', 'trials': [], 'status': 'initializing'}
    saved = []
    responses = iter(['REVIEWED', 'rectangle', 'container', 'SIMULATED', 'OPEN', 'START', 'START'])
    try:
        before = runner.configuration()
        run_trials(
            runner, report, lambda: saved.append(copy.deepcopy(report)),
            2, 10.0, ask=lambda _: next(responses),
        )
        assert report['status'] == 'requested_trials_completed'
        assert runner.configuration() == before
        assert report['configuration']['transport_waypoints'] == route
        assert record_summary(report)['software_successes'] == 2
        assert record_summary(report)['physical_success_rate'] is None
        assert gripper.requests == [('open', False), ('close', False), ('open', False),
                                    ('close', False), ('open', False)]
        assert not node._workflow_running
        assert not node.robot.is_busy
        assert not node.pick_place.holding_object
        assert len(saved) > 4
    finally:
        executor.remove_node(runner)
        runner.destroy_node()
