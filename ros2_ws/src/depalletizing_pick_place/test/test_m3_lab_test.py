"""Test the M3 operator script without commanding physical hardware."""

import copy
import json
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

pytest.importorskip('rclpy.node')
from depalletizing_pick_place.m3_lab_test import record_summary, run_trials, validated_plan


@pytest.fixture
def configuration():
    path = Path(__file__).resolve().parents[1] / 'config/simulation.yaml'
    return yaml.safe_load(path.read_text())['depalletizing_main_node']['ros__parameters']


def session(mode='hardware'):
    """Create an empty report suitable for the runner."""
    return {'mode': mode, 'trials': [], 'status': 'initializing'}


def fake_driver(configuration):
    """Provide explicit accepted/completed outcomes, with no ROS transport."""
    node = Mock()
    node.configuration.side_effect = lambda: copy.deepcopy(configuration)
    node.call.return_value = (True, 'Confirmed')
    node.execute.side_effect = lambda service, timeout: {
        'service': service, 'accepted': True,
        'status': 'completed', 'software_pass': True,
    }
    return node


def answers(mode, cycles=1, observation='y'):
    """Return operator confirmations for deterministic test sessions."""
    items = ['REVIEWED', 'rectangle', 'rectangular opening',
             'SIMULATED' if mode == 'simulation' else 'HOMED', 'OPEN']
    for _ in range(cycles):
        items.extend(['START'])
        if mode == 'hardware':
            items.extend([observation] * 6 + ['test note'])
    iterator = iter(items)
    return lambda prompt: next(iterator)


def test_m3_plan_preserves_configured_route(configuration):
    original = copy.deepcopy(configuration)
    plan = validated_plan(configuration)
    assert configuration == original
    transport = [pose for name, pose in plan if name.startswith('MOVL transport')]
    assert [value for pose in transport for value in pose] == configuration['transport_waypoints']
    assert plan[-3][1][3] == configuration['place_r']


@pytest.mark.parametrize('change', ['empty', 'same_angle', 'nan', 'bounds', 'no_alignment'])
def test_invalid_m3_route_is_rejected(configuration, change):
    if change == 'empty':
        configuration['transport_waypoints'] = []
    elif change == 'same_angle':
        configuration['place_r'] = configuration['pick_r']
    elif change == 'nan':
        configuration['pick_x'] = float('nan')
    elif change == 'bounds':
        configuration['workspace_z_max'] = 60.0
    else:
        configuration['transport_waypoints'] = [250.0, 100.0, 150.0, -60.0]
    node = fake_driver(configuration)
    with pytest.raises(ValueError):
        run_trials(node, session(), Mock(), 1, 10.0, ask=Mock())
    node.call.assert_not_called()
    node.execute.assert_not_called()


def test_hardware_trials_require_observations_and_do_not_change_parameters(configuration):
    node = fake_driver(configuration)
    report = session()
    snapshots = []
    run_trials(node, report, lambda: snapshots.append(json.dumps(report)), 2, 10.0,
               ask=answers('hardware', 2))
    assert report['status'] == 'requested_trials_completed'
    assert record_summary(report)['physical_success_rate'] == 1.0
    assert record_summary(report)['attempts'] == 2
    assert len(snapshots) > 2
    assert [call.args[0] for call in node.execute.call_args_list] == [
        '/manual/open_gripper', '/start_depalletizing', '/start_depalletizing',
    ]
    node.set_waypoints.assert_not_called()
    assert report['configuration'] == configuration


def test_simulation_never_reports_physical_success(configuration):
    node = fake_driver(configuration)
    report = session('simulation')
    run_trials(node, report, Mock(), 1, 10.0, ask=answers('simulation'))
    assert report['trials'][0]['physical_pass'] is None
    assert record_summary(report)['physical_successes'] is None
    assert record_summary(report)['physical_success_rate'] is None
    assert record_summary(report)['software_successes'] == 1


def test_unknown_observation_stops_series(configuration):
    report = session()
    node = fake_driver(configuration)
    run_trials(node, report, Mock(), 2, 10.0, ask=answers('hardware', observation='u'))
    assert report['status'] == 'stopped_after_observation_failure'
    assert report['trials'][0]['physical_pass'] is False
    assert record_summary(report)['physical_success_rate'] == 0.0
    assert node.execute.call_count == 2


def test_missing_review_never_sends_command(configuration):
    node = fake_driver(configuration)
    run_trials(node, session(), Mock(), 1, 10.0, ask=lambda _: '')
    node.call.assert_not_called()
    node.execute.assert_not_called()


def test_operation_timeout_stops_without_automatic_retry(configuration):
    node = fake_driver(configuration)
    node.execute.side_effect = [
        {'accepted': True, 'software_pass': True, 'status': 'completed'},
        {'accepted': True, 'software_pass': False, 'status': 'timeout'},
    ]
    report = session('simulation')
    run_trials(node, report, Mock(), 2, 10.0, ask=answers('simulation'))
    assert report['status'] == 'stopped_after_failure'
    assert node.execute.call_count == 2
    assert record_summary(report)['attempts'] == 1


def test_changed_configuration_prevents_next_start(configuration):
    node = fake_driver(configuration)
    changed = dict(configuration, place_r=-55.0)
    node.configuration.side_effect = [copy.deepcopy(configuration), changed]
    report = session('simulation')
    with pytest.raises(RuntimeError, match='Configuration changed'):
        run_trials(node, report, Mock(), 1, 10.0, ask=answers('simulation'))
    assert node.execute.call_count == 1  # Only explicitly confirmed preparation.


def test_unconfirmed_request_is_not_counted_as_success():
    report = session()
    report['trials'].append({'status': 'requesting', 'physical_pass': None})
    assert record_summary(report)['unconfirmed_requests'] == 1
    assert record_summary(report)['physical_success_rate'] is None


def test_cli_writes_json_and_never_overwrites_existing_report(configuration, tmp_path, monkeypatch):
    from depalletizing_pick_place import m3_lab_test as module
    node = fake_driver(configuration)
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'try_shutdown', Mock())
    monkeypatch.setattr(module, 'M3LabDriver', lambda: node)
    executor = Mock()
    monkeypatch.setattr(module, 'SingleThreadedExecutor', lambda: executor)
    original_runner = module.run_trials
    monkeypatch.setattr(module, 'run_trials', lambda n, r, s, c, t: original_runner(
        n, r, s, c, t, ask=answers('simulation'),
    ))
    output = tmp_path / 'm3.json'
    module.main(['--simulation', '--cycles', '1', '--revision', 'tested-commit',
                 '--output', str(output)])
    report = json.loads(output.read_text())
    assert report['revision'] == 'tested-commit'
    assert report['status'] == 'requested_trials_completed'
    assert report['summary']['physical_success_rate'] is None
    assert report['summary']['software_successes'] == 1
    before = output.read_bytes()
    with pytest.raises(FileExistsError):
        module.main(['--output', str(output)])
    assert output.read_bytes() == before
    node.destroy_node.assert_called_once()


def test_cli_interrupt_requests_cancel_and_saves_uncertain_attempt(configuration, tmp_path, monkeypatch):
    from depalletizing_pick_place import m3_lab_test as module
    node = fake_driver(configuration)
    node._log_event.wait.return_value = False
    monkeypatch.setattr(module.rclpy, 'init', Mock())
    monkeypatch.setattr(module.rclpy, 'try_shutdown', Mock())
    monkeypatch.setattr(module, 'M3LabDriver', lambda: node)
    monkeypatch.setattr(module, 'SingleThreadedExecutor', Mock())

    def interrupt(n, report, save, cycles, timeout):
        report['trials'].append({'status': 'requesting', 'physical_pass': None})
        save()
        raise KeyboardInterrupt()

    monkeypatch.setattr(module, 'run_trials', interrupt)
    output = tmp_path / 'interrupted.json'
    module.main(['--simulation', '--output', str(output)])
    report = json.loads(output.read_text())
    assert report['status'] == 'interrupted'
    assert report['summary']['unconfirmed_requests'] == 1
    assert report['summary']['physical_success_rate'] is None
    node.call.assert_called_once_with('/cancel_depalletizing', timeout=5.0)
    node.destroy_node.assert_called_once()
    node.set_waypoints.assert_not_called()
