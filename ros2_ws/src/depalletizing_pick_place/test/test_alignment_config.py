"""Check the M3 example route without requiring ROS or robot hardware."""

import math
from pathlib import Path

import pytest
import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / 'config' / 'm3_alignment_simulation.yaml'
)


@pytest.fixture
def parameters():
    """Load the standalone example configuration."""
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding='utf-8'))
    assert set(config) == {'depalletizing_main_node'}
    return config['depalletizing_main_node']['ros__parameters']


@pytest.fixture
def route(parameters):
    """Decode the existing flat waypoint format for assertions."""
    values = parameters['transport_waypoints']
    assert isinstance(values, list)
    assert len(values) == 20
    assert all(type(value) in (int, float) for value in values)
    assert all(math.isfinite(value) for value in values)
    return [
        tuple(values[index:index + 4])
        for index in range(0, len(values), 4)
    ]


def test_example_contains_required_node_parameters(parameters):
    """The example supplies all current required scalar parameters."""
    for prefix in ('pick', 'place'):
        for axis in ('x', 'y', 'z', 'r'):
            value = parameters[f'{prefix}_{axis}']
            assert type(value) is float
            assert math.isfinite(value)

    for name in ('movement_timeout', 'gripper_timeout', 'approach_height'):
        assert type(parameters[name]) is float
        assert math.isfinite(parameters[name])
        assert parameters[name] > 0

    for name in ('velocity_ratio', 'acceleration_ratio'):
        assert type(parameters[name]) is float
        assert 0 < parameters[name] <= 1

    assert isinstance(parameters['gripper_service_name'], str)
    assert parameters['gripper_service_name'].strip()
    assert type(parameters['keep_compressor_running']) is bool


def test_example_poses_fit_configured_bounds(parameters, route):
    """Numeric bounds cover transport, pick/place and derived approaches."""
    poses = list(route)
    for prefix in ('pick', 'place'):
        pose = tuple(parameters[f'{prefix}_{axis}'] for axis in 'xyzr')
        poses.append(pose)
        poses.append((
            pose[0],
            pose[1],
            pose[2] + parameters['approach_height'],
            pose[3],
        ))

    for axis_index, axis in enumerate('xyzr'):
        lower = parameters[f'workspace_{axis}_min']
        upper = parameters[f'workspace_{axis}_max']
        assert math.isfinite(lower) and math.isfinite(upper)
        assert lower <= upper
        for pose in poses:
            assert math.isfinite(pose[axis_index])
            assert lower <= pose[axis_index] <= upper


def test_example_raises_before_transport(parameters, route):
    """Retain pick orientation while raising and arriving at alignment."""
    first, arrival = route[:2]
    assert first[:2] == (parameters['pick_x'], parameters['pick_y'])
    assert first[3] == arrival[3] == parameters['pick_r']
    assert all(pose[2] == first[2] for pose in route[:4])

    for prefix in ('pick', 'place'):
        assert first[2] > (
            parameters[f'{prefix}_z'] + parameters['approach_height']
        )


def test_example_rotates_at_fixed_xyz(parameters, route):
    """Rotation has a separate goal at the already reached XYZ position."""
    arrival, alignment = route[1:3]
    assert arrival[:3] == alignment[:3]
    assert arrival[3] == parameters['pick_r']
    assert alignment[3] == parameters['place_r']
    assert arrival[3] != alignment[3]
    assert all(pose[3] == parameters['place_r'] for pose in route[2:])


def test_example_finishes_at_placement_approach(parameters, route):
    """Reach the place approach linearly before the existing place primitive."""
    above, approach = route[-2:]
    assert above[:2] == approach[:2]
    assert above[2] > approach[2]
    assert approach == (
        parameters['place_x'],
        parameters['place_y'],
        parameters['place_z'] + parameters['approach_height'],
        parameters['place_r'],
    )