"""Unit tests for the pure helpers of the M2 lab tool."""

import pytest

from depalletizing_pick_place.m2_lab_test import transport_via, update_yaml_text


def test_transport_via_is_raised_midpoint():
    """The via pose lies between pick and place, above both approach poses."""
    pick = (211.5, -21.6, 17.2, -5.8)
    place = (153.3, -146.9, 21.5, -43.8)
    via = transport_via(pick, place, 50.0, 300.0)
    assert via == (182.4, -84.2, 101.5, -24.8)


def test_transport_via_respects_workspace_ceiling():
    """The via height is clamped to workspace_z_max."""
    assert transport_via((0, 0, 280, 0), (0, 0, 0, 0), 50.0, 300.0)[2] == 300.0


def test_update_yaml_text_keeps_comments():
    """Only the value is replaced; comments and indentation stay."""
    text = '    pick_x: 200.0  # mm\n    pick_y: 0.0\n'
    expected = '    pick_x: 211.5  # mm\n    pick_y: 0.0\n'
    assert update_yaml_text(text, {'pick_x': 211.5}) == expected


def test_update_yaml_text_rejects_missing_key():
    """A key that is not in the file is an error, not a silent no-op."""
    with pytest.raises(RuntimeError, match='pick_q'):
        update_yaml_text('    pick_x: 1.0\n', {'pick_q': 2.0})
