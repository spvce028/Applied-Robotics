"""Tests for the fake PTP action server used by simulation mode.

Minimal ROS stubs let the server be exercised without a ROS installation. The
tests drive the real execute-loop state machine with a controllable clock and
a patched sleep, so they run instantly and deterministically.
"""

import importlib
import sys
import types
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest


_CREATED_STUBS = []


def _module(name):
    """Return an existing module, the real one, or a new stub module."""
    existing = sys.modules.get(name)
    if existing is not None:
        return existing
    try:
        return importlib.import_module(name)
    except ImportError:
        module = types.ModuleType(name)
        parent_name, _, child_name = name.rpartition(".")
        if parent_name:
            setattr(_module(parent_name), child_name, module)
        sys.modules[name] = module
        _CREATED_STUBS.append(name)
        return module


def _remove_created_stubs():
    """Unregister stubs so other test modules see their normal state."""
    for name in reversed(_CREATED_STUBS):
        parent_name, _, child_name = name.rpartition(".")
        parent = sys.modules.get(parent_name)
        if parent is not None and hasattr(parent, child_name):
            delattr(parent, child_name)
        sys.modules.pop(name, None)


class FakeTime:
    """Time value exposing the nanoseconds field used by the server."""

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


class FakeClock:
    """Clock whose time only advances when a test asks it to."""

    def __init__(self, clock_type=None):
        self.clock_type = clock_type
        self.nanoseconds = 0

    def now(self):
        """Return the current fake time."""
        return FakeTime(self.nanoseconds)

    def advance(self, seconds):
        """Move the clock forward by ``seconds``."""
        self.nanoseconds += int(seconds * 1e9)


class FakeNode:
    """Node stub exposing only the API used by the fake server."""

    def __init__(self, *args, **kwargs):
        self._logger = Mock()
        self._parameters = {}

    def get_logger(self):
        """Return the logger double."""
        return self._logger

    def declare_parameter(self, name, value):
        """Store a parameter default in the stub."""
        self._parameters.setdefault(name, value)

    def get_parameter(self, name):
        """Return a parameter value in the stub."""
        return SimpleNamespace(value=self._parameters[name])

    def destroy_node(self):
        """Node destruction is a no-op in the stub."""


class FakeActionServer:
    """Record the callbacks registered by the fake server."""

    def __init__(
        self, node, action_type, action_name,
        execute_callback=None, goal_callback=None, cancel_callback=None,
    ):
        self.node = node
        self.action_name = action_name
        self.execute_callback = execute_callback
        self.goal_callback = goal_callback
        self.cancel_callback = cancel_callback


class FakeGoalHandle:
    """Goal handle double matching rclpy's ``is_cancel_requested`` property."""

    def __init__(self, target, motion_type=1):
        self.request = SimpleNamespace(
            target_pose=target, motion_type=motion_type,
        )
        self.feedback = []
        self.cancel_requested = False
        self.status = None

    @property
    def is_cancel_requested(self):
        """Return whether cancellation was requested."""
        return self.cancel_requested

    def publish_feedback(self, feedback_msg):
        """Record one feedback message."""
        self.feedback.append(list(feedback_msg.current_pose))

    def succeed(self):
        """Record a successful terminal state."""
        self.status = "succeeded"

    def canceled(self):
        """Record a canceled terminal state."""
        self.status = "canceled"

    def abort(self):
        """Record an aborted terminal state."""
        self.status = "aborted"


def _install_stubs():
    """Install the minimal ROS surface needed to import the fake server."""
    clock_module = _module("rclpy.clock")
    if not hasattr(clock_module, "ClockType"):
        clock_module.ClockType = SimpleNamespace(STEADY_TIME=2)
    if not hasattr(clock_module, "Clock"):
        clock_module.Clock = FakeClock

    node_module = _module("rclpy.node")
    if not hasattr(node_module, "Node"):
        node_module.Node = FakeNode

    action_module = _module("rclpy.action")
    if not hasattr(action_module, "ActionServer"):
        action_module.ActionServer = FakeActionServer
        action_module.GoalResponse = SimpleNamespace(ACCEPT=1, REJECT=2)
        action_module.CancelResponse = SimpleNamespace(ACCEPT=1, REJECT=2)

    executors_module = _module("rclpy.executors")
    if not hasattr(executors_module, "MultiThreadedExecutor"):
        executors_module.MultiThreadedExecutor = object

    action_msgs = _module("dobot_msgs.action")

    class PointToPoint:
        """Stub of the dobot_msgs PointToPoint action types."""

        class Goal:
            def __init__(self):
                self.motion_type = 1
                self.target_pose = [0.0, 0.0, 0.0, 0.0]

        class Feedback:
            def __init__(self):
                self.current_pose = None

        class Result:
            def __init__(self):
                self.achieved_pose = None

    if not hasattr(action_msgs, "PointToPoint"):
        action_msgs.PointToPoint = PointToPoint


_install_stubs()

from rclpy.action import CancelResponse, GoalResponse  # noqa: E402
from depalletizing_pick_place import fake_ptp_action_server as fake_server  # noqa: E402

# The driver module keeps direct references to the imported classes, so the
# stub modules can be removed to keep other test modules isolated.
_remove_created_stubs()


@pytest.fixture(autouse=True)
def _ros_context():
    """Initialise real rclpy for these tests; the local stub needs no context."""
    rclpy_module = _module("rclpy")
    if not hasattr(rclpy_module, "init"):
        yield
        return

    started_here = not rclpy_module.ok()
    if started_here:
        rclpy_module.init()
    yield
    if started_here:
        rclpy_module.shutdown()


def make_server(clock, **kwargs):
    """Create a fake server with a controllable clock and action server.

    ``Node`` is deliberately not patched: ``super().__init__()`` resolves the
    original base class, so a real ROS install needs a live rclpy context
    (provided by the fixture above).
    """
    with patch.object(fake_server, "Clock", lambda clock_type=None: clock), \
            patch.object(fake_server, "ActionServer", FakeActionServer):
        return fake_server.FakePTPActionServer(**kwargs)


def test_interpolate_pose_clamps_and_interpolates():
    """Interpolate between two poses and clamp out-of-range progress."""
    start = [0.0, 0.0, 0.0, 0.0]
    target = [100.0, -20.0, 50.0, 90.0]

    assert fake_server.interpolate_pose(start, target, 0.0) == start
    assert fake_server.interpolate_pose(start, target, 0.5) == [
        50.0, -10.0, 25.0, 45.0,
    ]
    assert fake_server.interpolate_pose(start, target, 1.5) == target
    assert fake_server.interpolate_pose(start, target, -1.0) == start


def test_constructor_rejects_non_positive_timing():
    """Reject invalid feedback intervals and movement durations."""
    with pytest.raises(ValueError):
        make_server(FakeClock(), feedback_interval=0.0)
    with pytest.raises(ValueError):
        make_server(FakeClock(), movement_duration=-1.0)


def test_goal_callback_validates_motion_type_and_pose():
    """Reject unknown motion types, malformed poses and non-finite values."""
    server = make_server(FakeClock())

    valid = SimpleNamespace(motion_type=2, target_pose=[1.0, 2.0, 3.0, 4.0])
    assert server.goal_callback(valid) == GoalResponse.ACCEPT

    for goal in (
        SimpleNamespace(motion_type=3, target_pose=[1.0, 2.0, 3.0, 4.0]),
        SimpleNamespace(motion_type=1, target_pose=[1.0, 2.0, 3.0]),
        SimpleNamespace(
            motion_type=1, target_pose=[float("nan"), 2.0, 3.0, 4.0],
        ),
    ):
        assert server.goal_callback(goal) == GoalResponse.REJECT


def test_movement_publishes_feedback_then_succeeds():
    """Drive a full simulated movement through feedback to success."""
    clock = FakeClock()
    server = make_server(clock, movement_duration=1.0, feedback_interval=0.1)
    handle = FakeGoalHandle([100.0, 0.0, 50.0, 0.0])

    with patch.object(
        fake_server.time, "sleep",
        side_effect=lambda seconds: clock.advance(seconds),
    ):
        result = server.execute_callback(handle)

    assert handle.status == "succeeded"
    assert result.achieved_pose == pytest.approx([100.0, 0.0, 50.0, 0.0])
    assert len(handle.feedback) == 11
    assert handle.feedback[-1] == pytest.approx([100.0, 0.0, 50.0, 0.0])
    assert server.current_pose == pytest.approx([100.0, 0.0, 50.0, 0.0])
    assert server._active is None


def test_cancel_request_finishes_with_canceled_status():
    """Honour a cancellation request on the next feedback tick."""
    clock = FakeClock()
    server = make_server(clock, movement_duration=1.0, feedback_interval=0.1)
    handle = FakeGoalHandle([100.0, 0.0, 50.0, 0.0])
    sleeps = []

    def sleep_and_cancel(seconds):
        clock.advance(seconds)
        sleeps.append(seconds)
        if len(sleeps) == 2:
            handle.cancel_requested = True

    with patch.object(fake_server.time, "sleep", side_effect=sleep_and_cancel):
        result = server.execute_callback(handle)

    assert server.cancel_callback(handle) == CancelResponse.ACCEPT
    assert handle.status == "canceled"
    assert result.achieved_pose == pytest.approx([20.0, 0.0, 10.0, 0.0])
    assert len(handle.feedback) == 3
    assert server._active is None


def test_overlapping_goal_is_aborted():
    """Abort a second goal while another movement is still active."""
    clock = FakeClock()
    server = make_server(clock)
    first = FakeGoalHandle([10.0, 0.0, 0.0, 0.0])
    second = FakeGoalHandle([20.0, 0.0, 0.0, 0.0])
    second_results = []

    def sleep_and_start_second(seconds):
        clock.advance(seconds)
        if not second_results:
            second_results.append(server.execute_callback(second))

    with patch.object(
        fake_server.time, "sleep", side_effect=sleep_and_start_second,
    ):
        first_result = server.execute_callback(first)

    assert second.status == "aborted"
    assert second_results[0].achieved_pose == pytest.approx(
        [0.0, 0.0, 0.0, 0.0]
    )
    assert first_result.achieved_pose == pytest.approx([10.0, 0.0, 0.0, 0.0])
    assert server._active is None
