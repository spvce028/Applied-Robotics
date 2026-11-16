"""Tests for the asynchronous Dobot action-client robot driver.

The tests drive the real :class:`DobotActionRobot` through a fake action
client so that every movement path can be exercised deterministically.  When
ROS 2 is not installed, minimal stubs for the ROS modules are installed so the
suite still runs (the shared ``conftest.py`` already stubs ``Future``).
"""

import importlib
import sys
import types
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rclpy.task import Future


_CREATED_STUBS = []


def _install_stub(name, **attributes):
    """Create a stub module only when the real module is unavailable."""
    existing = sys.modules.get(name)
    if existing is not None:
        for key, value in attributes.items():
            if not hasattr(existing, key):
                setattr(existing, key, value)
        return existing

    try:
        return importlib.import_module(name)
    except ImportError:
        module = types.ModuleType(name)

    parent_name, _, child_name = name.rpartition(".")
    if parent_name:
        parent = _install_stub(parent_name)
        setattr(parent, child_name, module)
    sys.modules[name] = module
    _CREATED_STUBS.append((name, parent_name, child_name))

    for key, value in attributes.items():
        setattr(module, key, value)
    return module


def _remove_created_stubs():
    """Drop stub modules so other test modules still skip without ROS."""
    for name, parent_name, child_name in reversed(_CREATED_STUBS):
        sys.modules.pop(name, None)
        if parent_name:
            parent = sys.modules.get(parent_name)
            if parent is not None and hasattr(parent, child_name):
                delattr(parent, child_name)
    _CREATED_STUBS.clear()


def _install_ros_stubs():
    """Install the minimum ROS surface needed to import the driver."""
    clock_module = _install_stub("rclpy.clock")
    if not hasattr(clock_module, "ClockType"):
        class ClockType:
            ROS_TIME = 1
            STEADY_TIME = 2
            SYSTEM_TIME = 3

        class Clock:
            def __init__(self, clock_type=None):
                self.clock_type = clock_type

        clock_module.ClockType = ClockType
        clock_module.Clock = Clock

    action_module = _install_stub("rclpy.action")
    if not hasattr(action_module, "ActionClient"):
        class ActionClient:
            def __init__(self, node, action_type, action_name):
                self.node = node
                self.action_type = action_type
                self.action_name = action_name

            def server_is_ready(self):
                return False

            def wait_for_server(self, timeout_sec=0.0):
                return False

            def send_goal_async(self, goal, feedback_callback=None):
                raise RuntimeError("ActionClient stub is not configured")

        action_module.ActionClient = ActionClient

    node_module = _install_stub("rclpy.node")
    if not hasattr(node_module, "Node"):
        node_module.Node = object

    point_module = _install_stub("dobot_msgs.action")
    if not hasattr(point_module, "PointToPoint"):
        class PointToPoint:
            class Goal:
                def __init__(self):
                    self.motion_type = 0
                    self.target_pose = [0.0, 0.0, 0.0, 0.0]
                    self.velocity_ratio = 1.0
                    self.acceleration_ratio = 1.0

            class Feedback:
                def __init__(self):
                    self.current_pose = [0.0, 0.0, 0.0, 0.0]

            class Result:
                def __init__(self):
                    self.achieved_pose = [0.0, 0.0, 0.0, 0.0]

        point_module.PointToPoint = PointToPoint

    goal_status_module = _install_stub("action_msgs.msg")
    if not hasattr(goal_status_module, "GoalStatus"):
        class GoalStatus:
            STATUS_UNKNOWN = 0
            STATUS_ACCEPTED = 1
            STATUS_EXECUTING = 2
            STATUS_CANCELING = 3
            STATUS_SUCCEEDED = 4
            STATUS_CANCELED = 5
            STATUS_ABORTED = 6

        goal_status_module.GoalStatus = GoalStatus


_install_ros_stubs()

from action_msgs.msg import GoalStatus  # noqa: E402
from depalletizing_interfaces.models import RobotPose  # noqa: E402
from depalletizing_pick_place.robot_driver import (  # noqa: E402
    DobotActionRobot,
)

# The driver keeps direct references to the imported classes, so the stub
# modules can be removed again to keep other test modules isolated.
_remove_created_stubs()


def pose():
    """Return a representative target pose."""
    return RobotPose(x=200.0, y=10.0, z=40.0, r=90.0)


class FakeTimer:
    """Timer that can be fired manually by a test."""

    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False
        self.destroyed = False

    def cancel(self):
        self.cancelled = True

    def fire(self):
        self.callback()


class FakeNode:
    """Minimal node exposing the API used by the driver."""

    def __init__(self):
        self.timers = []
        self.logger = Mock()

    def get_logger(self):
        return self.logger

    def create_timer(self, period, callback, clock=None):
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.destroyed = True


class FakeGoalHandle:
    """Goal handle that lets a test control acceptance and results."""

    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0
        self._result_future = Future()
        self._cancel_future = Future()

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self._cancel_future

    def succeed(self):
        self._result_future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)
        )

    def fail(self, status=GoalStatus.STATUS_ABORTED):
        self._result_future.set_result(SimpleNamespace(status=status))

    def cancel_accepted(self):
        self._cancel_future.set_result(
            SimpleNamespace(goals_canceling=[object()])
        )

    def cancel_rejected(self):
        self._cancel_future.set_result(SimpleNamespace(goals_canceling=[]))


class FakeActionClient:
    """Action client double that records goals and drives callbacks."""

    def __init__(self, ready=True):
        self.ready = ready
        self.goals = []
        self.feedback_callback = None
        self.send_goal_future = None
        self.goal_handle = None

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec=0.0):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        self.feedback_callback = feedback_callback
        self.send_goal_future = Future()
        return self.send_goal_future

    def accept(self, accepted=True):
        self.goal_handle = FakeGoalHandle(accepted)
        self.send_goal_future.set_result(self.goal_handle)
        return self.goal_handle


def make_driver(ready=True, **kwargs):
    """Build a driver wired to a fake action client."""
    node = FakeNode()
    client = FakeActionClient(ready=ready)
    with patch(
        "depalletizing_pick_place.robot_driver.ActionClient",
        return_value=client,
    ):
        driver = DobotActionRobot(node, **kwargs)
    return driver, node, client


def test_move_to_sends_configured_goal():
    """Use configured ratios, timeout, and motion type in the goal."""
    driver, node, client = make_driver(
        velocity_ratio=0.7,
        acceleration_ratio=0.3,
        movement_timeout=12.0,
    )

    future = driver.move_to(pose(), motion_type=2)

    assert future.done() is False
    assert len(client.goals) == 1
    goal = client.goals[0]
    assert goal.motion_type == 2
    assert list(goal.target_pose) == [200.0, 10.0, 40.0, 90.0]
    assert goal.velocity_ratio == 0.7
    assert goal.acceleration_ratio == 0.3
    assert node.timers[0].period == 12.0
    assert driver.is_busy is True


def test_action_server_name_is_configurable():
    """Pass the configured action-server name to the action client."""
    node = FakeNode()
    client = FakeActionClient()
    with patch(
        "depalletizing_pick_place.robot_driver.ActionClient",
        return_value=client,
    ) as constructor:
        driver = DobotActionRobot(node, action_server_name="/custom_ptp")

    assert constructor.call_args.args[2] == "/custom_ptp"
    assert driver.action_server_name == "/custom_ptp"


def test_invalid_configuration_is_rejected():
    """Reject nonsensical motion configuration."""
    node = FakeNode()
    client = FakeActionClient()
    with patch(
        "depalletizing_pick_place.robot_driver.ActionClient",
        return_value=client,
    ):
        for kwargs in (
            {"velocity_ratio": 0.0},
            {"acceleration_ratio": 2.0},
            {"movement_timeout": 0.0},
        ):
            try:
                DobotActionRobot(node, **kwargs)
            except ValueError:
                continue
            raise AssertionError(f"Expected ValueError for {kwargs}")


def test_move_to_reports_success():
    """Resolve with success and release the lock after a terminal result."""
    driver, node, client = make_driver()
    future = driver.move_to(pose())

    handle = client.accept()
    handle.succeed()

    result = future.result()
    assert result.success is True
    assert result.error_code is None
    assert driver.is_busy is False
    assert node.timers[0].cancelled is True


def test_goal_rejection_is_reported_and_unlocks():
    """Report GOAL_REJECTED and free the driver for a new movement."""
    driver, _, client = make_driver()
    future = driver.move_to(pose())

    client.accept(accepted=False)

    result = future.result()
    assert result.success is False
    assert result.error_code == "GOAL_REJECTED"
    assert driver.is_busy is False


def test_movement_failure_is_reported():
    """Report MOVE_FAILED for an aborted action."""
    driver, _, client = make_driver()
    future = driver.move_to(pose())

    client.accept().fail(GoalStatus.STATUS_ABORTED)

    result = future.result()
    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert driver.is_busy is False


def test_cancellation_is_requested_and_reported():
    """Request cancellation and report CANCELLED on a canceled result."""
    driver, _, client = make_driver()
    future = driver.move_to(pose())
    handle = client.accept()

    assert driver.cancel_current_goal() is True
    assert handle.cancel_calls == 1
    handle.cancel_accepted()
    handle.fail(GoalStatus.STATUS_CANCELED)

    result = future.result()
    assert result.success is False
    assert result.error_code == "CANCELLED"
    assert driver.is_busy is False


def test_cancel_without_active_movement_returns_none():
    """Do nothing when no movement is active."""
    driver, _, _ = make_driver()
    assert driver.cancel_current_goal() is None


def test_timeout_reports_and_cancels_while_locked():
    """Timeout reports TIMEOUT, cancels, and blocks a second movement."""
    driver, node, client = make_driver(movement_timeout=1.0)
    future = driver.move_to(pose())
    handle = client.accept()

    node.timers[0].fire()

    assert future.done() is True
    assert future.result().error_code == "TIMEOUT"
    assert handle.cancel_calls == 1
    assert driver.is_busy is True

    second = driver.move_to(pose())
    assert second.result().error_code == "BUSY"
    assert len(client.goals) == 1

    handle.fail(GoalStatus.STATUS_CANCELED)

    assert driver.is_busy is False
    # The terminal result must not overwrite the reported timeout.
    assert future.result().error_code == "TIMEOUT"


def test_late_goal_acceptance_is_cancelled():
    """Cancel a goal that is accepted only after the timeout fired."""
    driver, node, client = make_driver(movement_timeout=1.0)
    future = driver.move_to(pose())

    node.timers[0].fire()
    assert future.result().error_code == "TIMEOUT"

    handle = client.accept()

    assert handle.cancel_calls == 1
    assert driver.is_busy is True
    assert driver.move_to(pose()).result().error_code == "BUSY"

    handle.fail(GoalStatus.STATUS_CANCELED)
    assert driver.is_busy is False
    assert future.result().error_code == "TIMEOUT"


def test_unavailable_server_fails_without_sending():
    """Report SERVER_UNAVAILABLE and never send a goal."""
    driver, _, client = make_driver(ready=False)

    result = driver.move_to(pose()).result()

    assert result.success is False
    assert result.error_code == "SERVER_UNAVAILABLE"
    assert client.goals == []
    assert driver.is_busy is False


def test_overlapping_commands_are_rejected():
    """Reject a second movement while the first is still active."""
    driver, _, client = make_driver()
    first = driver.move_to(pose())
    second = driver.move_to(pose())

    assert second.done() is True
    assert second.result().error_code == "BUSY"
    assert len(client.goals) == 1

    client.accept().succeed()
    assert first.result().success is True


def test_cancellation_rejection_faults_and_locks():
    """Fault and stay locked when the server rejects cancellation."""
    driver, _, client = make_driver()
    future = driver.move_to(pose())
    handle = client.accept()

    driver.cancel_current_goal()
    handle.cancel_rejected()

    assert driver.is_faulted is True
    assert future.result().error_code == "FAULTED"
    assert driver.is_busy is True
    assert driver.reset_fault() is False

    blocked = driver.move_to(pose())
    assert blocked.result().error_code == "FAULTED"

    # A terminal result clears the movement but keeps the fault until reset.
    handle.fail(GoalStatus.STATUS_CANCELED)
    assert driver.is_busy is False
    assert driver.is_faulted is True

    assert driver.reset_fault() is True
    assert driver.is_faulted is False

    recovered = driver.move_to(pose())
    assert recovered.done() is False
    client.accept().succeed()
    assert recovered.result().success is True


def test_force_reset_clears_uncertain_fault():
    """Force a reset only after an uncertain state has been acknowledged."""
    driver, _, client = make_driver()
    driver.move_to(pose())
    handle = client.accept()
    driver.cancel_current_goal()
    handle.cancel_rejected()

    assert driver.is_faulted is True
    assert driver.is_busy is True

    assert driver.reset_fault(force=True) is True
    assert driver.is_faulted is False
    assert driver.is_busy is False


def test_feedback_is_processed():
    """Store and log meaningful movement feedback."""
    driver, node, client = make_driver()
    driver.move_to(pose())
    client.accept()

    client.feedback_callback(
        SimpleNamespace(
            feedback=SimpleNamespace(current_pose=[1.0, 2.0, 3.0, 4.0])
        )
    )

    assert driver.last_feedback == [1.0, 2.0, 3.0, 4.0]
    assert node.logger.debug.called


def test_reset_without_fault_is_a_noop():
    """Resetting a healthy driver succeeds without side effects."""
    driver, _, _ = make_driver()
    assert driver.reset_fault() is True
    assert driver.is_faulted is False
