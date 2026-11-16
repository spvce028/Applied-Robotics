"""Tests for pick and place operations using mocked robot and gripper."""

from rclpy.task import Future

from depalletizing_interfaces.mocks import MockGripper, MockRobot
from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_pick_place.pick_place import HELD_NOTE, MOVL, MOVJ, PickPlace


APPROACH_OFFSET_Z = 50.0


def make_pose():
    """Create a representative object pose."""
    return RobotPose(x=200.0, y=10.0, z=40.0, r=90.0)


def approach_of(pose):
    """Return the approach pose used by pick and place."""
    return RobotPose(
        x=pose.x,
        y=pose.y,
        z=pose.z + APPROACH_OFFSET_Z,
        r=pose.r,
        frame_id=pose.frame_id,
    )


class FailOnNthRobot(MockRobot):
    """Fail on a specific 1-based move_to call."""

    def __init__(
        self,
        fail_on_call,
        error_code="MOVE_FAILED",
        message="Simulated robot movement failure",
    ):
        super().__init__()
        self.fail_on_call = fail_on_call
        self.error_code = error_code
        self.message = message
        self.call_count = 0

    def move_to(self, pose: RobotPose, motion_type: int = 1) -> OperationResult:
        self.call_count += 1
        if self.call_count == self.fail_on_call:
            return OperationResult(
                success=False,
                message=self.message,
                error_code=self.error_code,
            )
        return super().move_to(pose, motion_type)


def test_pick_object_success_sequence():
    """Pick follows approach, grasp, gripper, then lift."""
    robot = MockRobot()
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()
    approach = approach_of(pose)

    result = pick_place.pick_object(pose).result()

    assert result.success is True
    assert gripper.is_active is True
    assert robot.movement_history == [approach, pose, approach]


def test_place_object_success_sequence():
    """Place follows approach, placement, gripper release, then retreat."""
    robot = MockRobot()
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()
    approach = approach_of(pose)

    result = pick_place.place_object(pose).result()

    assert result.success is True
    assert gripper.is_active is False
    assert robot.movement_history == [approach, pose, approach]


def test_pick_stops_when_approach_fails():
    """Do not grasp or activate the gripper after an approach failure."""
    robot = FailOnNthRobot(fail_on_call=1)
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)

    result = pick_place.pick_object(make_pose()).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 1
    assert robot.movement_history == []
    assert gripper.is_active is False


def test_pick_stops_when_grasp_move_fails():
    """Do not activate the gripper after a grasp-pose failure."""
    robot = FailOnNthRobot(fail_on_call=2)
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.pick_object(pose).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 2
    assert robot.movement_history == [approach_of(pose)]
    assert gripper.is_active is False


def test_pick_stops_when_gripper_fails():
    """Do not lift after a gripper activation failure."""
    robot = MockRobot()
    gripper = MockGripper()
    gripper.should_fail = True
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.pick_object(pose).result()

    assert result.success is False
    assert result.error_code == "GRIPPER_FAILED"
    assert gripper.is_active is False
    assert robot.movement_history == [approach_of(pose), pose]


def test_pick_stops_when_lift_fails():
    """Report lift failure after the object has already been grasped."""
    robot = FailOnNthRobot(fail_on_call=3)
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.pick_object(pose).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 3
    assert gripper.is_active is True
    assert robot.movement_history == [approach_of(pose), pose]


def test_place_stops_when_approach_fails():
    """Do not move to the place pose after an approach failure."""
    robot = FailOnNthRobot(fail_on_call=1)
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)

    result = pick_place.place_object(make_pose()).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 1
    assert robot.movement_history == []
    assert gripper.is_active is True


def test_place_stops_when_placement_move_fails():
    """Do not release the gripper after a placement-pose failure."""
    robot = FailOnNthRobot(fail_on_call=2)
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.place_object(pose).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 2
    assert robot.movement_history == [approach_of(pose)]
    assert gripper.is_active is True


def test_place_stops_when_gripper_fails():
    """Do not retreat after a gripper deactivation failure."""
    robot = MockRobot()
    gripper = MockGripper()
    gripper.activate_tool()
    gripper.should_fail = True
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.place_object(pose).result()

    assert result.success is False
    assert result.error_code == "GRIPPER_FAILED"
    assert gripper.is_active is True
    assert robot.movement_history == [approach_of(pose), pose]


def test_place_stops_when_retreat_fails():
    """Report retreat failure after the object has already been released."""
    robot = FailOnNthRobot(fail_on_call=3)
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.place_object(pose).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 3
    assert gripper.is_active is False
    assert robot.movement_history == [approach_of(pose), pose]


class AsyncGripper:
    """Gripper whose commands finish only when the returned Future is set."""

    def __init__(self):
        self.is_active = False
        self.pending = None

    def activate_tool(self):
        self.pending = Future()
        return self.pending

    def deactivate_tool(self):
        self.pending = Future()
        return self.pending


def test_pick_waits_for_async_gripper_before_lift():
    """Do not lift until an asynchronous close command finishes."""
    robot = MockRobot()
    gripper = AsyncGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    workflow = pick_place.pick_object(pose)

    assert workflow.done() is False
    assert robot.movement_history == [approach_of(pose), pose]
    gripper.pending.set_result(OperationResult(success=True, message="closed"))

    assert workflow.result().success is True
    assert robot.movement_history == [approach_of(pose), pose, approach_of(pose)]


def test_pick_does_not_lift_when_async_gripper_fails():
    """Stop the pick sequence when an asynchronous close command fails."""
    robot = MockRobot()
    gripper = AsyncGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    workflow = pick_place.pick_object(pose)
    gripper.pending.set_result(
        OperationResult(
            success=False,
            message="close failed",
            error_code="GRIPPER_FAILED",
        )
    )

    result = workflow.result()
    assert result.success is False
    assert result.error_code == "GRIPPER_FAILED"
    assert robot.movement_history == [approach_of(pose), pose]


def test_place_waits_for_async_gripper_before_retreat():
    """Do not retreat until an asynchronous open command finishes."""
    robot = MockRobot()
    gripper = AsyncGripper()
    gripper.is_active = True
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    workflow = pick_place.place_object(pose)

    assert workflow.done() is False
    assert robot.movement_history == [approach_of(pose), pose]
    gripper.is_active = False
    gripper.pending.set_result(OperationResult(success=True, message="opened"))

    assert workflow.result().success is True
    assert robot.movement_history == [approach_of(pose), pose, approach_of(pose)]


def test_approach_height_is_configurable():
    """Use the configured approach height instead of a fixed offset."""
    robot = MockRobot()
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper, approach_height=80.0)
    pose = make_pose()
    approach = RobotPose(
        x=pose.x,
        y=pose.y,
        z=pose.z + 80.0,
        r=pose.r,
        frame_id=pose.frame_id,
    )

    result = pick_place.pick_object(pose).result()

    assert result.success is True
    assert robot.movement_history == [approach, pose, approach]


def test_pick_stops_when_approach_is_cancelled():
    """Do not descend after a cancelled approach movement."""
    robot = FailOnNthRobot(
        fail_on_call=1,
        error_code="CANCELLED",
        message="Motion was canceled.",
    )
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)

    result = pick_place.pick_object(make_pose()).result()

    assert result.success is False
    assert result.error_code == "CANCELLED"
    assert robot.call_count == 1
    assert gripper.is_active is False


def test_place_stops_when_descent_times_out():
    """Do not open the gripper after a timed-out placement movement."""
    robot = FailOnNthRobot(
        fail_on_call=2,
        error_code="TIMEOUT",
        message="Motion execution timed out.",
    )
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.place_object(pose).result()

    assert result.success is False
    assert result.error_code == "TIMEOUT"
    assert robot.call_count == 2
    assert robot.movement_history == [approach_of(pose)]
    assert gripper.is_active is True


class AsyncRobot:
    """Robot whose movements finish only when the returned Future is set."""

    def __init__(self):
        self.calls = []
        self.motion_types = []
        self.pending = None

    def move_to(self, pose, motion_type=1):
        self.calls.append(pose)
        self.motion_types.append(motion_type)
        self.pending = Future()
        return self.pending


def test_async_robot_and_gripper_follow_pick_order():
    """Start each pick step only after the previous future succeeds."""
    robot = AsyncRobot()
    gripper = AsyncGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()
    approach = approach_of(pose)

    workflow = pick_place.pick_object(pose)

    assert workflow.done() is False
    assert robot.calls == [approach]
    assert gripper.pending is None

    robot.pending.set_result(OperationResult(success=True))
    assert robot.calls == [approach, pose]

    robot.pending.set_result(OperationResult(success=True))
    assert gripper.pending is not None
    assert gripper.pending.done() is False
    assert robot.calls == [approach, pose]

    gripper.pending.set_result(OperationResult(success=True))
    assert robot.calls == [approach, pose, approach]

    robot.pending.set_result(OperationResult(success=True))
    assert workflow.result().success is True


def test_pick_passes_shared_error_codes_through():
    """Keep the robot client's error code and skip the remaining steps."""
    codes = (
        "CANCELLED",
        "TIMEOUT",
        "SERVER_UNAVAILABLE",
        "GOAL_REJECTED",
        "MOVE_FAILED",
    )

    for error_code in codes:
        robot = FailOnNthRobot(fail_on_call=1, error_code=error_code)
        gripper = MockGripper()
        pick_place = PickPlace(robot, gripper)

        result = pick_place.pick_object(make_pose()).result()

        assert result.success is False
        assert result.error_code == error_code
        assert robot.call_count == 1
        assert gripper.is_active is False


def test_step_exception_returns_workflow_error():
    """Turn an unexpected step exception into WORKFLOW_ERROR."""
    robot = MockRobot()
    gripper = MockGripper()

    def broken_move(pose, motion_type=1):
        raise RuntimeError("driver crashed")

    robot.move_to = broken_move
    pick_place = PickPlace(robot, gripper)

    result = pick_place.pick_object(make_pose()).result()

    assert result.success is False
    assert result.error_code == "WORKFLOW_ERROR"
    assert "driver crashed" in result.message
    assert gripper.is_active is False


def test_vertical_moves_use_linear_motion():
    """Approach with MOVJ. Descend and lift with MOVL."""
    robot = MockRobot()
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    pick_result = pick_place.pick_object(pose).result()
    assert pick_result.success is True
    assert robot.motion_types == [MOVJ, MOVL, MOVL]
    assert pick_place.holding_object is True

    robot.motion_types.clear()
    place_result = pick_place.place_object(pose).result()
    assert place_result.success is True
    assert robot.motion_types == [MOVJ, MOVL, MOVL]
    assert pick_place.holding_object is False


def test_cancel_between_steps_skips_the_descent():
    """Do not start the next movement after cancellation."""
    cancelled = False

    def is_cancelled():
        return cancelled

    robot = AsyncRobot()
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper, is_cancelled=is_cancelled)
    pose = make_pose()

    workflow = pick_place.pick_object(pose)
    assert robot.calls == [approach_of(pose)]
    assert workflow.done() is False

    cancelled = True
    robot.pending.set_result(OperationResult(success=True))

    result = workflow.result()
    assert result.success is False
    assert result.error_code == "CANCELLED"
    assert robot.calls == [approach_of(pose)]
    assert gripper.is_active is False
    assert pick_place.holding_object is False
    assert HELD_NOTE not in result.message


def test_cancel_after_grasp_keeps_the_object_held():
    """Leave the gripper closed when cancellation happens after grasping."""
    cancelled = False

    def is_cancelled():
        return cancelled

    robot = AsyncRobot()
    gripper = AsyncGripper()
    pick_place = PickPlace(robot, gripper, is_cancelled=is_cancelled)
    pose = make_pose()
    approach = approach_of(pose)

    workflow = pick_place.pick_object(pose)
    robot.pending.set_result(OperationResult(success=True))
    robot.pending.set_result(OperationResult(success=True))
    assert robot.calls == [approach, pose]

    cancelled = True
    gripper.pending.set_result(OperationResult(success=True, message="closed"))

    result = workflow.result()
    assert result.success is False
    assert result.error_code == "CANCELLED"
    assert HELD_NOTE in result.message
    assert robot.calls == [approach, pose]
    assert pick_place.holding_object is True


def test_lift_failure_leaves_the_object_held():
    """Do not open the gripper when the lift fails after a grasp."""
    robot = FailOnNthRobot(fail_on_call=3)
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()

    result = pick_place.pick_object(pose).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert HELD_NOTE in result.message
    assert gripper.is_active is True
    assert pick_place.holding_object is True
    assert robot.movement_history == [approach_of(pose), pose]


def test_place_failure_before_release_keeps_the_object_held():
    """Do not open the gripper when placement fails before release."""
    robot = FailOnNthRobot(fail_on_call=1)
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)
    pick_place.holding_object = True

    result = pick_place.place_object(make_pose()).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert HELD_NOTE in result.message
    assert gripper.is_active is True
    assert pick_place.holding_object is True


def test_retreat_failure_after_release_is_not_holding():
    """A failed retreat after release does not report a held object."""
    robot = FailOnNthRobot(fail_on_call=3)
    gripper = MockGripper()
    gripper.activate_tool()
    pick_place = PickPlace(robot, gripper)

    result = pick_place.place_object(make_pose()).result()

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert HELD_NOTE not in result.message
    assert gripper.is_active is False
    assert pick_place.holding_object is False
