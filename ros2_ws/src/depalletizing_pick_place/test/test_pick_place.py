"""Tests for pick and place operations using mocked robot and gripper."""

from depalletizing_interfaces.mocks import MockGripper, MockRobot
from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_pick_place.pick_place import PickPlace


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

    def __init__(self, fail_on_call):
        super().__init__()
        self.fail_on_call = fail_on_call
        self.call_count = 0

    def move_to(self, pose: RobotPose) -> OperationResult:
        self.call_count += 1
        if self.call_count == self.fail_on_call:
            return OperationResult(
                success=False,
                message="Simulated robot movement failure",
                error_code="MOVE_FAILED",
            )
        return super().move_to(pose)


def test_pick_object_success_sequence():
    """Pick follows approach, grasp, gripper, then lift."""
    robot = MockRobot()
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)
    pose = make_pose()
    approach = approach_of(pose)

    result = pick_place.pick_object(pose)

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

    result = pick_place.place_object(pose)

    assert result.success is True
    assert gripper.is_active is False
    assert robot.movement_history == [approach, pose, approach]


def test_pick_stops_when_approach_fails():
    """Do not grasp or activate the gripper after an approach failure."""
    robot = FailOnNthRobot(fail_on_call=1)
    gripper = MockGripper()
    pick_place = PickPlace(robot, gripper)

    result = pick_place.pick_object(make_pose())

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

    result = pick_place.pick_object(pose)

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

    result = pick_place.pick_object(pose)

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

    result = pick_place.pick_object(pose)

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

    result = pick_place.place_object(make_pose())

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

    result = pick_place.place_object(pose)

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

    result = pick_place.place_object(pose)

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

    result = pick_place.place_object(pose)

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.call_count == 3
    assert gripper.is_active is False
    assert robot.movement_history == [approach_of(pose), pose]
