
from depalletizing_interfaces.models import RobotPose
from depalletizing_interfaces.mocks import MockRobot, MockGripper


def execute_mock_pick_and_place(robot, gripper, source, target):
    """Temporary workflow for testing shared mock interfaces."""
    result = robot.move_to(source)
    if not result.success:
        return result

    result = gripper.activate_tool()
    if not result.success:
        return result

    result = robot.move_to(target)
    if not result.success:
        return result

    return gripper.deactivate_tool()


def test_complete_pick_and_place():
    robot = MockRobot()
    gripper = MockGripper()

    source = RobotPose(200, 0, 50, 0)
    target = RobotPose(150, 100, 50, 0)

    result = execute_mock_pick_and_place(
        robot, gripper, source, target
    )

    assert result.success is True
    assert robot.movement_history == [source, target]
    assert robot.current_pose == target
    assert gripper.is_active is False


def test_pick_and_place_movement_failure():
    robot = MockRobot()
    gripper = MockGripper()
    robot.should_fail = True

    result = execute_mock_pick_and_place(
        robot,
        gripper,
        RobotPose(200, 0, 50, 0),
        RobotPose(150, 100, 50, 0),
    )

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.movement_history == []
    assert gripper.is_active is False


def test_pick_and_place_gripper_failure():
    robot = MockRobot()
    gripper = MockGripper()
    gripper.should_fail = True

    source = RobotPose(200, 0, 50, 0)
    target = RobotPose(150, 100, 50, 0)

    result = execute_mock_pick_and_place(
        robot, gripper, source, target
    )

    assert result.success is False
    assert result.error_code == "GRIPPER_FAILED"
    assert robot.movement_history == [source]
    assert gripper.is_active is False
