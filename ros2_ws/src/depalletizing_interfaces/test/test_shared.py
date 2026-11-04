
from depalletizing_interfaces.models import RobotPose, OperationResult
from depalletizing_interfaces.mocks import MockRobot, MockGripper


def test_robot_pose():
    pose = RobotPose(200, 0, 50, 0)

    assert pose.x == 200
    assert pose.z == 50
    assert pose.frame_id == "base_link"


def test_operation_result():
    result = OperationResult(True, "Success")

    assert result.success is True
    assert result.message == "Success"
    assert result.error_code is None


def test_robot_movement_success():
    robot = MockRobot()
    pose = RobotPose(200, 0, 50, 0)

    result = robot.move_to(pose)

    assert result.success is True
    assert robot.current_pose == pose
    assert len(robot.movement_history) == 1


def test_robot_movement_failure():
    robot = MockRobot()
    robot.should_fail = True

    result = robot.move_to(RobotPose(200, 0, 50, 0))

    assert result.success is False
    assert result.error_code == "MOVE_FAILED"
    assert robot.current_pose is None
    assert robot.movement_history == []


def test_gripper_activation():
    gripper = MockGripper()

    result = gripper.activate_tool()

    assert result.success is True
    assert gripper.is_active is True


def test_gripper_deactivation():
    gripper = MockGripper()
    gripper.activate_tool()

    result = gripper.deactivate_tool()

    assert result.success is True
    assert gripper.is_active is False


def test_gripper_failure():
    gripper = MockGripper()
    gripper.should_fail = True

    result = gripper.activate_tool()

    assert result.success is False
    assert result.error_code == "GRIPPER_FAILED"
    assert gripper.is_active is False
