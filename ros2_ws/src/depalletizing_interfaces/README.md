# Depalletizing Interfaces

Shared Python interfaces and testing utilities for the Applied Robotics depalletizing project.

## 1. Shared Data Models

The package provides two shared data classes.

### RobotPose

Represents a target pose for the DOBOT robot.

| Field | Type | Description |
|---|---|---|
| x | float | X position in millimeters |
| y | float | Y position in millimeters |
| z | float | Z position in millimeters |
| r | float | Rotation in degrees |
| frame_id | str | Coordinate frame, default `base_link` |

Example:

```python
from depalletizing_interfaces.models import RobotPose

target = RobotPose(x=200, y=0, z=50, r=0)
```

### OperationResult

Represents the result of a robot or gripper operation.

| Field | Type | Description |
|---|---|---|
| success | bool | Whether the operation succeeded |
| message | str | Description of the result |
| error_code | Optional[str] | Error identifier, or None |

Example:

```python
from depalletizing_interfaces.models import OperationResult

result = OperationResult(
    success=True,
    message="Movement completed"
)
```

## 2. Mock Components

The package provides simulated robot and gripper implementations for testing without physical hardware.

### MockRobot

```python
from depalletizing_interfaces.models import RobotPose
from depalletizing_interfaces.mocks import MockRobot

robot = MockRobot()
result = robot.move_to(RobotPose(200, 0, 50, 0))
```

Set `robot.should_fail = True` to simulate a movement failure.

### MockGripper

```python
from depalletizing_interfaces.mocks import MockGripper

gripper = MockGripper()
gripper.activate_tool()
gripper.deactivate_tool()
```

Set `gripper.should_fail = True` to simulate a gripper command failure.

A successful mock gripper command does not verify that a physical object has been grasped.

## 3. Interface Contracts

The following Python function signatures are supported by the mocks:

- `move_to(pose: RobotPose) -> OperationResult`
- `activate_tool() -> OperationResult`
- `deactivate_tool() -> OperationResult`

These are synchronous mock interfaces. Real ROS 2 action clients may require asynchronous execution and additional handling for timeouts or cancellation.

## 4. Waypoint Sequence Execution

`WaypointExecutor` executes `RobotPose` objects sequentially using:

```python
move_to(pose: RobotPose) -> OperationResult
```

`move_to()` must return only after success, failure, timeout, rejection, or cancellation.

### Example

```python
from depalletizing_interfaces import RobotPose, WaypointExecutor
from depalletizing_interfaces.mocks import MockRobot

robot = MockRobot()
executor = WaypointExecutor(robot.move_to)

poses = [
    RobotPose(100, 0, 50, 0),
    RobotPose(150, 0, 50, 0),
    RobotPose(200, 0, 50, 0),
]

result = executor.move_through_waypoints(poses)
```

The executor processes poses in order, waits for each movement to finish, and stops after failure or cancellation. It returns success for an empty sequence.

Active-movement cancellation must be handled by `move_to()`. Tests use a mocked `move_to()` and require neither Gazebo nor a physical robot.

## 5. Build Instructions

Run from the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select depalletizing_interfaces --symlink-install
source install/setup.bash
```

## 6. Run Automated Tests

```bash
colcon test --packages-select depalletizing_interfaces
colcon test-result --verbose
```

Latest verification: 18 tests, 0 errors, 0 failures, 1 skipped.

## 7. Integration Notes

All team members should use the same coordinate conventions and shared result types.

- Position: millimeters
- Rotation: degrees
- Default coordinate frame: `base_link`
- A successful operation result indicates command completion according to the component's implementation, not necessarily physical grasp verification
- Mock components do not control physical hardware

Before integrating with the actual DOBOT driver, verify coordinate transformations, supported motion commands and robot safety limits.