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
| error_code | Optional[str] | Error identifier, or `None` |

Example:

```python
from depalletizing_interfaces.models import OperationResult

result = OperationResult(
    success=True,
    message="Movement completed",
)
```

### Shared Error Codes

Producers should import constants from `depalletizing_interfaces.errors`
instead of writing string literals:

```python
from depalletizing_interfaces.errors import ERROR_MOVE_FAILED
```

Available codes: `BUSY`, `FAULTED`, `SERVER_UNAVAILABLE`, `GOAL_REJECTED`,
`TIMEOUT`, `CANCELLED`, `MOVE_FAILED`, `OUT_OF_WORKSPACE`, `GRIPPER_FAILED`,
`WORKFLOW_BUSY`, `WORKFLOW_ERROR`. They are also re-exported from
`depalletizing_interfaces`.

## 2. Mock Components

The package provides in-process robot and gripper mocks for testing without
physical hardware. They do not create ROS action/service servers. The
`depalletizing_pick_place` package supplies those simulation nodes separately.

### MockRobot

```python
from depalletizing_interfaces.models import RobotPose
from depalletizing_interfaces.mocks import MockRobot

robot = MockRobot()
result = robot.move_to(RobotPose(200, 0, 50, 0))
```

Set `robot.should_fail = True` to simulate a movement failure.

`MockRobot.move_to(pose, motion_type=1)` also records the requested motion type
in `motion_types` and successful target poses in `movement_history`.

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

The production interfaces are asynchronous:

```text
move_to(pose: RobotPose, motion_type: int = 1) -> Future[OperationResult]
activate_tool() -> Future[OperationResult]
deactivate_tool() -> Future[OperationResult]
```

`Future[OperationResult]` describes the result contract: an `rclpy.task.Future`
whose result is an `OperationResult`.

A completed future reports the component's outcome, including immediate
rejection, failure or timeout. It does not always prove that a physical
operation has ended. In particular, the robot driver reports timeout before
it receives a terminal action result and retains its movement lock meanwhile.
Requesting cancellation also does not itself confirm standstill.

The mock components may return `OperationResult` directly. The sequencing classes accept these synchronous results for hardware-free testing.

## 4. Waypoint Sequence Execution

`WaypointExecutor` executes configured `RobotPose` objects sequentially using the asynchronous `move_to()` interface.

```python
from depalletizing_interfaces import RobotPose, WaypointExecutor
from depalletizing_interfaces.mocks import MockRobot

robot = MockRobot()
executor = WaypointExecutor(robot.move_to)

transport_waypoints = [
    RobotPose(150, 0, 120, 0),
    RobotPose(220, 80, 120, 0),
]

sequence_future = executor.move_through_waypoints(
    transport_waypoints
)

def on_transport_done(future):
    result = future.result()
    if result.success:
        print("Transport completed")
    else:
        print(result.error_code, result.message)


sequence_future.add_done_callback(on_transport_done)
```

The coordinates above are examples only. Real transport waypoints must come from the ROS configuration and must be verified on the physical robot.

The example uses synchronous mocks. With real clients, keep the ROS executor
running so movement results and continuation callbacks can be processed; do not
assume that the returned future is already complete.

The executor:

- Preserves the configured waypoint order.
- Starts the next movement only after the previous future succeeds.
- Stops after failure, timeout, cancellation, or exception.
- Checks cancellation before every waypoint.
- Returns a successful future for an empty sequence.

Cancellation of an active movement is handled by the robot driver. The executor prevents remaining waypoints from starting after cancellation.

`PickAndPlaceWorkflow` implements the following integration sequence:

```text
pick -> transport waypoints -> place
```

It receives pick/place functions and an optional `move_through_waypoints`
function at construction. `execute_pick_and_place(source, target,
transport_waypoints=())` returns a future and rejects overlapping calls with
`WORKFLOW_BUSY`. It tracks `idle`, `running`, `picking`, `transporting`,
`placing`, `completed`, `failed` and `cancelled` states. An empty transport
list skips that phase. Failure or cancellation prevents later phases; an
object may still be held. Active motion cancellation belongs to the driver.

## 5. Build Instructions

Run from the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build \
  --packages-select depalletizing_interfaces \
  --symlink-install
source install/setup.bash
```

## 6. Run Automated Tests

```bash
colcon test --packages-select depalletizing_interfaces
colcon test-result --verbose
```

The waypoint tests use mocked movement operations and require neither Gazebo nor a physical robot.

## 7. Integration Notes

All team members should use the same coordinate conventions and shared result types.

- Position: millimeters
- Rotation: degrees
- Default coordinate frame: `base_link`
- `frame_id` is metadata; these helpers do not perform coordinate transforms.
- Transport waypoints must be loaded from configuration.
- A successful result indicates completion according to the component implementation.
- Mock components do not control physical hardware.
- Physical coordinates, transformations, motion types and robot safety limits must be verified before hardware operation.
