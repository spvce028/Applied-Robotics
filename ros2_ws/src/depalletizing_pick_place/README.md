# Depalletizing Pick and Place

Pick-and-place operations and the asynchronous DOBOT action client for the
Applied Robotics depalletizing project.

The main entry point for robot motion is `DobotActionRobot`, an asynchronous
ROS 2 action client for the `dobot_msgs/action/PointToPoint` action server
(`/PTP_action` by default).

## 1. Overview

`DobotActionRobot` sends Point-to-Point movement goals and reports the outcome
as a `Future` containing an `OperationResult`. It never blocks the ROS
executor, it serialises movements (one active goal at a time), and it guards
against timeouts, cancellation, and uncertain robot states.

| Concern | Behaviour |
|---|---|
| Blocking | `move_to()` returns immediately |
| Concurrency | Only one movement may be active; extra calls return `BUSY` |
| Timeout | Reports `TIMEOUT`, cancels the goal, stays locked until a terminal result |
| Cancellation | `cancel_current_goal()` requests cancellation of the active goal |
| Faults | Uncertain states fault the driver; recover with `reset_fault()` or restart |
| Workspace | Poses outside configured limits are rejected before a goal is sent |
| Feedback | Latest `current_pose` is stored in `last_feedback` and logged |

## 2. Requirements

- ROS 2 (Humble) with `rclpy`
- `dobot_msgs` (`PointToPoint` action) and `action_msgs`
- The `depalletizing_interfaces` package (`RobotPose`, `OperationResult`)

## 3. Constructing the Driver

All motion settings are configurable; the defaults match the original
hardcoded values.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `action_server_name` | str | `/PTP_action` | PointToPoint action server name |
| `velocity_ratio` | float | `0.4` | Motion velocity ratio, `(0.0, 1.0]` |
| `acceleration_ratio` | float | `0.4` | Motion acceleration ratio, `(0.0, 1.0]` |
| `movement_timeout` | float | `30.0` | Seconds before a goal times out |
| `workspace_limits` | `WorkspaceLimits` | conservative Dobot envelope | Cartesian bounds checked before each goal |

```python
from depalletizing_pick_place.robot_driver import (
    DobotActionRobot,
    WorkspaceLimits,
)

# Measure the real cell before a hardware run and adjust these bounds.
limits = WorkspaceLimits(
    x=(-320.0, 320.0), y=(-320.0, 320.0),
    z=(-100.0, 300.0), r=(-360.0, 360.0),
)
robot = DobotActionRobot(
    node,
    action_server_name="/PTP_action",
    velocity_ratio=0.5,
    acceleration_ratio=0.3,
    movement_timeout=15.0,
    workspace_limits=limits,
)

if not robot.wait_for_server(timeout_sec=5.0):
    node.get_logger().error("Dobot action server is not available")
```

Invalid configuration (ratios outside `(0.0, 1.0]` or a non-positive timeout)
raises `ValueError` at construction time.

## 4. Sending a Movement

`move_to()` returns a `Future[OperationResult]`. Attach a done callback or
await it from a coroutine; do not call `spin_until_future_complete` from
inside a callback.

```python
from depalletizing_interfaces.models import RobotPose

target = RobotPose(x=200.0, y=0.0, z=50.0, r=0.0)

def on_done(future):
    result = future.result()
    if result.success:
        node.get_logger().info("Reached target")
    else:
        node.get_logger().error(f"{result.error_code}: {result.message}")

future = robot.move_to(target)          # motion_type defaults to 1 (MOVJ XYZ)
future.add_done_callback(on_done)
```

`motion_type` is passed straight to the action goal (1 = joint-interpolated
Cartesian, 2 = linear Cartesian, 4/5 = joint coordinates).

## 5. Result and Error Codes

Every code path resolves the future with an `OperationResult`:

| `error_code` | `success` | Meaning |
|---|---|---|
| `None` | True | Motion completed successfully |
| `BUSY` | False | Another movement is already active |
| `SERVER_UNAVAILABLE` | False | Action server is not ready; no goal sent |
| `GOAL_REJECTED` | False | Server rejected the goal (e.g. workspace limit) |
| `MOVE_FAILED` | False | Motion aborted or failed on the server |
| `CANCELLED` | False | Motion was canceled |
| `TIMEOUT` | False | Motion exceeded `movement_timeout` |
| `FAULTED` | False | Driver is faulted; recovery required |
| `OUT_OF_WORKSPACE` | False | Target pose is outside configured workspace limits |

## 6. Timeout and Cancellation

On timeout the driver:

1. resolves the future with `TIMEOUT`,
2. requests cancellation of the active goal,
3. keeps the driver locked until a terminal action result is received.

While locked, `move_to()` returns `BUSY`, so a second command cannot be sent
while the first goal may still be moving. A terminal result (succeeded,
aborted, canceled, or rejected) releases the lock but never overwrites the
already-reported result.

```python
# Ask the server to stop the current movement.
robot.cancel_current_goal()

# Status helpers
robot.is_busy        # True while a movement is active or awaiting a result
robot.is_faulted     # True after an uncertain state
robot.fault_reason   # Human-readable fault reason, or None
```

## 7. Faults and Recovery

The driver enters a fault when the robot state becomes uncertain, for example
when the server rejects a cancellation request or a result cannot be
retrieved. While faulted, every `move_to()` returns `FAULTED`.

```python
if robot.is_faulted:
    # A plain reset is refused while a movement may still be active.
    if not robot.reset_fault():
        node.get_logger().error(
            "Robot state uncertain - restart the node or verify safety first"
        )
```

- `reset_fault()` clears the fault once no movement is active.
- `reset_fault(force=True)` clears the lock unconditionally; only use it after
  verifying the robot is stationary.
- If a movement can never reach a terminal result, restart the node.

## 8. Feedback

While a movement is running the server publishes its current pose. The driver
stores the most recent value and logs changes at debug level.

```python
latest = robot.last_feedback   # [x, y, z, r] in mm/degrees, or None
```

## 9. End-to-End Example

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor

from depalletizing_interfaces.models import RobotPose
from depalletizing_pick_place.robot_driver import DobotActionRobot

rclpy.init()
node = rclpy.create_node("dobot_example")
robot = DobotActionRobot(node, movement_timeout=10.0)

if not robot.wait_for_server(timeout_sec=5.0):
    node.get_logger().error("Server unavailable")
else:
    def on_done(future):
        result = future.result()
        node.get_logger().info(f"{result.error_code}: {result.message}")

    robot.move_to(RobotPose(200.0, 0.0, 50.0, 0.0)).add_done_callback(on_done)

executor = MultiThreadedExecutor()
executor.add_node(node)
try:
    executor.spin()
finally:
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
```

## 10. Workspace Limits

`WorkspaceLimits` checks x, y, z and r against axis-aligned bounds before a
goal is sent. Non-finite pose values are rejected as well. A rejected pose
resolves immediately with `OUT_OF_WORKSPACE`; the driver is not locked and the
action server is not contacted.

The defaults are conservative and are a software guard only. Measure the real
cell before a hardware run. The main node reads `workspace_x_min/max`,
`workspace_y_min/max`, `workspace_z_min/max` and `workspace_r_min/max` from
`config/depalletizing.yaml`; library users can pass a `WorkspaceLimits` object
directly to `DobotActionRobot`. `is_pose_within_workspace(pose)` can be used
for a pre-check. Malformed bounds (non-finite values, reversed or missing
limits) are rejected.

## 11. Simulation Without Hardware

`fake_ptp_action_server.py` provides a ROS action server with the same
`PointToPoint` interface as the real driver. It publishes interpolated
feedback, honours cancellation, and returns `achieved_pose`.

```bash
ros2 run depalletizing_pick_place fake_ptp_action_server
```

Run it with a `MultiThreadedExecutor` (as `main()` does) so cancel requests
can be processed while a movement is simulated. The server name, feedback
interval and movement duration can be overridden with the `action_name`,
`feedback_interval` and `movement_duration` node parameters.

The fake server replaces the PTP action server only. A gripper service is
still required by the main node; either run the real gripper node or provide a
matching `dobot_msgs/srv/GripperControl` service.

## 12. Integration

- `PickPlace` (`pick_place.py`) chains `move_to()` and gripper calls and stops
  on the first failure, cancellation, or timeout.
- `DepalletizingMainNode` (`main_node.py`) owns the driver, the gripper, and
  the workflow, and refuses to start while `is_faulted` is `True` or the
  server/gripper is unavailable.

## 13. Build and Test

Run from the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select depalletizing_pick_place --symlink-install
source install/setup.bash
```

```bash
colcon test --packages-select depalletizing_pick_place
colcon test-result --verbose
```

The driver tests (`test/test_robot_driver.py`) drive the real driver through a
fake action client and cover success, rejection, failure, cancellation,
timeout with cancellation, late goal acceptance, unavailable server,
overlapping commands, workspace limits, shared error codes, and fault
locking/recovery. `test/test_fake_ptp_action_server.py` exercises the simulated
action server's feedback, cancellation and overlap handling. These tests run
without a physical robot; the full-ROS tests require the ROS 2 environment.

Error codes are defined once in
`depalletizing_interfaces/depalletizing_interfaces/errors.py`. Import the
`ERROR_*` constants instead of writing string literals so producers and
consumers stay in sync.
