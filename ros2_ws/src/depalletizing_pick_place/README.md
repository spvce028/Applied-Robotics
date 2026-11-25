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
| Feedback | Latest `current_pose` is available in `last_feedback` while a goal is active |

## 2. Requirements

- ROS 2 (Humble) with `rclpy`
- `dobot_msgs` (`PointToPoint` action) and `action_msgs`
- The `depalletizing_interfaces` package (`RobotPose`, `OperationResult`)
- A robot action server and gripper service, either hardware drivers or the
  included simulation nodes

## 3. Constructing the Driver

The following defaults belong to the Python driver constructor. The main node
requires explicit motion settings through ROS parameters and passes them to
the driver; it does not use these constructor defaults as fallback configuration.

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
- If a movement can never reach a terminal result, recovery requires verifying
  the robot's physical state. A node restart alone does not stop robot motion.

## 8. Feedback

While a movement is running the server publishes its current pose. The driver
stores the most recent value for the active operation and logs changes at debug
level. `last_feedback` returns `None` once that operation is released, so collect
feedback during motion if it is needed after completion.

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
feedback, exposes a cancellation callback, and returns `achieved_pose`.

```bash
ros2 run depalletizing_pick_place fake_ptp_action_server
```

Run it with a `MultiThreadedExecutor` (as `main()` does). The server name, feedback
interval and movement duration can be overridden with the `action_name`,
`feedback_interval` and `movement_duration` node parameters.

The server uses a reentrant callback group so cancel requests can be handled
while the simulated execute loop is active. ROS integration tests exercise
cancellation during a movement as well as successful complete runs.

For a complete hardware-free setup, run the included gripper service in a
second sourced terminal:

```bash
ros2 run depalletizing_pick_place fake_gripper_service
```

It provides the same `dobot_msgs/srv/GripperControl` interface as the real
gripper on `/dobot_gripper_service`. Its `mode` parameter supports `success`,
`fail` and `no_response`. The latter delays the reply for 30 seconds to exercise
client timeouts and late replies. For example:

```bash
ros2 param set /fake_gripper_service mode fail
ros2 param set /fake_gripper_service mode success
```


Start the configured main node in another sourced terminal:

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py \
  simulation:=false \
  config:="$(ros2 pkg prefix --share depalletizing_pick_place)/config/simulation.yaml"
```

The commands above show separate-process startup with manually started
fake servers and the simulation configuration.

### M3: Oriented placement simulation

Alternatively, start the complete hardware-free system with a single
launch command. Do not also start the fake servers manually.

For a local workspace with the Dobot dependencies installed separately,
run from the repository root:

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

If the external dependencies are built in the same workspace, the
separate Dobot workspace sourcing step is not required.

In Terminal 1:

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py simulation:=true
```

In Terminal 2, source the same ROS environment and workspace.
Verify the loaded orientation and transport parameters:

```bash
ros2 param get /depalletizing_main_node pick_r
ros2 param get /depalletizing_main_node place_r
ros2 param get /depalletizing_main_node transport_waypoints
```

The default M3 simulation route is:

| Position | X (mm) | Y (mm) | Z (mm) | R (deg) |
|---|---:|---:|---:|---:|
| Pick | 200 | 0 | 50 | 15 |
| Raised waypoint | 200 | 0 | 150 | 15 |
| Alignment waypoint | 200 | 0 | 150 | -60 |
| Transport waypoint | 250 | 100 | 150 | -60 |
| Place | 250 | 100 | 50 | -60 |

All positions are expressed in the robot base frame (`base_link`).
The object is lifted before rotating to the target orientation.
The placement approach, lowering and retraction preserve the target
orientation.

Confirm simulated homing and explicitly start the workflow:

```bash
ros2 service call /confirm_homing std_srvs/srv/Trigger '{}'
ros2 service call /start_depalletizing std_srvs/srv/Trigger '{}'
```

Homing confirmation does not command movement. The start response
acknowledges acceptance, not successful completion.

A successful cycle ends with the main-node log:

```text
Operation completed: Pick-and-place completed successfully
```

Use the direct service calls for the M3 simulation. The M2 lab demo
tool can override the configured transport route.

### Simulation and hardware separation

Simulation mode uses `config/simulation.yaml` and starts the main node,
the fake PTP action server and the fake gripper service.

Default hardware mode (`simulation:=false`) uses the measured
`config/depalletizing.yaml` and starts only the main node.
Hardware bringup remains external. The `config:=PATH` argument
overrides the YAML in either mode.

After startup, no motion occurs until homing is confirmed and an
explicit command is received.

The M3 simulation poses are examples for fake servers only.
They must not replace the measured hardware configuration.
Simulation validates command sequencing, orientation handling and
workflow behavior, but does not validate physical clearance,
collision avoidance, grasp stability or container insertion.

`use_rviz:=true` optionally includes `dobot_description`'s display launch
for the four-axis gripper model. In simulation its joint GUI visualizes
manually selected model configurations; it is not synchronized to the
Cartesian fake server and does not provide collision or grasp simulation.
The headless launch is covered by automated tests; GUI rendering has
not been visually verified.

## 12. Integration

- `PickPlace` (`pick_place.py`) chains `move_to()` and gripper calls and stops
  on the first failure, cancellation, or timeout.
- `DepalletizingMainNode` (`main_node.py`) owns the driver, the gripper, and
  the workflow. Commands are rejected before homing confirmation, during
  shutdown, while another operation/cancel dispatch is active, while the robot
  is busy or faulted, or when the robot server/gripper service is unavailable.

### Configuration and services

Start with the configuration loaded:

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py
```

Required parameters are `pick_x/y/z/r`, `place_x/y/z/r`, `velocity_ratio`,
`acceleration_ratio`, `movement_timeout`, `approach_height`,
`gripper_service_name`, `gripper_timeout` and `keep_compressor_running`.
The shorthand `pick_x/y/z/r` means four separate names: `pick_x`, `pick_y`,
`pick_z`, `pick_r`; the same applies to place. Missing or invalid values abort
initialization. The compressor setting must be chosen for the actual lab setup.
Workspace limits and transport waypoints are optional; their configuration is
described below.

Pick/place poses and transport waypoints are read for each workflow start.
Driver settings, gripper settings, workspace limits and approach height are
applied when components are constructed; restart the node after changing them.
Parameter updates do not automatically reconfigure those component instances.

| Trigger service | Purpose |
|---|---|
| `/confirm_homing` | Record prior manual homing; performs no movement |
| `/start_depalletizing` | Accept the complete pick/transport/place workflow |
| `/manual/pick` | Pick at the configured source pose |
| `/manual/place` | Place at the configured target pose |
| `/manual/open_gripper` | Open the gripper |
| `/manual/close_gripper` | Close the gripper |
| `/cancel_depalletizing` | Request cancellation and prevent subsequent steps |
| `/reset_fault` | Reset an idle faulted driver without forcing its lock open |

All services use `std_srvs/srv/Trigger`. For example, after actual homing on
hardware (or simulated confirmation when using the fake servers):

```bash
ros2 service call /confirm_homing std_srvs/srv/Trigger '{}'
ros2 service call /start_depalletizing std_srvs/srv/Trigger '{}'
```

Manual commands and full workflows share one execution slot. Start/manual
responses acknowledge acceptance; final success or failure is logged by the
main node. A cancel response acknowledges a request, not confirmed standstill.
Reset is refused during active operations, never uses `force=True`, and clears
the previous homing confirmation when recovering a fault. After successful
manual closing, the holding state conservatively means an object may be held;
it is not a sensor-based grasp verification.

## 13. Build and Test

Run from the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

This builds the source dependencies as well. Ensure the pinned external
dependency from the repository's `dependencies.repos` has been imported and its
requirements installed, or use the Docker image which performs those steps.

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

## Optional transport waypoints

The main node runs `pick -> transport waypoints -> place`. Configure
`transport_waypoints` in `config/depalletizing.yaml` as a flat numeric list,
with four values per pose: `[x, y, z, r, x, y, z, r, ...]`. Positions use
millimetres and rotation uses degrees in `base_link`.

```yaml
transport_waypoints: []  # No additional transport stage
```

A non-empty route is executed sequentially with `MOVL`. All poses are checked
for finite values and against the configured workspace limits before picking.
The route is snapshotted at each start, so a parameter update does not alter an
active sequence. Pick/place manual services do not execute transport waypoints.

The transport executor uses the same cancellation signal as pick and place.
Failure, timeout or cancellation stops the remaining route and prevents place;
the gripper is not automatically opened. Configure and verify the route for the
actual cell: workspace bounds do not perform collision checking.

## Controlled shutdown (Ctrl+C / SIGTERM)

The main entry point handles SIGINT and SIGTERM without shutting down the ROS
context first. It disables new operations, sets the shared cancellation event,
and requests cancellation of an active robot goal. The executor continues
processing responses for up to five seconds before ROS entities are destroyed.

Shutdown waits for both workflow completion and the driver's active-movement
lock to clear. A timeout or lost ROS context produces a warning that standstill
is not confirmed; it never force-resets the driver. A pending gripper service
request cannot be cancelled by the robot action cancellation, and shutdown does
not automatically open the gripper. Previously installed Python signal handlers
are restored when the entry point exits.

## Hardware-free integration checks

From a built and sourced ROS workspace:

```bash
python3 -m pytest \
  src/depalletizing_pick_place/test/test_config_contract.py \
  src/depalletizing_pick_place/test/test_ros_dry_run.py -v -rs
```

The configuration checks compare required main-node parameters with the shipped
`depalletizing.yaml` and check that the launch file names that configuration.
The lab pose measurements are stored in that file; there is no separate
`lab_params.yaml` in the repository.

The ROS dry run uses `FakePTPActionServer`, the actual robot and gripper clients,
pick/place primitives, the waypoint executor and the workflow/main node.
Unique remapped endpoints isolate it from hardware services. It checks startup
without motion, rejection before homing and two full consecutive runs including
transport, feedback and gripper command ordering.

### Oriented-placement integration (Milestone 3)

The ROS integration fixture reads `simulation.yaml`, keeping automated tests
independent of measured hardware coordinates. Its observers subclass the
packaged fake servers and delegate to their real handlers; the production
clients, waypoint executor, pick/place primitives and workflow are not replaced.
Observers record the accepted movement goals and completed gripper commands
in one ordered log.

The oriented-placement scenario uses a pick angle of `15` degrees and a place
angle of `-60` degrees. The explicit route first raises the object to `z=150`
with the pick orientation, then changes orientation at fixed x/y/z, and finally
transports it to the destination while preserving the place angle. Place
approach, lowering and retraction all retain that target orientation.

`test_m3_oriented_placement_two_complete_cycles` checks the exact command log
for two consecutive cycles, including positions, angles, motion types, gripper
order, final pose and released holding state. Concurrent workflow/manual
requests are rejected while the oriented cycle is active.

`test_m3_cancel_during_alignment_prevents_placement` issues a real action
cancellation after the rotation goal reaches the fake server. It waits until
the workflow ends and the driver releases its active-goal lock, then verifies
that no later transport or place goal was received and the simulated gripper
remains closed.

Run the integration checks from the built and sourced ROS workspace:

```bash
python3 -m pytest src/depalletizing_pick_place/test/test_ros_dry_run.py -v -rs
```

Verification on 24 November 2026: **7 integration tests passed, none skipped**,
including both oriented-placement scenarios. The existing workflow supports
the route through configuration; no additional production rotation logic was
needed. These results establish software sequencing only. Physical grip
stability, swept clearance, container fit and the M3 hardware acceptance must
still be verified in the lab.

The gripper endpoint is the packaged `FakeGripperService`; no test-only service
implementation replaces it. Additional tests cover service failure, timeout
recovery, late replies and robot cancellation during an active action. These
checks do not verify a physical grasp or actual standstill.

The fake gripper exposes received commands through `requests` and simulated jaw
state through `is_closed`. `service_name`, `response_delay` (default 0.2 s) and
`no_response_delay` (default 30 s) are configurable. A timeout does not undo an
already sent physical command; a late response is ignored by the client but
the underlying operation can still have taken effect.

To reuse the fake in a ROS test, instantiate it directly and spin it in a
`MultiThreadedExecutor` so delayed replies do not block other callbacks:

```python
gripper = FakeGripperService(
    service_name='/test/gripper', response_delay=0.01, no_response_delay=0.3,
)
executor.add_node(gripper)
gripper.set_parameters([Parameter('mode', value='fail')])
assert gripper.requests == [('close', False)]  # every received command, in order
```

`mode`, `response_delay` and `no_response_delay` are read per request, so they
can be changed between commands. `service_name` is only read at construction.
Observers can subclass the fake and extend `handle_request`, as
`ObservedGripperService` in `test/test_ros_dry_run.py` does.

The gripper client now releases `_pending` and destroys its timer on timeout.
Responses and timer callbacks are associated with their originating command,
so a late reply cannot complete or overwrite a newer command. Regression tests
in `test/test_gripper_client.py` cover timeout cleanup and ignored late replies.

## Interactive lab tool and milestone validation

`m2_lab_test` provides three modes:

```bash
# With robot bringup running: read pick/place poses and offer to save them.
ros2 run depalletizing_pick_place m2_lab_test measure

# With the configured main node running: test operations individually.
ros2 run depalletizing_pick_place m2_lab_test run

# With the configured main node running: execute the complete demo.
ros2 run depalletizing_pick_place m2_lab_test demo
```

`measure` reads `/dobot_pose_raw`, converts position values from metres to
millimetres and updates the YAML only after confirmation. It keeps a backup in
`~/.ros/depalletizing.yaml.bak`. Run it from the workspace root to update the
source config, or supply `--config PATH`. Restart the main node afterward so
the measured configuration is loaded.

`run` steps through homing confirmation, gripper commands, manual pick/place
and a complete workflow. `demo` prints the movement plan, configures a raised
transport midpoint, opens the gripper and starts the complete sequence. It
clears the transport waypoint parameter afterward. Completion is reported from
the main node's operation logs. In `run` and `demo`, Ctrl+C during an operation
attempts to request cancellation through `/cancel_depalletizing`.

Homing must have been performed manually before confirming it; the tool's
confirmation does not execute or measure homing. The computed transport
midpoint is bounded by `workspace_z_max`, but does not perform collision
planning. Review the displayed path against the actual setup.

Milestone 2 (Basic Functionality and Simulation) was completed in the lab using
this tool: the robot picked up an object at a known pose and placed it at a
predefined pose. The same workflow can be exercised without hardware using
the included robot action server and gripper service. Re-measure poses when
the physical setup changes; the committed lab coordinates describe the tested
setup rather than a universal robot configuration.
