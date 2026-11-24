# Milestone 2 Completion Report

Verification date: 23 November 2026.

## Outcome

The team reported successful hardware execution using `m2_lab_test`: an object
was picked from a known position and placed at a predefined position. Measured
poses are committed in `config/depalletizing.yaml`.

The remaining software verification has now been completed using ROS 2 Humble
in Ubuntu 22.04 / WSL. Both team packages build, and the full package test run
reports **222 tests, 0 errors, 0 failures, 1 skipped**. The skip is the existing
copyright-header template check; no functional test is skipped.

This report covers the working-tree changes based on commit `e535ff2`, not a
claim that those changes have already been committed or published.

## Verification Results

| Package | Tests | Errors | Failures | Skipped |
|---|---:|---:|---:|---:|
| `depalletizing_interfaces` | 43 | 0 | 0 | 1 |
| `depalletizing_pick_place` | 179 | 0 | 0 | 0 |
| **Total** | **222** | **0** | **0** | **1** |

Results were read from the generated `build/<package>/pytest.xml` files and
`colcon test-result --verbose`. The test tooling emits a non-failing
`SelectableGroups` deprecation warning.

### Completed Technical Follow-up

- The ROS dry run now uses the packaged `FakePTPActionServer` **and**
  `FakeGripperService`, together with the real robot and gripper clients.
- Two consecutive complete cycles cover transport, gripper order, final pose,
  feedback and execution-slot release.
- Real ROS integration covers gripper failure, timeout cleanup, retry and late
  replies. A timed-out command cannot complete a newer client request.
- The fake action server uses a reentrant callback group so cancellation can be
  processed during simulated execution. An integration test verifies this path.
- Launch smoke tests exercise both simulation and hardware-client modes in
  isolated ROS domains. Neither mode starts motion automatically, and a start
  without homing confirmation is rejected. Only simulation starts fake servers.
- The simulation launch uses a separate `simulation.yaml`; measured hardware
  coordinates remain unchanged.
- Interfaces style findings were corrected without changing model/mock behavior.
  The Flake8 check is explicitly scoped to its own package.
- `~/lab_tools` was checked in the available WSL environment and is absent.
  Other team machines were not inspected.

## Starting the System

After building and sourcing the workspace:

```bash
# Complete simulation, without hardware or GUI.
ros2 launch depalletizing_pick_place depalletizing.launch.py simulation:=true

# Optional robot-description visualization; use instead of the command above.
ros2 launch depalletizing_pick_place depalletizing.launch.py simulation:=true use_rviz:=true

# Hardware-client mode; start the real DOBOT bringup separately.
ros2 launch depalletizing_pick_place depalletizing.launch.py simulation:=false
```

`config:=/absolute/path/to/config.yaml` overrides the configuration in either
mode. Hardware mode does not launch fake servers or command automatic homing.

## Reproducing the Verification

The verification used the Windows checkout through its WSL-mounted path, with
external DOBOT dependencies sourced from the existing `~/dobot_ros2_ws/install`.
The following commands can be run from the repository root in the same setup:

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select depalletizing_interfaces depalletizing_pick_place
colcon test-result --verbose
```

Building from `ros2_ws` instead is also supported; run build, source and test
consistently from that directory. Generated build/install/log directories are
not source artifacts. The Docker build was not re-executed for this verification.

## Boundaries of the Evidence

- The hardware success is the team's reported lab result. The number of
  physical attempts and success rate were not supplied, so none are invented
  here. Add the actual run count or video link if needed for course assessment.
- No new physical movement, homing, grasp or emergency-stop test was performed
  during this software verification.
- RViz is wired through `dobot_description`; its four-axis gripper URDF was
  generated and parsed successfully. GUI rendering was not visually checked.
  Simulation joint sliders are independent of the Cartesian fake action server;
  the visualization is not a collision/physical-grasp simulator.
- A service timeout or cancellation acknowledgement does not prove physical
  standstill. In particular, a delayed gripper command can still take effect;
  ignoring its late response only protects newer software requests.
- Further container-fit, clearance and rotation validation belongs to the next
  milestone's hardware work.

The functional M2 objective and its software follow-up are complete on this
evidence. Remaining physical run statistics and optional visual inspection
are documentation/assessment follow-up, not fabricated test results.
