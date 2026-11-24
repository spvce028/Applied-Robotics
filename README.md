# Applied-Robotics

Repo for Applied Robotics - DOBOT Magician.

## Current status

Milestone 2 (Basic Functionality and Simulation) was completed in the lab:
the robot picks an object at a measured position and places it at a predefined
position. The `m2_lab_test` tool supports measuring poses, testing individual
operations and running the full demo. Robot and gripper simulation servers are
included for exercising the workflow without hardware. They simulate ROS
interfaces and motion feedback; they do not model collisions or physical grasps.

Milestone 3 (Oriented Placement into a Container) extends the hardware-free workflow with different pick and place orientations and raised transport waypoints. The default simulation configuration includes an intermediate alignment waypoint for rotating the object before placement.

The oriented placement workflow has been verified using ROS integration tests and a manual simulation run. Physical validation of the M3 route remains pending. The measured hardware configuration remains unchanged.

See the [pick-and-place package README](ros2_ws/src/depalletizing_pick_place/README.md)
for services, configuration, simulation and lab-tool usage, and the
[interfaces README](ros2_ws/src/depalletizing_interfaces/README.md) for shared
models, asynchronous workflows and mocks.

The shell examples below use Bash in Linux, WSL or the Docker container.
The ROS commands require a built and sourced ROS 2 Humble workspace.

## Build Docker container

```bash
cd path/to/Applied-Robotics
docker build -t ros2_depalletizing:latest .
```

The image already contains a built workspace, including the pinned
`magician_ros2` dependency (`dobot_msgs` and the real robot driver).

## Run container

```bash
docker run -it --rm --name depalletizing_dev ros2_depalletizing:latest
```

Do **not** bind-mount `./ros2_ws` over `/home/ros/ros2_ws`: that hides the
`magician_ros2` sources and the prebuilt `install/` space from the image.

For live development, mount only the two team packages and rebuild inside the
container:

```bash
docker run -it --rm \
  -v "$PWD/ros2_ws/src/depalletizing_pick_place:/home/ros/ros2_ws/src/depalletizing_pick_place" \
  -v "$PWD/ros2_ws/src/depalletizing_interfaces:/home/ros/ros2_ws/src/depalletizing_interfaces" \
  ros2_depalletizing:latest
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For a real robot, also add `--network host` and the serial device, e.g.
`--device=/dev/ttyUSB0`.

The container and main node do not automatically start the hardware bringup.
Start the DOBOT action server and gripper service separately for hardware use.

## Run the node

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run depalletizing_pick_place depalletizing_node \
  --ros-args --params-file ~/ros2_ws/src/depalletizing_pick_place/config/depalletizing.yaml
```

Without the required parameters the node exits with a `Missing required parameters`
error; that is intentional. On success it logs that it is in `IDLE` state and
waits for `/confirm_homing` followed by `/start_depalletizing`. It does **not**
start a workflow automatically.

The launch file loads the installed YAML automatically:

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py
```

The configuration contains measured lab poses. Re-measure them when the
physical setup changes; see `m2_lab_test measure` in the package README.


## Simulation without hardware

### Local workspace setup

For local development in Linux or WSL, run from the repository root:

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The separate Dobot workspace sourcing step is only required when the
external dependencies are installed in a separate workspace.

When using Docker, the image already contains the built workspace.
Open additional terminals in the named container with:

```bash
docker exec -it depalletizing_dev bash
```

Use separate sourced terminals in the same ROS environment.

### M3 oriented placement simulation

Terminal 1: start the main node and both packaged fake servers.

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py simulation:=true
```

This loads `config/simulation.yaml`, which contains the M3 example poses:

- Pick orientation: 15 degrees.
- Place orientation: -60 degrees.
- Three raised transport waypoints at Z = 150 mm.
- An intermediate alignment waypoint before container approach.

Measured hardware poses remain in `config/depalletizing.yaml`.

Terminal 2: source the same ROS environment and workspace, then confirm
simulated homing and explicitly start the workflow.

```bash
ros2 service call /confirm_homing std_srvs/srv/Trigger '{}'
ros2 service call /start_depalletizing std_srvs/srv/Trigger '{}'
```

Homing confirmation does not command movement. The workflow does not
start automatically, and start requests before homing confirmation
are rejected.

A successful workflow ends with the following main-node log:

```text
Operation completed: Pick-and-place completed successfully
```

Use the direct service calls above to execute the configured M3 route.

### Optional RViz visualization

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py simulation:=true use_rviz:=true
```

Use one launch command, not both concurrently. RViz shows the robot
description; in simulation its joint sliders are independent of the
Cartesian fake action server.

### Optional M2 interactive demo

The existing M2 lab tool remains available:

```bash
ros2 run depalletizing_pick_place m2_lab_test demo
```

The M2 demo can override the configured transport route and should
not be used to verify the default M3 alignment sequence.

### Hardware separation and limitations

Use fake servers instead of hardware servers on the same endpoints.

Hardware mode is the default (`simulation:=false`); it does not start
fake servers or hardware bringup. Start the real Dobot bringup
separately and perform actual homing before confirming it.

Override the YAML with `config:=PATH` when necessary.

Simulation verifies ROS interfaces, command sequencing, orientation
handling and workflow behavior. It does not verify physical collision
clearance, grasp stability or container insertion.

The latest M2 verification and remaining physical-test evidence are
documented in the [M2 completion report](M2_COMPLETION.md).

See the [pick-and-place package README](ros2_ws/src/depalletizing_pick_place/README.md)
for the detailed M3 route, configuration and service documentation.

## Build and test

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select depalletizing_pick_place depalletizing_interfaces
colcon test-result --verbose
```
