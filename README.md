# Applied-Robotics

Repo for Applied Robotics - DOBOT Magician.

## Current status

Milestone 2 (Basic Functionality and Simulation) was completed in the lab:
the robot picks an object at a measured position and places it at a predefined
position. The `m2_lab_test` tool supports measuring poses, testing individual
operations and running the full demo. Robot and gripper simulation servers are
included for exercising the workflow without hardware. They simulate ROS
interfaces and motion feedback; they do not model collisions or physical grasps.

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

Use separate sourced terminals in the same ROS environment. When using the
named container above, open additional shells with
`docker exec -it depalletizing_dev bash`.

Terminal 1: simulated robot action server.

```bash
ros2 run depalletizing_pick_place fake_ptp_action_server
```

Terminal 2: simulated gripper service.

```bash
ros2 run depalletizing_pick_place fake_gripper_service
```

Terminal 3: configured main node.

```bash
ros2 launch depalletizing_pick_place depalletizing.launch.py
```

Terminal 4: interactive demo, after reviewing the displayed movement plan.

```bash
ros2 run depalletizing_pick_place m2_lab_test demo
```

Use these fake servers instead of hardware servers on the same endpoints.
The demo records a homing confirmation; on real hardware, perform actual homing
before confirming. The launch file does not yet select simulation/hardware mode
or start a visualization automatically.

## Build and test

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select depalletizing_pick_place depalletizing_interfaces
colcon test-result --verbose
```
