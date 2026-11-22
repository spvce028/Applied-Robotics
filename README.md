# Applied-Robotics

Repo for Applied Robotics - DOBOT Magician.

## Build Docker container

```bash
cd path/to/Applied-Robotics
docker build -t ros2_depalletizing:latest .
```

The image already contains a built workspace, including the pinned
`magician_ros2` dependency (`dobot_msgs` and the real robot driver).

## Run container

```bash
docker run -it --rm ros2_depalletizing:latest
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

## Run the node

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run depalletizing_pick_place depalletizing_node \
  --ros-args --params-file ~/ros2_ws/src/depalletizing_pick_place/config/depalletizing.yaml
```

Without the params file the node exits with a `Missing required parameters`
error; that is intentional. On success it logs that it is in `IDLE` state and
waits for `/confirm_homing` followed by `/start_depalletizing`. It does **not**
start a workflow automatically.

To simulate the robot action server without hardware:

```bash
ros2 run depalletizing_pick_place fake_ptp_action_server
```

The fake server replaces only the PTP action server; a `GripperControl`
service is still required by the main node.

## Build and test

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select depalletizing_pick_place depalletizing_interfaces
colcon test-result --verbose
```
