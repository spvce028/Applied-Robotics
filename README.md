# Applied-Robotics
Rep for Applied Robotics - DOBOT Magician

# Build Dockercontainer
cd path/to/Applied-Robotics
docker build -t ros2_depalletizing:latest .

# Run container
docker run -it --rm -v ./ros2_ws:/home/ros/ros2_ws ros2_depalletizing:latest

# Build and test packages inside container terminal
cd ~/ros2_ws
colcon build --symlink-install

# Source and run node
source install/setup.bash
ros2 run depalletizing_pick_place depalletizing_node

Verification: You should see the node start up and execute the pick-and-place workflow logic using the drivers.

