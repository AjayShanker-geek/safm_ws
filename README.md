# SAFM Workspace

## Install some dependencies:
```bash
sudo apt-get install ros-foxy-px4-msgs ros-foxy-mavros-msgs
sudo apt-get install ros-noetic-px4-msgs ros-noetic-mavros ros-noetic-mavros-extras
```

```bash
cd safm_ws
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash

# Run these commands if you didn't install mavros_msgs and px4_msgs using apt-get in Foxy
colcon build --packages-select px4_msgs mavros_msgs
source install/local_setup.bash

source /opt/ros/noetic/setup.bash
colcon build --packages-select conversion_pkg ros1_bridge
```

## To run with ros1_bridge:
First terminal:
```bash
source /opt/ros/noetic/setup.bash
```

Second terminal:
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source ~/safm_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```