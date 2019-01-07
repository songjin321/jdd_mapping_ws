### JD Competition mapping module src

## Install

```
cd jdd_mapping_ws
sudo apt-get install -y python-wstool python-rosdep ninja-build
src/cartographer/scripts/install_proto3.sh
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make_isolated --install --use-ninja
```

## Run
```
source install_isolated/setup.bash
roslaunch cartographer_ros demo_my_robot.launch bag_filename:=test1.bag
rosservice call /finish_trajectory 0
```

