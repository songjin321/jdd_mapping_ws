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
# use test1.bag create map
roslaunch cartographer_ros assets_writer_pcd.launch bag_filenames:=/home/neousys/Data/jdd/test1.bag pose_graph_filename:=/home/neousys/Data/jdd/test1_best.pbstream
```

