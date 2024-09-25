# Cobra-ROS-Mapping

# 1. Installation
## Installation Instructions
Clone this repository with recursive option since you will also need  **[submodule] nvblox** library.

```
mkdir -p ~/workspaces/src && cd ~/workspaces/src
git clone --recurse-submodules http://gitlab.ram-lab.com/ramlab_dataset_sensor/mapping_codebase/glimpse_nvblox_ros1.git
```

Following the instructions in the [Nvblox](nvblox/README.md) to build the **Nvblox** dependencies as well. This library expects NVCC version > 11.2. Next, building the ROS interface is straight forward:
```
catkin build nvblox_ros nvblox_msgs nvblox_rviz_plugin
```

# 2. Usage
## i. [FusionPortable](https://ram-lab.com/file/site/fusionportable/dataset/fusionportable) Dataset 
```
roslaunch nvblox_ros nvblox_lidar_ros_fusionportable.launch
```
## ii. [SemanticKITTI]() Dataset 
```
roslaunch nvblox_ros nvblox_lidar_ros_semantickitti.launch
```
## iii. [KITTI-360]() Dataset 
```
roslaunch nvblox_ros nvblox_lidar_ros_kitti360.launch
```

This node expects syncronized lidar information matrices with the converted depth and height images. Current state is adjusted for Glimpse team.

