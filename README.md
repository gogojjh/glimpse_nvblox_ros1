# 1. Installation
## Installation Instructions
Clone this repository with recursive option since you will also need  **[submodule] nvblox** library.

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone --recurse-submodules git@github.com:gogojjh/glimpse_nvblox_ros1.git
```

Following the instructions in the [Nvblox](nvblox/README.md) to build the **Nvblox** dependencies as well. This library expects NVCC version > 11.2. Next, building the ROS interface is straight forward:
```
catkin build nvblox_ros nvblox_msgs nvblox_rviz_plugin
```

# 2. Usage
Check this [website](https://github.com/gogojjh/cobra) for the usage