#ifndef NVBLOX_ROS_ROSBAG_HELPER_H_
#define NVBLOX_ROS_ROSBAG_HELPER_H_

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class RosbagHelper {
 public:
  explicit RosbagHelper(ros::NodeHandle node_handle);

  void pauseRosbag();
  void playRosbag();

 private:
  ros::ServiceClient rosbag_pause_srv_;
  std_srvs::SetBool srv_msg_;
};

#endif  // NVBLOX_ROS_ROSBAG_HELPER_H_
