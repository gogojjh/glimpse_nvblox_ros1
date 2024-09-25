#ifndef NVBLOX_ROS_MONITOR_HELPER_H_
#define NVBLOX_ROS_MONITOR_HELPER_H_

#include <map>

#include <ros/ros.h>
#include <nvblox_ros/StatusInfo.h>

class MonitorHelper {
 public:
  explicit MonitorHelper(ros::NodeHandle node_handle);

  std::string checkPerformance(std::string program_name);

 private:
  ros::ServiceClient check_performance_srv_;
  nvblox_ros::StatusInfo srv_msg_;
};

#endif  // NVBLOX_ROS_MONITOR_HELPER_H_
