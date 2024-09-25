#include "nvblox_ros/monitor_helper.h"

MonitorHelper::MonitorHelper(ros::NodeHandle node_handle) {
  check_performance_srv_ =
      node_handle.serviceClient<nvblox_ros::StatusInfo>("/check_performance");
}

std::string MonitorHelper::checkPerformance(std::string program_name) {
  srv_msg_.request.program = program_name;
  check_performance_srv_.call(srv_msg_);
  return srv_msg_.response.status;
}
