/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NVBLOX_ROS__NVBLOX_NODE_HPP_
#define NVBLOX_ROS__NVBLOX_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <nvblox/nvblox.h>
#include <tf2_eigen/tf2_eigen.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/service_server.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "nvblox_ros/conversions.hpp"
#include "nvblox_ros/transformer.hpp"

namespace nvblox {

class NvbloxNode {
 public:
  NvbloxNode(ros::NodeHandle& nodeHandle);

  // Check saving path.
  bool manageSaveDirectory();

  // Create path recursively.
  bool makePath(const std::string& path);

  // Check whether directory exits
  bool doesDirectoryExist(const std::string& path);

  // Setup the ros communication
  void setupRosCommunication();

  // Read mapping parameters
  bool readMappingParameters();

  // Read parameters
  bool readParameters();

  // Callback functions. These just stick images in a queue.
  void depthImageCallback(
      const sensor_msgs::ImageConstPtr& depth_img_ptr,
      const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  void colorImageCallback(
      const sensor_msgs::ImageConstPtr& color_img_ptr,
      const sensor_msgs::CameraInfoConstPtr& color_info_msg);

  bool savePly(std_srvs::Empty::Request& request,
               std_srvs::Empty::Response& response);

  // Does whatever processing there is to be done, depending on what
  // transforms are available.
  void processDepthQueue();
  void processColorQueue();

  // Alternative callbacks to using TF.
  void transformCallback(
      const geometry_msgs::TransformStampedConstPtr& transform_msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& transform_msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

  // Check memory status on GPU and save/purge data if necessary.
  void memoryTimerCallback(const ros::TimerEvent& event);

  // Timer for checking the available memory
  ros::Timer memoryTimer_;

  // Storage of the available memory after the previous save.
  int freeMemoryAfterLastSave{0};

  // Map saving counter. We are naming accordingly.
  unsigned int mapSaveCounter_{1u};

 private:
  // Helper functions to make the code more readable.
  void updateEsdf(const ros::Time& timestamp);
  void updateMesh(const ros::Time& timestamp);

  // A copied function from nvblox utilities. Gets the free memory in GPU.
  std::pair<int, float> getFreeGPUMemory();

  // ROS publishers and subscribers

  // Transformer to handle... everything, let's be honest.
  Transformer transformer_;

  // Syncronization of mesages ExactTime or ApproximateTime. Since in this
  // version only a single queue is maintained
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    sensor_msgs::CameraInfo>
      image_pair_sync_pol;

  // Naively initiated subscribers. TODO(ugly and inefficient, monotonize)
  std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>>
      depthSync1_;
  std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>> rgbSync1_;

  std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>>
      depthSync2_;
  std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>> rgbSync2_;

  std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>>
      depthSync3_;
  std::unique_ptr<message_filters::Synchronizer<image_pair_sync_pol>> rgbSync3_;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub1_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub1_;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub2_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub2_;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub3_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_camera_info_sub3_;

  message_filters::Subscriber<sensor_msgs::Image> color_sub1_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> color_camera_info_sub1_;

  message_filters::Subscriber<sensor_msgs::Image> color_sub2_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> color_camera_info_sub2_;

  message_filters::Subscriber<sensor_msgs::Image> color_sub3_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> color_camera_info_sub3_;

  // Ros hangle
  ros::NodeHandle nodeHandle_;

  ros::Subscriber transform_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher mesh_publisher_;
  ros::Publisher pointcloud_publisher_;
  ros::Publisher map_slice_publisher_;
  ros::ServiceServer save_ply_service_;

  // ROS & nvblox settings
  float voxel_size_ = 0.05f;
  bool esdf_ = true;
  bool esdf_2d_ = true;
  bool distance_slice_ = true;
  bool mesh_ = true;
  float slice_height_ = 1.0f;

  // Used for ESDF slicing. Everything between min and max height will be
  // compressed to a single 2D level, output at slice_height_.
  float min_height_ = 0.0f;
  float max_height_ = 1.0f;

  // ROS settings & update throttles
  std::string global_frame_ = "map";
  /// Pose frame to use if using transform topics.
  std::string pose_frame_ = "base_link";

  float max_tsdf_update_hz_ = 10.0f;
  float max_color_update_hz_ = 5.0f;
  float max_mesh_update_hz_ = 5.0f;
  float max_esdf_update_hz_ = 2.0f;

  int memoryCheckingPeriod_{5};
  bool useHelperFrame_{false};

  //! Threshold to purge mesh in cache.
  int memoryPurgeThreshold_{1000};

  std::string depth_image_1_topic =
      "/point_cloud_colorizer_ros/depth_image_camera_1";
  std::string depth_image_2_topic =
      "/point_cloud_colorizer_ros/depth_image_camera_2";
  std::string depth_image_3_topic =
      "/point_cloud_colorizer_ros/depth_image_camera_3";

  std::string rgb_image_1_topic =
      "/alphasense_driver_ros/cam3/dropped/debayered/slow";
  std::string rgb_image_2_topic =
      "/alphasense_driver_ros/cam4/dropped/debayered/slow";
  std::string rgb_image_3_topic =
      "/alphasense_driver_ros/cam5/dropped/debayered/slow";

  std::string camera_info_1_topic = "/camera_utils/alphasense_cam3/cameraInfo";
  std::string camera_info_2_topic = "/camera_utils/alphasense_cam4/cameraInfo";
  std::string camera_info_3_topic = "/camera_utils/alphasense_cam5/cameraInfo";

  // Mapper
  // Holds the map layers and their associated integrators
  // - TsdfLayer, ColorLayer, EsdfLayer, MeshLayer
  std::unique_ptr<RgbdMapper> mapper_;

  // The most important part: the ROS converter. Just holds buffers as state.
  RosConverter converter_;

  // Caches for GPU images
  ColorImage color_image_;
  DepthImage depth_image_;

  // Output directory
  std::string output_dir_{""};

  // State for integrators running at various speeds.
  ros::Time last_tsdf_update_time_;
  ros::Time last_color_update_time_;
  ros::Time last_esdf_update_time_;
  ros::Time last_mesh_update_time_;

  // Cache the last known number of subscribers.
  std::size_t mesh_subscriber_count_{0u};

  // Image queues.
  std::deque<
      std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr>>
      depth_image_queue_;
  std::deque<
      std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr>>
      color_image_queue_;
};

}  // namespace nvblox

#endif  // NVBLOX_ROS__NVBLOX_NODE_HPP_
