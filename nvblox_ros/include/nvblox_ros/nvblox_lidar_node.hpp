//
// Created by kevinlad on 23-1-5.
//

#ifndef NVBLOX_ROS_INCLUDE_NVBLOX_ROS_NVBLOX_LIDAR_NODE_H_
#define NVBLOX_ROS_INCLUDE_NVBLOX_ROS_NVBLOX_LIDAR_NODE_H_

#include <cv_bridge/cv_bridge.h>

#include <nvblox/nvblox.h>
#include <tf2_eigen/tf2_eigen.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_srvs/Empty.h>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include "conversions.hpp"
#include "nvblox/core/cuda/image_operation.h"
#include "nvblox/core/cuda/warmup.h"
#include "nvblox_ros/monitor_helper.h"
#include "nvblox_ros/rosbag_helper.h"
#include "transformer.hpp"

namespace nvblox {

constexpr float kDefaultUintScaleFactor = 1000.0f;
constexpr float kDefaultUintDepthScaleOffset = 0.0f;
constexpr float kDefaultUintHeightScaleOffset = 10.0f;

class NvbloxLidarNode {
 public:
  NvbloxLidarNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void SetupRosCommunication();

  bool ReadMappingParameters();

  bool ReadParameters();

  // ROS callback functions
  void LidarImageCallback(
      const sensor_msgs::ImageConstPtr& depth_img_msg,
      const sensor_msgs::ImageConstPtr& height_img_msg,
      const sensor_msgs::CameraInfoConstPtr& lidar_info_msg);

  /*
  void SemanticImageCallback(
      const sensor_msgs::ImageConstPtr& semantic_img_msg,
      const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

  void ColorImageCallback(
      const sensor_msgs::ImageConstPtr& color_img_msg,
      const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  */

  void SemanticImageCallback(
      const sensor_msgs::ImageConstPtr& semantic_img_msg);
  void SemanticUCTImageCallback(
      const sensor_msgs::ImageConstPtr& semantic_uct_img_msg);

  void ColorImageCallback(const sensor_msgs::ImageConstPtr& color_img_msg);

  void CameraInfoCallback(
      const sensor_msgs::CameraInfoConstPtr& semantic_cam_info_msg,
      const sensor_msgs::CameraInfoConstPtr& color_cam_info_msg);

  void OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

  bool SaveMapServiceCallback(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& rep);

  // Check memory status on GPU and save/purge data if necessary.
  void memoryTimerCallback(const ros::TimerEvent& event);

  // Timer for checking the available memory
  ros::Timer memoryTimer_;

  // Storage of the available memory after the previous save.
  int freeMemoryAfterLastSave{0};

  // Main programs of processing data
  void ProcessLidarDataQueue();

  void ProcessSemanticImage(const DepthImage& depth_img,
                            const std::shared_ptr<OSLidar>& lidar_ptr);

  void ProcessColorImage();

  std::pair<int, float> GetFreeGPUMemory();

  void SaveMesh();

  std::vector<std::vector<float>> v_status_;

 private:
  void UpdateEsdf(const ros::Time& timestamp);
  void UpdateMesh(const ros::Time& timestamp);

 private:
  std::shared_ptr<Transformer> transformer_ptr_ = nullptr;
  std::shared_ptr<RosConverter> converter_ptr_ = nullptr;
  std::unique_ptr<nvblox::RgbdMapper> mapper_ptr_ = nullptr;

  // thread
  std::mutex mutex_lidar_data_;     // mutex for ROS lidar data input
  std::mutex mutex_semantic_data_;  // mutex for ROS semantic data input
  std::mutex mutex_color_data_;     // mutex for ROS image data input
  std::mutex mutex_integration_;    // mutex for TSDF, Mesh, ESDF integration

  ros::NodeHandle nh_, pnh_;

  // ****************** ROS Subscriber and Publisher
  // clang-format off
	///// Define ROS subscriber
  // subscribe sensor data
  using LidarImageSyncPol = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
  using LidarImageSync = message_filters::Synchronizer<LidarImageSyncPol>;
  std::unique_ptr<LidarImageSync> lidar_image_sync_;

	using LiDARImgTuple = std::tuple<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfoConstPtr>;
	std::queue<LiDARImgTuple> lidar_img_info_queue_;
  message_filters::Subscriber<sensor_msgs::Image> lidar_depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> lidar_height_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> lidar_camera_info_sub_;

  // using SemanticImageSyncPol = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
  // using SemanticImageSync = message_filters::Synchronizer<SemanticImageSyncPol>;
  // std::unique_ptr<SemanticImageSync> semantic_image_sync_;
	using ImgTuple = std::tuple<sensor_msgs::ImageConstPtr, sensor_msgs::CameraInfo>;
  std::queue<ImgTuple> semantic_img_info_queue_;
  std::queue<ImgTuple> color_img_info_queue_;

  ros::Subscriber semantic_image_sub_;
  ros::Subscriber semantic_uct_image_sub_;
  ros::Subscriber color_image_sub_;	

  ros::Subscriber semantic_camera_info_sub_;
  ros::Subscriber color_camera_info_sub_;

  sensor_msgs::CameraInfo semantic_camera_info_;
  sensor_msgs::CameraInfo color_camera_info_;

  // subscribe odometry data
  ros::Subscriber odom_sub_;
  std::queue<nav_msgs::OdometryConstPtr> odom_queue_;

	///// Define ROS publisher
	ros::Publisher mesh_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher map_slice_pub_;

  ///// Define ROS ServiceServer
  ros::ServiceServer save_map_service_sub_;

  bool save_mesh_map_call_ = false;
  bool save_tsdf_lowdis_call_ = false;
  bool save_tsdf_map_call_ = false;
  bool save_esdf_map_call_ = false;
  // clang-format on

  // ******************  ROS & nvblox settings
  float voxel_size_ = 0.05f;
  bool color_integration_ = false;
  bool semantic_integration_ = false;
  bool esdf_ = true;
  bool esdf_2d_ = true;
  bool distance_slice_ = true;
  bool mesh_ = true;
  bool performance_monitor_ = false;
  float slice_height_ = 1.0f;

  int semantic_sensor_source_ = 0;
  int dataset_type_ = 0;
  int tsdf_voxel_update_method_;
  bool ray_tracing_enabled_;
  bool bayesian_semantics_enabled_;

  // Used for ESDF slicing. Everything between min and max height will be
  // compressed to a single 2D level, output at slice_height_.
  float min_height_ = 0.0f;
  float max_height_ = 1.0f;

  std::string global_frame_ = "world";
  /// Pose frame to use if using transform topics.
  std::string pose_frame_ = "body_imu";

  float max_tsdf_update_hz_ = 10.0f;
  float max_color_update_hz_ = 5.0f;
  float max_mesh_update_hz_ = 5.0f;
  float max_esdf_update_hz_ = 2.0f;
  float max_lidar_update_hz_ = 5.0f;
  float max_mesh_update_time_ = 5.0f;

  // ******************
  int memoryCheckingPeriod_{5};
  bool useHelperFrame_{false};

  //! Threshold to purge mesh in cache.
  int memoryPurgeThreshold_{1000};

  // Map saving counter. We are naming accordingly.
  unsigned int mapSaveCounter_{1u};

  // Output directory
  std::string output_dir_{"/tmp"};

  size_t frame_number_ = 0;

  ros::Time last_odom_update_time_ = ros::Time(0.0f);
  ros::Time last_esdf_update_time_ = ros::Time(0.0f);
  ros::Time last_mesh_update_time_ = ros::Time(0.0f);
  ros::Time last_semantic_update_time_ = ros::Time(0.0f);

  int mesh_subscriber_count_ = 1;

  // Transform
  Transform T_BaseFrame_PoseFrame_;

  RosbagHelper rosbag_helper_;
  MonitorHelper monitor_helper_;

  double normal_comp_time_, metric_mapping_time_, semantic_mapping_time_;
};

}  // namespace nvblox
#endif  // NVBLOX_ROS_INCLUDE_NVBLOX_ROS_NVBLOX_LIDAR_NODE_H_
