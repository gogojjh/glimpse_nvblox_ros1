//
// Created by kevinlad on 23-1-5.
//
#include <signal.h>

#include "nvblox/io/mesh_io.h"
#include "nvblox/io/pointcloud_io.h"
#include "nvblox/utils/timing.h"
#include "nvblox_ros/nvblox_lidar_node.hpp"

// #define DEBUG

namespace nvblox {

NvbloxLidarNode::NvbloxLidarNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), rosbag_helper_(nh), monitor_helper_(nh) {
  // Initialize the transformer and converter
  transformer_ptr_ = std::make_shared<Transformer>(pnh_);
  converter_ptr_ = std::make_shared<RosConverter>();

  if (!ReadParameters()) {
    ROS_ERROR("Error reading parameters for Nvblox. Program will exit.");
    return;
  }

  SetupRosCommunication();

  // Initialize the mapper
  mapper_ptr_ = std::make_unique<RgbdMapper>(voxel_size_);

  // Read mapping, integration related parameters.
  if (!ReadMappingParameters()) {
    ROS_ERROR(
        "Error reading mapping parameters for Nvblox. Program will exit.");
    return;
  }
  LOG(INFO) << "[DEBUG] voxel_size: " << voxel_size_;
  LOG(INFO) << "[DEBUG] tsdf_voxel_update_method: "
            << tsdf_voxel_update_method_;
  LOG(INFO) << "[DEBUG] dataset_type: " << dataset_type_;
  LOG(INFO) << "[DEBUG] ray_tracing_enabled: " << ray_tracing_enabled_;
  LOG(INFO) << "[DEBUG] bayesian_semantics_enabled: "
            << bayesian_semantics_enabled_;

  std::tie(freeMemoryAfterLastSave, std::ignore) = GetFreeGPUMemory();

  transformer_ptr_->set_global_frame(global_frame_);
  transformer_ptr_->set_pose_frame(pose_frame_);

  // NOTE (gogojjh): Add the tf between the baselink and the SLAM frame
  // if (!transformer_ptr_->lookupTransformTf(
  //         "r3live_bodyimu", "baselink", ros::Time::now(),
  //         &T_BaseFrame_PoseFrame_))
  //         {
  //   LOG(WARNING) << "No transform from bodyimu to baselink found. Stamp = "
  //                << ros::Time::now();
  //   T_BaseFrame_PoseFrame_.setIdentity();
  // }
  // Eigen::Quaternionf Qbp(1.0, 0.0, 0.0, 0.0);
  // Eigen::Vector3f tbp(2.0, 0.0, 1.7);
  Eigen::Quaternionf Qbp(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3f tbp(0.0, 0.0, 0.0);
  Eigen::Matrix4f Tbp = Eigen::Matrix4f::Identity();
  Tbp.block<3, 3>(0, 0) = Qbp.toRotationMatrix();
  Tbp.block<3, 1>(0, 3) = tbp;
  T_BaseFrame_PoseFrame_ = Transform(Tbp);
  LOG(INFO) << "T_BaseFrame_PoseFrame_: (" << T_BaseFrame_PoseFrame_(0, 3)
            << ", " << T_BaseFrame_PoseFrame_(1, 3) << ", "
            << T_BaseFrame_PoseFrame_(2, 3) << ")";

  v_status_.reserve(100000);
}

std::pair<int, float> NvbloxLidarNode::GetFreeGPUMemory() {
  size_t free_bytes;
  size_t total_bytes;
  cudaMemGetInfo(&free_bytes, &total_bytes);
  const int free_mb = free_bytes / 1e6;
  const int total_mb = total_bytes / 1e6;
  const float free_percent =
      static_cast<float>(free_mb) * 100.0f / static_cast<float>(total_mb);
  return {free_mb, free_percent};
}

bool NvbloxLidarNode::ReadParameters() {
  bool success{true};
  // Ros param reading.
  success &= pnh_.param<float>("voxel_size", voxel_size_, 0.05f);
  success &= pnh_.param<std::string>("global_frame", global_frame_, "world");
  success &= pnh_.param<std::string>("pose_frame", pose_frame_, "body_imu");

  success &=
      pnh_.param<int>("memory_checking_period", memoryCheckingPeriod_, 5);
  success &=
      pnh_.param<int>("memory_purge_threshold", memoryPurgeThreshold_, 750);

  success &=
      pnh_.param<bool>("semantic_integration", semantic_integration_, false);
  success &= pnh_.param<bool>("color_integration", color_integration_, false);
  success &=
      pnh_.param<bool>("performance_monitor", performance_monitor_, false);
  if (success) {
    LOG(INFO) << "performance_monitor: " << performance_monitor_;
  }
  success &= pnh_.param<bool>("mesh", mesh_, true);
  success &= pnh_.param<bool>("esdf", esdf_, true);
  success &= pnh_.param<bool>("esdf_2d", esdf_2d_, true);
  success &= pnh_.param<bool>("distance_slice", distance_slice_, true);
  success &= pnh_.param<float>("slice_height", slice_height_, 1.0f);
  success &= pnh_.param<float>("min_height", min_height_, 0.0f);
  success &= pnh_.param<float>("max_height", max_height_, 1.0f);

  success &=
      pnh_.param<float>("max_tsdf_update_hz", max_tsdf_update_hz_, 10.0f);
  if (success) {
    DLOG(INFO) << "max_tsdf_update_hz: " << max_tsdf_update_hz_;
  }

  success &=
      pnh_.param<float>("max_color_update_hz", max_color_update_hz_, 5.0f);
  success &= pnh_.param<float>("max_mesh_update_hz", max_mesh_update_hz_, 5.0f);
  success &= pnh_.param<float>("max_esdf_update_hz", max_esdf_update_hz_, 2.0f);
  success &=
      pnh_.param<float>("max_mesh_update_time", max_mesh_update_time_, 5.0f);
  if (success) {
    LOG(INFO) << "max_mesh_update_time: " << max_mesh_update_time_;
  }

  int maxMeshingHeight{4};
  success &= pnh_.param<int>("max_meshing_height", maxMeshingHeight, 4);

  success &=
      pnh_.param<int>("semantic_sensor_source", semantic_sensor_source_, 0);

  CHECK_NOTNULL(converter_ptr_);
  converter_ptr_->maxHeight_ = maxMeshingHeight;
  return success;
}

bool NvbloxLidarNode::ReadMappingParameters() {
  bool success{true};

  // clang-format off
  // Default parameters
  mapper_ptr_->lidar_tsdf_integrator()
      .view_calculator().raycast_subsampling_factor(4);

  mapper_ptr_->semantic_integrator()
      .view_calculator().raycast_subsampling_factor(4);

  mapper_ptr_->color_integrator()
      .view_calculator().raycast_subsampling_factor(4);

  // ************************** lidar_tsdf_integrator
  float tsdf_max_integration_distance_m_local;
  success &= pnh_.param<float>("tsdf_integrator_max_integration_distance_m",
                              tsdf_max_integration_distance_m_local, 10.0f);
  mapper_ptr_->lidar_tsdf_integrator().max_integration_distance_m(tsdf_max_integration_distance_m_local);

  float tsdf_truncation_distance_vox_local;
  success &= pnh_.param<float>("tsdf_integrator_truncation_distance_vox",
                              tsdf_truncation_distance_vox_local, 4.0f);
  mapper_ptr_->lidar_tsdf_integrator().truncation_distance_vox(tsdf_truncation_distance_vox_local);

  float tsdf_max_weight_local;
  success &= pnh_.param<float>("tsdf_integrator_max_weight",
                              tsdf_max_weight_local, 100.0f);
  mapper_ptr_->lidar_tsdf_integrator().max_weight(tsdf_max_weight_local);

  success &= pnh_.param<int>("tsdf_voxel_update_method",
                            tsdf_voxel_update_method_, 6);
  mapper_ptr_->lidar_tsdf_integrator().voxel_weight_method(tsdf_voxel_update_method_);

  success &= pnh_.param<bool>("ray_tracing_enabled",
                            ray_tracing_enabled_, true);
  mapper_ptr_->lidar_tsdf_integrator().view_calculator().ray_tracing_enabled(ray_tracing_enabled_);
 
  // ************************** semantic_integrator
  float semantic_max_integration_distance_m_local;
  success &= pnh_.param<float>("semantic_integrator_max_integration_distance_m",
                              semantic_max_integration_distance_m_local, 100.0f);
  mapper_ptr_->semantic_integrator().max_integration_distance_m(semantic_max_integration_distance_m_local);

  float semantic_truncation_distance_vox_local;
  success &= pnh_.param<float>("semantic_integrator_truncation_distance_vox",
                              semantic_truncation_distance_vox_local, 1.0f);
  mapper_ptr_->semantic_integrator().truncation_distance_vox(semantic_truncation_distance_vox_local);

  float semantic_max_weight_local;
  success &= pnh_.param<float>("semantic_integrator_max_weight", semantic_max_weight_local, 100.0f);
  mapper_ptr_->semantic_integrator().max_weight(semantic_max_weight_local);

  success &= pnh_.param<int>("dataset_type", dataset_type_, 0);
  mapper_ptr_->semantic_integrator().dataset_type(dataset_type_); 

  mapper_ptr_->semantic_integrator().view_calculator().ray_tracing_enabled(ray_tracing_enabled_);

  success &= pnh_.param<bool>("bayesian_semantics_enabled", bayesian_semantics_enabled_, true);
  mapper_ptr_->semantic_integrator().bayesian_semantics_enabled(bayesian_semantics_enabled_);

  // ************************** color_integrator
  float color_max_integration_distance_m_local;
  success &= pnh_.param<float>("color_integrator_max_integration_distance_m",
                              color_max_integration_distance_m_local, 10.0f);
  mapper_ptr_->color_integrator().max_integration_distance_m(color_max_integration_distance_m_local);

  float color_truncation_distance_vox_local;
  success &= pnh_.param<float>("color_integrator_truncation_distance_vox",
                              color_truncation_distance_vox_local, 10.0f);
  mapper_ptr_->color_integrator().truncation_distance_vox(color_truncation_distance_vox_local);

  float color_max_weight_local;
  success &= pnh_.param<float>("color_integrator_max_weight", color_max_weight_local, 10.0f);
  mapper_ptr_->color_integrator().max_weight(color_max_weight_local);  

  // ************************** mesh_integrator
  float mesh_min_weight_local;
  success &= pnh_.param<float>("mesh_integrator_min_weight", mesh_min_weight_local, 1e-4);
  mapper_ptr_->mesh_integrator().min_weight(mesh_min_weight_local);

  bool mesh_weld_vertices;
  success &= pnh_.param<bool>("mesh_integrator_weld_vertices", mesh_weld_vertices, false);
  mapper_ptr_->mesh_integrator().weld_vertices(mesh_weld_vertices);

  // ************************** esdf_integrator
  float esdf_min_weight;
  success &= pnh_.param<float>("esdf_integrator_min_weight", esdf_min_weight, 1e-4);
  mapper_ptr_->esdf_integrator().min_weight(esdf_min_weight);

  float max_site_distance_vox;
  success &= pnh_.param<float>("esdf_integrator_max_site_distance_vox",
                              max_site_distance_vox, 1.0f);
  mapper_ptr_->esdf_integrator().max_site_distance_vox(max_site_distance_vox);

  float esdf_max_distance_m = mapper_ptr_->esdf_integrator().max_distance_m();
  success &= pnh_.param<float>("esdf_integrator_max_distance_m",
                              esdf_max_distance_m, 10.0f);

  // **************************
  success &= pnh_.param<std::string>("output_dir", output_dir_, "");

  // clang-format on
  return success;
}

void NvbloxLidarNode::SetupRosCommunication() {
  // Queue size is important, since we have to wait synced messages.
  constexpr int kQueueSize = 20;

  // The timer for checking the avilable free memory in GPU.
  // memoryTimer_ =
  //     nh_.createTimer(ros::Duration(static_cast<double>(memoryCheckingPeriod_)),
  //                     &NvbloxLidarNode::memoryTimerCallback, this);

  // Subscriber
  // clang-format off
  lidar_depth_image_sub_.subscribe(nh_, "depth_image", kQueueSize);
  lidar_height_image_sub_.subscribe(nh_, "height_image", kQueueSize);
  lidar_camera_info_sub_.subscribe(nh_, "lidar_camera_info", kQueueSize);
  lidar_image_sync_ = std::make_unique<LidarImageSync>(
    LidarImageSyncPol(kQueueSize), lidar_depth_image_sub_, lidar_height_image_sub_, lidar_camera_info_sub_);
  lidar_image_sync_->registerCallback(boost::bind(&NvbloxLidarNode::LidarImageCallback, this, _1, _2, _3));

  if (semantic_integration_ ) {
    semantic_image_sub_ = nh_.subscribe<sensor_msgs::Image>("semantic_image", kQueueSize, &NvbloxLidarNode::SemanticImageCallback, this);
    semantic_uct_image_sub_ = nh_.subscribe<sensor_msgs::Image>("semantic_uct_image", kQueueSize, &NvbloxLidarNode::SemanticUCTImageCallback, this);
    semantic_camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("semantic_camera_info", kQueueSize, 
      boost::bind(&NvbloxLidarNode::CameraInfoCallback, this, _1, sensor_msgs::CameraInfoConstPtr()));
  }

  if (color_integration_) {
    color_image_sub_ = nh_.subscribe<sensor_msgs::Image>("color_image", kQueueSize, &NvbloxLidarNode::ColorImageCallback, this);
    color_camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("color_camera_info", kQueueSize, 
      boost::bind(&NvbloxLidarNode::CameraInfoCallback, this, sensor_msgs::CameraInfoConstPtr(), _1));
  }

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometry", kQueueSize, &NvbloxLidarNode::OdomCallback, this);

  // Publisher
  mesh_pub_ = nh_.advertise<nvblox_msgs::Mesh>("mesh", 1);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
  map_slice_pub_ = nh_.advertise<nvblox_msgs::DistanceMapSlice>("map_slice", 1, false);

  // ServiceServer
  save_map_service_sub_ = nh_.advertiseService("save_map", &NvbloxLidarNode::SaveMapServiceCallback, this);

  // clang-format on
}

void NvbloxLidarNode::OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  transformer_ptr_->odomCallback(odom_msg);
  last_odom_update_time_ = odom_msg->header.stamp;
}

void NvbloxLidarNode::LidarImageCallback(
    const sensor_msgs::ImageConstPtr& lidar_depth_img_msg,
    const sensor_msgs::ImageConstPtr& lidar_height_img_msg,
    const sensor_msgs::CameraInfoConstPtr& lidar_info_msg) {
  DLOG(INFO) << "LidarImageCallback";
  mutex_lidar_data_.lock();
  while (lidar_img_info_queue_.size() > 10) lidar_img_info_queue_.pop();
  lidar_img_info_queue_.push(std::make_tuple(
      lidar_depth_img_msg, lidar_height_img_msg, lidar_info_msg));
  mutex_lidar_data_.unlock();
  DLOG(INFO) << "LidarImageCallback finished";
}

void NvbloxLidarNode::SemanticImageCallback(
    const sensor_msgs::ImageConstPtr& semantic_img_msg) {
  DLOG(INFO) << "SemanticCallback";
  mutex_semantic_data_.lock();
  while (semantic_img_info_queue_.size() > 10) semantic_img_info_queue_.pop();
  if ((semantic_camera_info_.height != 0) &&
      (semantic_camera_info_.width != 0)) {
    semantic_img_info_queue_.push(
        std::make_tuple(semantic_img_msg, semantic_camera_info_));
  }
  mutex_semantic_data_.unlock();
  DLOG(INFO) << "SemanticCallback finished";
}

// TODO(gogojjh): process semantic_uct_image_msg
void NvbloxLidarNode::SemanticUCTImageCallback(
    const sensor_msgs::ImageConstPtr& semantic_uct_img_msg) {
  DLOG(INFO) << "SemanticUCTCallback";
  // mutex_semantic_data_.lock();
  // while (semantic_img_info_queue_.size() > 10)
  // semantic_img_info_queue_.pop(); if ((semantic_camera_info_.height != 0) &&
  //     (semantic_camera_info_.width != 0)) {
  //   semantic_img_info_queue_.push(
  //       std::make_tuple(semantic_img_msg, semantic_camera_info_));
  // }
  // mutex_semantic_data_.unlock();
  DLOG(INFO) << "SemanticUCTCallback finished";
}

void NvbloxLidarNode::ColorImageCallback(
    const sensor_msgs::ImageConstPtr& color_img_msg) {
  DLOG(INFO) << "ColorCallback";
  mutex_color_data_.lock();
  while (color_img_info_queue_.size() > 10) color_img_info_queue_.pop();
  if ((color_camera_info_.height != 0) && (color_camera_info_.width != 0)) {
    color_img_info_queue_.push(
        std::make_tuple(color_img_msg, color_camera_info_));
  }
  mutex_color_data_.unlock();
  DLOG(INFO) << "ColorCallback finished";
}

void NvbloxLidarNode::CameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& semantic_camera_info_msg,
    const sensor_msgs::CameraInfoConstPtr& color_camera_info_msg) {
  DLOG(INFO) << "CameraInfoCallback";
  if (semantic_camera_info_msg) {
    semantic_camera_info_ = *semantic_camera_info_msg;
  } else {
    color_camera_info_ = *color_camera_info_msg;
  }
  DLOG(INFO) << "CameraInfoCallback finished";
}

bool NvbloxLidarNode::SaveMapServiceCallback(std_srvs::Empty::Request& req,
                                             std_srvs::Empty::Response& rep) {
  LOG(INFO) << "Call SaveMapService ------------------------------";
  save_mesh_map_call_ = true;
  // save_tsdf_lowdis_call_ = true;
  // save_tsdf_map_call_ = true;
  // save_esdf_map_call_ = true;
  return true;
}

void NvbloxLidarNode::ProcessLidarDataQueue() {
  while (ros::ok()) {
    if (lidar_img_info_queue_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const auto& data_tuple = lidar_img_info_queue_.front();
    // To guarantee that the we already have TF
    if (std::get<0>(data_tuple)->header.stamp > last_odom_update_time_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    const ros::Time timestamp = std::get<0>(data_tuple)->header.stamp;
    const std::string sensor_frame = std::get<0>(data_tuple)->header.frame_id;

    // lookup TF
    Transform T_W_sensor;
    if (!transformer_ptr_->lookupTransformToGlobalFrame(sensor_frame, timestamp,
                                                        &T_W_sensor)) {
      // LOG(WARNING) << "No transform from " << global_frame_ << " to "
      //              << sensor_frame << " found. Stamp = " << timestamp;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      mutex_lidar_data_.lock();
      lidar_img_info_queue_.pop();
      mutex_lidar_data_.unlock();
      continue;
    }
    Transform T_BW_sensor = T_BaseFrame_PoseFrame_ * T_W_sensor;
    LOG_EVERY_N(INFO, 100) << "Position: (" << T_BW_sensor(0, 3) << ", "
                           << T_BW_sensor(1, 3) << ", " << T_BW_sensor(2, 3)
                           << ")";

    // convert ROS message into data
    mutex_lidar_data_.lock();
    sensor_msgs::ImageConstPtr lidar_depth_img_msg;
    sensor_msgs::ImageConstPtr lidar_height_img_msg;
    sensor_msgs::CameraInfoConstPtr lidar_info_msg;
    std::tie(lidar_depth_img_msg, lidar_height_img_msg, lidar_info_msg) =
        data_tuple;
    mutex_lidar_data_.unlock();

    DepthImage lidar_depth_img;
    HeightImage lidar_height_img;
    if (!converter_ptr_->depthImageFromImageMessage(
            lidar_depth_img_msg, &lidar_depth_img, kDefaultUintScaleFactor,
            kDefaultUintDepthScaleOffset) ||
        !converter_ptr_->heightImageFromImageMessage(
            lidar_height_img_msg, &lidar_height_img, kDefaultUintScaleFactor,
            kDefaultUintHeightScaleOffset)) {
      std::cerr << "Failed to transform lidar depth or height image."
                << std::endl;
      mutex_lidar_data_.lock();
      lidar_img_info_queue_.pop();
      mutex_lidar_data_.unlock();
      continue;
    }

    std::shared_ptr<OSLidar> lidar_ptr = std::make_shared<OSLidar>(
        static_cast<int>(lidar_info_msg->D[0]),
        static_cast<int>(lidar_info_msg->D[1]), lidar_info_msg->D[2],
        lidar_info_msg->D[3], lidar_info_msg->D[4], lidar_info_msg->D[5],
        lidar_info_msg->D[6], lidar_info_msg->D[7]);
    if (lidar_depth_img.rows() != lidar_ptr->num_elevation_divisions() ||
        lidar_depth_img.cols() != lidar_ptr->num_azimuth_divisions() ||
        lidar_height_img.rows() != lidar_ptr->num_elevation_divisions() ||
        lidar_height_img.cols() != lidar_ptr->num_azimuth_divisions()) {
      std::cerr << "Lidar intrinsics do not match with Lidar images"
                << std::endl;
      mutex_lidar_data_.lock();
      lidar_img_info_queue_.pop();
      mutex_lidar_data_.unlock();
      continue;
    }

    mutex_lidar_data_.lock();
    lidar_img_info_queue_.pop();
    mutex_lidar_data_.unlock();

    // TSDF integration
    {
      // set depth and height frame
      lidar_ptr->setDepthFrameCUDA(lidar_depth_img.dataPtr());
      lidar_ptr->setHeightFrameCUDA(lidar_height_img.dataPtr());

      std::chrono::time_point<std::chrono::system_clock> start_time;

      // compute the normal image
      start_time = std::chrono::system_clock::now();
      timing::Timer timer_normal("ros/nvblox_lidar_node/compute_normal_image");
      nvblox::cuda::getNormalImageOSLidar(*lidar_ptr);
      normal_comp_time_ =
          static_cast<double>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now() - start_time)
                  .count()) *
          1e-9;
      timer_normal.Stop();

      // integration
      timing::Timer timer_integrate("ros/nvblox_lidar_node/integrate_tsdf");
      start_time = std::chrono::system_clock::now();
      mapper_ptr_->integrateOSLidarDepth(lidar_depth_img, T_BW_sensor,
                                         *lidar_ptr);
      metric_mapping_time_ =
          static_cast<double>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now() - start_time)
                  .count()) *
          1e-9;
      timer_integrate.Stop();

      nvblox::cuda::freeNormalImageOSLidar(*lidar_ptr);
    }

    // Semantic integration
    if (semantic_integration_) {
      // semantics are from the camera
      if (semantic_sensor_source_ == 0) {
        std::chrono::time_point<std::chrono::system_clock> start_time =
            std::chrono::system_clock::now();
        ProcessSemanticImage(lidar_depth_img, nullptr);
        semantic_mapping_time_ =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now() - start_time)
                    .count()) *
            1e-9;
      }

      // semantics are from the lidar
      else if (semantic_sensor_source_ == 1) {
        std::chrono::time_point<std::chrono::system_clock> start_time =
            std::chrono::system_clock::now();
        ProcessSemanticImage(lidar_depth_img, lidar_ptr);
        semantic_mapping_time_ =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now() - start_time)
                    .count()) *
            1e-9;
      }
    }

    // Color integration
    if (color_integration_) {
      ProcessColorImage();
    }

#ifdef DEBUG
    save_tsdf_map_call_ = false;
    if (save_tsdf_map_call_) {
      nvblox::timing::Timer timer_write("ros/nvblox_lidar_node/tsdf/write");
      nvblox::io::outputVoxelLayerToPly(mapper_ptr_->tsdf_layer(),
                                        output_dir_ + "tsdf.ply");
      save_tsdf_map_call_ = false;
    }
#endif

    if (performance_monitor_) {
      std::cout << "before check performance" << std::endl;
      std::string status =
          monitor_helper_.checkPerformance(std::string("nvblox_lidar_node"));
      std::stringstream ss(status);
      float cpu_memory, gpu_memory;
      ss >> cpu_memory >> gpu_memory;
      size_t block_size = mapper_ptr_->tsdf_layer().getAllBlockIndices().size();
      std::cout << "Performance information: " << frame_number_ << " "
                << block_size << " " << cpu_memory << " "
                << normal_comp_time_ * 1e3 << " " << metric_mapping_time_ * 1e3
                << " " << semantic_mapping_time_ * 1e3 << std::endl;
      v_status_.push_back(
          {static_cast<float>(frame_number_),               // NOLINT
           static_cast<float>(block_size),                  // NOLINT
           cpu_memory,                                      // NOLINT
           gpu_memory,                                      // NOLINT
           static_cast<float>(normal_comp_time_ * 1e3),     // NOLINT
           static_cast<float>(metric_mapping_time_ * 1e3),  // NOLINT
           static_cast<float>(semantic_mapping_time_ * 1e3)});
    }

    // TODO(gogojjh): address the bugs in generating the zero-crossing
    // if (save_tsdf_lowdis_call_) {
    //   nvblox::timing::Timer timer_write(
    //       "ros/nvblox_lidar_node/tsdf/write_zero_crossing");
    //   nvblox::io::outputLowDistanceToPly(
    //       mapper_ptr_->tsdf_layer(), output_dir_ + "tsdf_zero_crossing.ply");
    //   save_tsdf_lowdis_call_ = false;
    // }

    // Mesh integration
    if (mesh_) {
      if (max_mesh_update_time_ > 0.0f &&
          (timestamp - last_mesh_update_time_).toSec() >=
              max_mesh_update_time_) {
        last_mesh_update_time_ = timestamp;

        LOG(INFO) << "UpdateMesh";
        rosbag_helper_.pauseRosbag();
        UpdateMesh(lidar_depth_img_msg->header.stamp);

#ifdef DEBUG
        if (save_mesh_map_call_) {
          nvblox::timing::Timer timer_write("ros/nvblox_lidar_node/mesh/write");
          nvblox::io::outputMeshLayerToPly(mapper_ptr_->mesh_layer(),
                                           output_dir_ + "mesh.ply",
                                           output_dir_ + "pts.ply");
          save_mesh_map_call_ = false;
        }
        nvblox::timing::Timer timer_write("ros/nvblox_lidar_node/mesh/write");
#endif

        nvblox::io::outputMeshLayerToPly(mapper_ptr_->mesh_layer(),
                                         "/tmp/mesh_nvblox.ply");
        LOG(INFO) << "Output PLY files to /tmp/mesh_nvblox.ply";
        rosbag_helper_.playRosbag();

#ifdef DEBUG
        nvblox::io::outputVoxelLayerToPly(mapper_ptr_->tsdf_layer(),
                                          "/tmp/tsdf_nvblox.ply");
#endif
      }
    }

    // ESDF integration
    if (esdf_) {
      // Check if it's been long enough since the last frame.
      if (max_esdf_update_hz_ <= 0.0f ||
          (timestamp - last_esdf_update_time_).toSec() >=
              1.0f / max_esdf_update_hz_) {
        last_esdf_update_time_ = timestamp;
        UpdateEsdf(lidar_depth_img_msg->header.stamp);
        if (save_esdf_map_call_) {
          nvblox::timing::Timer timer_write("ros/nvblox_lidar_node/esdf/write");
          nvblox::io::outputVoxelLayerToPly(mapper_ptr_->esdf_layer(),
                                            output_dir_ + "esdf.ply");
          save_esdf_map_call_ = false;
        }
      }
    }

    DLOG(INFO) << "Integrating frame " << frame_number_;
    frame_number_++;
  }
}

void NvbloxLidarNode::ProcessSemanticImage(
    const DepthImage& depth_img, const std::shared_ptr<OSLidar>& lidar_ptr) {
  if (semantic_img_info_queue_.empty()) return;

  while (!semantic_img_info_queue_.empty()) {
    const auto& data_tuple = semantic_img_info_queue_.front();
    const ros::Time timestamp = std::get<0>(data_tuple)->header.stamp;
    const std::string target_frame = std::get<0>(data_tuple)->header.frame_id;
    if (timestamp > last_odom_update_time_) {  // if the state estimator delays
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return;
    }
    DLOG(INFO) << target_frame << " " << timestamp << " "
               << last_odom_update_time_;

    Transform T_W_sensor;
    if (!transformer_ptr_->lookupTransformToGlobalFrame(target_frame, timestamp,
                                                        &T_W_sensor)) {
      LOG(WARNING) << "No transform from " << global_frame_ << " to "
                   << target_frame << " found. Stamp = " << timestamp;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return;
    }
    Transform T_BW_sensor = T_BaseFrame_PoseFrame_ * T_W_sensor;
    DLOG(INFO) << global_frame_ << " -> " << target_frame;
    DLOG(INFO) << T_BW_sensor.matrix();

    // Load semantic image
    mutex_semantic_data_.lock();
    sensor_msgs::ImageConstPtr semantic_img_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    std::tie(semantic_img_msg, camera_info_msg) = data_tuple;
    mutex_semantic_data_.unlock();

    SemanticImage semantic_img;
    if (!converter_ptr_->semanticImageFromImageMessage(semantic_img_msg,
                                                       &semantic_img)) {
      LOG(WARNING) << "Failed to transform semantic image.";
      mutex_semantic_data_.lock();
      semantic_img_info_queue_.pop();
      mutex_semantic_data_.unlock();
      break;
    }

    mutex_semantic_data_.lock();
    semantic_img_info_queue_.pop();
    mutex_semantic_data_.unlock();

    // clang-format off
    // with the lidar object, semantics are from the camera
    if (lidar_ptr) {
      // clang-format off
      if (semantic_img.rows() != lidar_ptr->num_elevation_divisions() ||
          semantic_img.cols() != lidar_ptr->num_azimuth_divisions()) {
        std::cerr << "Lidar intrinsics do not match with semantic Lidar images"
                  << std::endl;
        break;
      }
      timing::Timer timer_integrate("ros/nvblox_lidar_node/integrate_lidar_semantic");
      mapper_ptr_->integrateOSLidarSemantic(depth_img, semantic_img, T_BW_sensor, *lidar_ptr);
      timer_integrate.Stop();
    }
    // without the lidar object, semantics are from the camera
    else {
      // Vector5f D;
      // D << camera_info_msg.D[0], camera_info_msg.D[1], camera_info_msg.D[2], camera_info_msg.D[3], 0.0;
      Matrix3f K;
      K << camera_info_msg.K[0], camera_info_msg.K[1], camera_info_msg.K[2],
           camera_info_msg.K[3], camera_info_msg.K[4], camera_info_msg.K[5],
           camera_info_msg.K[6], camera_info_msg.K[7], camera_info_msg.K[8];
      Matrix3f R_rect;
      R_rect << camera_info_msg.R[0], camera_info_msg.R[1], camera_info_msg.R[2], 
                camera_info_msg.R[3], camera_info_msg.R[4], camera_info_msg.R[5], 
                camera_info_msg.R[6], camera_info_msg.R[7], camera_info_msg.R[8];
      Matrix3x4f P_rect;
      P_rect << camera_info_msg.P[0], camera_info_msg.P[1], camera_info_msg.P[2], camera_info_msg.P[3], 
                camera_info_msg.P[4], camera_info_msg.P[5], camera_info_msg.P[6], camera_info_msg.P[7], 
                camera_info_msg.P[8], camera_info_msg.P[9], camera_info_msg.P[10], camera_info_msg.P[11];
      const size_t &width = camera_info_msg.width;
      const size_t &height = camera_info_msg.height;
      if (semantic_img.rows() != height || semantic_img.cols() != width) {
        std::cerr << "Camera intrinsics do not match with semantic camera images"
                  << std::endl;
        break;
      }
      std::shared_ptr<CameraPinhole> camera_ptr;
      if (R_rect.diagonal().sum() == 3.0f) {
        camera_ptr = std::make_shared<CameraPinhole>(K, width, height);
      } else {
        camera_ptr = std::make_shared<CameraPinhole>(P_rect, R_rect, width, height);
      }
      timing::Timer timer_integrate("ros/nvblox_lidar_node/integrate_camera_semantic");
      mapper_ptr_->integrateCameraSemantic(semantic_img, T_BW_sensor, *camera_ptr);
      timer_integrate.Stop();
    }
    // clang-format on 
  }
}

void NvbloxLidarNode::ProcessColorImage() {
  if (color_img_info_queue_.empty()) return;

  while (!color_img_info_queue_.empty()) {
    const auto& data_tuple = color_img_info_queue_.front();
    const ros::Time timestamp = std::get<0>(data_tuple)->header.stamp;
    const std::string target_frame = std::get<0>(data_tuple)->header.frame_id;
    if (timestamp > last_odom_update_time_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      return;
    }

    Transform T_W_sensor;
    if (!transformer_ptr_->lookupTransformToGlobalFrame(target_frame, timestamp,
                                                        &T_W_sensor)) {
      // LOG(WARNING) << "No transform from " << global_frame_ << " to "
      //               << target_frame << " found. Stamp = " << timestamp;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      mutex_color_data_.lock();
      color_img_info_queue_.pop();
      mutex_color_data_.unlock();
      break;
    }
    Transform T_BW_sensor = T_BaseFrame_PoseFrame_ * T_W_sensor;
    DLOG(INFO) <<  T_BW_sensor.matrix();

    mutex_color_data_.lock();
    sensor_msgs::ImageConstPtr color_img_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    std::tie(color_img_msg, camera_info_msg) = data_tuple;
    mutex_color_data_.unlock();

    ColorImage color_img;
    if (!converter_ptr_->colorImageFromImageMessage(color_img_msg,
                                                    &color_img)) {
      LOG(WARNING) << "Failed to transform color image.";
      mutex_color_data_.lock();
      color_img_info_queue_.pop();
      mutex_color_data_.unlock();
      break;
    }

    mutex_color_data_.lock();
    color_img_info_queue_.pop();
    mutex_color_data_.unlock();

    // clang-format off
    Matrix3f R_rect;
    R_rect << camera_info_msg.R[0], camera_info_msg.R[1], camera_info_msg.R[2], 
              camera_info_msg.R[3], camera_info_msg.R[4], camera_info_msg.R[5], 
              camera_info_msg.R[6], camera_info_msg.R[7], camera_info_msg.R[8];
    Matrix3x4f P_rect;
    P_rect << camera_info_msg.P[0], camera_info_msg.P[1], camera_info_msg.P[2], camera_info_msg.P[3], 
              camera_info_msg.P[4], camera_info_msg.P[5], camera_info_msg.P[6], camera_info_msg.P[7], 
              camera_info_msg.P[8], camera_info_msg.P[9], camera_info_msg.P[10], camera_info_msg.P[11];
    const size_t &width = camera_info_msg.width;
    const size_t &height = camera_info_msg.height;
    if (color_img.rows() != height || color_img.cols() != width) {
      std::cerr << "Camera intrinsics do not match with color camera images"
                << std::endl;
      break;
    }    
    std::shared_ptr<CameraPinhole> camera_ptr = std::make_shared<CameraPinhole>(P_rect, R_rect, width, height);
    timing::Timer timer_integrate("ros/nvblox_lidar_node/integrate_color");
    mapper_ptr_->integrateColor(color_img, T_BW_sensor, *camera_ptr);
    timer_integrate.Stop();
    // clang-format on
  }
}

void NvbloxLidarNode::UpdateEsdf(const ros::Time& timestamp) {
  if (esdf_2d_) {
    mapper_ptr_->updateEsdfSlice(min_height_, max_height_, slice_height_);
  } else {
    mapper_ptr_->updateEsdf();
  }

  timing::Timer esdf_output_timer("ros/esdf/output");

  if (pointcloud_pub_.getNumSubscribers() > 0u) {
    timing::Timer output_pointcloud_timer("ros/esdf/output/pointcloud");

    // Output the ESDF. Let's just do the full thing for now.
    sensor_msgs::PointCloud2 pointcloud_msg;

    // AABB of a certain slice height.
    AxisAlignedBoundingBox aabb(Vector3f(std::numeric_limits<float>::lowest(),
                                         std::numeric_limits<float>::lowest(),
                                         slice_height_ - voxel_size_ / 2.0f),
                                Vector3f(std::numeric_limits<float>::max(),
                                         std::numeric_limits<float>::max(),
                                         slice_height_ + voxel_size_ / 2.0f));

    converter_ptr_->pointcloudFromLayerInAABB(mapper_ptr_->esdf_layer(), aabb,
                                              &pointcloud_msg);

    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = timestamp;
    pointcloud_pub_.publish(pointcloud_msg);

    output_pointcloud_timer.Stop();
  }

  // Also publish the map slice.
  if (distance_slice_ && map_slice_pub_.getNumSubscribers() > 0u) {
    timing::Timer output_map_slice_timer("ros/esdf/output/map_slice");

    nvblox_msgs::DistanceMapSlice map_slice;

    converter_ptr_->distanceMapSliceFromLayer(mapper_ptr_->esdf_layer(),
                                              slice_height_, &map_slice);
    map_slice.header.frame_id = global_frame_;
    map_slice.header.stamp = timestamp;
    map_slice_pub_.publish(map_slice);
  }
}

void NvbloxLidarNode::UpdateMesh(const ros::Time& timestamp) {
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/update");
  const std::vector<Index3D> mesh_updated_list = mapper_ptr_->updateMesh();
  mesh_integration_timer.Stop();

  // Convert mesh data to the ROS messhage and publish
  timing::Timer mesh_output_timer("ros/mesh/output");
  size_t new_subscriber_count = mesh_pub_.getNumSubscribers();
  if (new_subscriber_count > 0u) {
    nvblox_msgs::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      ROS_INFO("Got a new subscriber, sending entire map.");

      converter_ptr_->meshMessageFromMeshBlocks(
          mapper_ptr_->mesh_layer(),
          mapper_ptr_->mesh_layer().getAllBlockIndices(), &mesh_msg);
      mesh_msg.clear = true;
    } else {
      converter_ptr_->meshMessageFromMeshBlocks(mapper_ptr_->mesh_layer(),
                                                mesh_updated_list, &mesh_msg);
    }
    mesh_msg.header.frame_id = global_frame_;
    mesh_msg.header.stamp = timestamp;
    mesh_pub_.publish(mesh_msg);
  }
  mesh_subscriber_count_ = new_subscriber_count;
  mesh_output_timer.Stop();
}

void NvbloxLidarNode::SaveMesh() {
  UpdateMesh(ros::Time::now());
  nvblox::io::outputMeshLayerToPly(mapper_ptr_->mesh_layer(),
                                   "/tmp/mesh_nvblox.ply");
  LOG(INFO) << "Output PLY files to /tmp/mesh_nvblox.ply";
}

// NOTE(gogojjh): not used
void NvbloxLidarNode::memoryTimerCallback(const ros::TimerEvent& /*event*/) {
  int start_free_gpu_memory_mb{0};
  std::tie(start_free_gpu_memory_mb, std::ignore) = GetFreeGPUMemory();
  LOG(INFO) << "Timed Memory check. Free memory: " << start_free_gpu_memory_mb;
  // Update the free memory holder if we have more available memory.
  freeMemoryAfterLastSave = (start_free_gpu_memory_mb > freeMemoryAfterLastSave)
                                ? start_free_gpu_memory_mb
                                : freeMemoryAfterLastSave;

  // Check whether we have a big memory diff.
  if (freeMemoryAfterLastSave - start_free_gpu_memory_mb >
      memoryPurgeThreshold_) {
    LOG(INFO) << "Remining memory: "
              << freeMemoryAfterLastSave - start_free_gpu_memory_mb << "MB";
    // Generate mesh save path.
    // const std::string nameOfFile{"mesh_nvblox_" +
    //                              std::to_string(mapSaveCounter_) +
    //                              ".ply"};
    // const std::string fileName{output_dir_ + "/" + nameOfFile};
    const std::string filename = "/tmp/mesh_nvblox.ply";
    bool success =
        io::outputMeshLayerToPly(mapper_ptr_->mesh_layer(), filename);
    if (!success) {
      LOG(INFO) << "Couldn't Save Mesh File.!";
      return;
    }
    // Purge the mesh at cache,
    mapper_ptr_->mesh_layer().clear();
    mapper_ptr_->esdf_layer().clear();
    // Update Saved Map Counter
    mapSaveCounter_ += 1u;
    // Re-query memory
    std::tie(freeMemoryAfterLastSave, std::ignore) = GetFreeGPUMemory();
    // ROS_INFO_STREAM("Nvblox: Saved a local mesh file named "
    //                 << nameOfFile << ". Free Space from "
    //                 << start_free_gpu_memory_mb << " MB to "
    //                 << freeMemoryAfterLastSave << " MB.");
  }
}

}  // namespace nvblox

void SigintCallback(int sig) {
  LOG(INFO) << "Press ctrl-c";

  // Print computation time
  std::vector<std::string> keywords = {std::string("")};
  LOG(INFO) << nvblox::timing::Timing::Print(keywords);

  std::ofstream time_outfile("/tmp/time_nvblox.txt");
  time_outfile << nvblox::timing::Timing::Print();
  time_outfile.close();
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  // google::InstallFailureSignalHandler();

  ros::init(argc, argv, "nvblox_lidar_node");
  ros::NodeHandle nh, pnh("~");

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();
  nvblox::NvbloxLidarNode nvblox_lidar_node(nh, pnh);

  // signal(SIGINT, SigintCallback);
  std::thread thread_proess_lidar_data(
      &nvblox::NvbloxLidarNode::ProcessLidarDataQueue, &nvblox_lidar_node);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Print computation time
  {
    std::vector<std::string> keywords = {std::string("ros")};
    LOG(INFO) << nvblox::timing::Timing::Print(keywords);
    std::ofstream time_outfile("/tmp/time_nvblox.txt");
    time_outfile << nvblox::timing::Timing::Print();
    time_outfile.close();
  }

  {
    std::vector<std::string> keywords = {
        std::string("ros/nvblox_lidar_node/compute_normal_image"),
        std::string("ros/nvblox_lidar_node/integrate_tsdf"),
        std::string("ros/nvblox_lidar_node/integrate_lidar_semantic"),
        std::string("ros/mesh/update")};
    LOG(INFO) << nvblox::timing::Timing::Print(keywords);
    std::ofstream time_outfile("/tmp/time_nvblox_keysteps.txt");
    time_outfile << nvblox::timing::Timing::Print(keywords);
    time_outfile.close();
  }

  {
    std::ofstream performance_outfile("/tmp/performance_nvblox.txt");
    // NOTE(gogojjh): Format
    // frame_num, block size, cpu_memory (GB), gpu_memory (GB),
    // normal_comp_time, metric_mapping_time, semantic_mapping_time,
    // mesh_update_time
    for (auto status : nvblox_lidar_node.v_status_) {
      for (int i = 0; i < status.size(); ++i) {
        performance_outfile << status[i] << " ";
      }
      performance_outfile << std::endl;
    }
    performance_outfile.close();
  }

  return EXIT_SUCCESS;
}