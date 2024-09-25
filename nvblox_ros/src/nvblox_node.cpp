/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "nvblox_ros/nvblox_node.hpp"

#include <nvblox/core/cuda/warmup.h>
#include <nvblox/io/mesh_io.h>
#include <nvblox/io/pointcloud_io.h>
#include <nvblox/utils/timing.h>

// std libs

// Jetson requires Jetpack 5.0 to have c++17 so libraries like filesystem cant
// be used now.
//#include <filesystem>

#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nvblox_ros/conversions.hpp"

using namespace std::chrono_literals;

namespace nvblox {

NvbloxNode::NvbloxNode(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), transformer_(nodeHandle) {
  if (!readParameters()) {
    ROS_ERROR("Error reading parameters for Nvblox. Program will exit.");
    return;
  }

  // Setup ros communication.
  setupRosCommunication();

  // Set the transformer settings.
  transformer_.set_global_frame(global_frame_);
  transformer_.set_pose_frame(pose_frame_);

  // Initialize the map
  mapper_ = std::make_unique<RgbdMapper>(voxel_size_);

  // Read mapping, integration related parameters.
  if (!readMappingParameters()) {
    ROS_ERROR(
        "Error reading mapping parameters for Nvblox. Program will exit.");
    return;
  }

  // Prematurely check correctness and existance of save path.
  if (!manageSaveDirectory()) {
    return;
  }

  ROS_INFO_STREAM("Started up nvblox node in frame "
                  << global_frame_ << " and voxel size " << voxel_size_);

  // Set state.
  last_tsdf_update_time_ = ros::Time(0.0);
  last_color_update_time_ = ros::Time(0.0);
  last_esdf_update_time_ = ros::Time(0.0);
  last_mesh_update_time_ = ros::Time(0.0);

  std::tie(freeMemoryAfterLastSave, std::ignore) = getFreeGPUMemory();
}

bool NvbloxNode::manageSaveDirectory() {
  if (output_dir_.empty()) {
    // Get username
    std::string userName{getlogin()};
    ROS_INFO_STREAM("User name: " << userName);
    output_dir_ = "/home/" + userName + "/maps";

    if (!doesDirectoryExist(output_dir_)) {
      if (makePath(output_dir_)) {
        ROS_INFO_STREAM("Created folder : " << output_dir_);
      } else {
        ROS_ERROR_STREAM("Could not create folder " << output_dir_
                                                    << " Exitting program.");
        return false;
      }
    }

    ROS_INFO_STREAM("Outputting results (as requested) to: (" << output_dir_
                                                              << ")");

  } else {
    // Check if folder exists
    if (!doesDirectoryExist(output_dir_)) {
      // If does not exist create the folder recursively.
      if (makePath(output_dir_)) {
        ROS_INFO_STREAM("Created folder : " << output_dir_);
      } else {
        // Quit if the folder was not created.
        ROS_ERROR_STREAM("Could not create folder " << output_dir_
                                                    << "Exitting program.");
        return false;
      }
    }

    ROS_INFO_STREAM("Outputting results (as requested) to: " << output_dir_);
  }

  return true;
}

bool NvbloxNode::makePath(const std::string& path) {
  // C magic, read, write and execute rights.
  const mode_t mode{0777};

  if (mkdir(path.c_str(), mode) == 0) {
    return true;
  }

  // if the deepest folder was not possible to create check parents.
  switch (errno) {
    case ENOENT:
      // Parent didn't exist, try to create it
      {
        std::size_t lastSlashPosition{path.find_last_of('/')};
        if (lastSlashPosition == std::string::npos) {
          // If the slash is at the end, nothing to do.
          return false;
        }
        // Get the path until the previous `/` and create that path.
        if (!makePath(path.substr(0, lastSlashPosition))) {
          return false;
        }
      }
      // now, try to create again
      return 0 == mkdir(path.c_str(), mode);

    case EEXIST:
      // done!
      return doesDirectoryExist(path);

    default:
      return false;
  }
}

bool NvbloxNode::doesDirectoryExist(const std::string& path) {
  struct stat info;

  if (stat(path.c_str(), &info) != 0) {
    return false;
  }

  return (info.st_mode & S_IFDIR) != 0;
}

void NvbloxNode::setupRosCommunication() {
  // Optional ways to provide transformations instead of TF.
  transform_sub_ = nodeHandle_.subscribe("transform", 10,
                                         &NvbloxNode::transformCallback, this);
  pose_sub_ =
      nodeHandle_.subscribe("pose", 10, &NvbloxNode::poseCallback, this);

  odom_sub_ = nodeHandle_.subscribe("aft_mapped_to_init", 10,
                                    &NvbloxNode::odomCallback, this);

  // A service to save the mesh as PLY.
  save_ply_service_ =
      nodeHandle_.advertiseService("save_ply", &NvbloxNode::savePly, this);

  // The timer for checking the avilable free memory in GPU.
  memoryTimer_ = nodeHandle_.createTimer(
      ros::Duration(static_cast<double>(memoryCheckingPeriod_)),
      &NvbloxNode::memoryTimerCallback, this);

  // Queue size is important, since we have to wait synced messages.
  constexpr int kQueueSize = 20;

  depth_sub1_.subscribe(nodeHandle_, depth_image_1_topic, 20);
  depth_camera_info_sub1_.subscribe(nodeHandle_, camera_info_1_topic, 20);

  depth_sub2_.subscribe(nodeHandle_, depth_image_2_topic, 20);
  depth_camera_info_sub2_.subscribe(nodeHandle_, camera_info_2_topic, 20);

  depth_sub3_.subscribe(nodeHandle_, depth_image_3_topic, 20);
  depth_camera_info_sub3_.subscribe(nodeHandle_, camera_info_3_topic, 20);

  color_sub1_.subscribe(nodeHandle_, rgb_image_1_topic, 20);
  color_camera_info_sub1_.subscribe(nodeHandle_, camera_info_1_topic, 20);

  color_sub2_.subscribe(nodeHandle_, rgb_image_2_topic, 20);
  color_camera_info_sub2_.subscribe(nodeHandle_, camera_info_2_topic, 20);

  color_sub3_.subscribe(nodeHandle_, rgb_image_3_topic, 20);
  color_camera_info_sub3_.subscribe(nodeHandle_, camera_info_3_topic, 20);

  // All depth images are binded to the same callback.
  depthSync1_ =
      std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(
          image_pair_sync_pol(kQueueSize), depth_sub1_,
          depth_camera_info_sub1_);
  depthSync1_->registerCallback(std::bind(&NvbloxNode::depthImageCallback, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

  depthSync2_ =
      std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(
          image_pair_sync_pol(kQueueSize), depth_sub2_,
          depth_camera_info_sub2_);
  depthSync2_->registerCallback(std::bind(&NvbloxNode::depthImageCallback, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

  depthSync3_ =
      std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(
          image_pair_sync_pol(kQueueSize), depth_sub3_,
          depth_camera_info_sub3_);
  depthSync3_->registerCallback(std::bind(&NvbloxNode::depthImageCallback, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

  // All rgb images are binded to the same callback.
  rgbSync1_ =
      std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(
          image_pair_sync_pol(kQueueSize), color_sub1_,
          color_camera_info_sub1_);
  rgbSync1_->registerCallback(std::bind(&NvbloxNode::colorImageCallback, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  rgbSync2_ =
      std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(
          image_pair_sync_pol(kQueueSize), color_sub2_,
          color_camera_info_sub2_);
  rgbSync2_->registerCallback(std::bind(&NvbloxNode::colorImageCallback, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  rgbSync3_ =
      std::make_unique<message_filters::Synchronizer<image_pair_sync_pol>>(
          image_pair_sync_pol(kQueueSize), color_sub3_,
          color_camera_info_sub3_);
  rgbSync3_->registerCallback(std::bind(&NvbloxNode::colorImageCallback, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  // Publishers of Nvblox.
  mesh_publisher_ = nodeHandle_.advertise<nvblox_msgs::Mesh>("mesh", 1, true);
  pointcloud_publisher_ =
      nodeHandle_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, false);
  map_slice_publisher_ = nodeHandle_.advertise<nvblox_msgs::DistanceMapSlice>(
      "map_slice", 1, false);
}

bool NvbloxNode::readMappingParameters() {
  bool success{true};
  // Integrator settings.

  // ************************** tsdf_integrator
  float max_integration_distance_m_local = 10.0f;
  success &=
      nodeHandle_.param<float>("tsdf_integrator_max_integration_distance_m",
                               max_integration_distance_m_local, 10.0f);
  mapper_->tsdf_integrator().max_integration_distance_m(
      max_integration_distance_m_local);

  float truncation_distance_vox_local = 4.0f;
  success &= nodeHandle_.param<float>("tsdf_integrator_truncation_distance_vox",
                                      truncation_distance_vox_local, 4.0f);
  mapper_->tsdf_integrator().truncation_distance_vox(
      truncation_distance_vox_local);

  float max_weight_local = 100.0f;
  success &= nodeHandle_.param<float>("tsdf_integrator_max_weight",
                                      max_weight_local, 100.0f);
  mapper_->tsdf_integrator().max_weight(max_weight_local);

  // ************************** mesh_integrator
  float mesh_min_weight_local = mapper_->mesh_integrator().min_weight();
  success &= nodeHandle_.param<float>("mesh_integrator_min_weight",
                                      mesh_min_weight_local, 1e-4);

  bool mesh_weld_vertices = mapper_->mesh_integrator().weld_vertices();
  success &= nodeHandle_.param<bool>("mesh_integrator_weld_vertices",
                                     mesh_weld_vertices, false);

  // ************************** color_integrator
  float max_integration_distance_m_color_local = 10.0f;
  success &=
      nodeHandle_.param<float>("color_integrator_max_integration_distance_m",
                               max_integration_distance_m_color_local, 10.0f);
  mapper_->color_integrator().max_integration_distance_m(
      max_integration_distance_m_color_local);

  // ************************** esdf_integrator
  float esdf_min_weight = mapper_->esdf_integrator().min_weight();
  success &= nodeHandle_.param<float>("esdf_integrator_min_weight",
                                      esdf_min_weight, 1e-4);

  float max_site_distance_vox =
      mapper_->esdf_integrator().max_site_distance_vox();
  success &= nodeHandle_.param<float>("esdf_integrator_max_site_distance_vox",
                                      max_site_distance_vox, 1.0f);

  float esdf_max_distance_m = mapper_->esdf_integrator().max_distance_m();
  success &= nodeHandle_.param<float>("esdf_integrator_max_distance_m",
                                      esdf_max_distance_m, 10.0f);

  success &= nodeHandle_.param<std::string>("output_dir", output_dir_, "");

  return success;
}

bool NvbloxNode::readParameters() {
  bool success{true};
  // Ros param reading.
  success &= nodeHandle_.param<float>("voxel_size", voxel_size_, 0.05f);
  success &=
      nodeHandle_.param<std::string>("global_frame", global_frame_, "map");
  success &= nodeHandle_.param<std::string>("pose_frame", pose_frame_, "base");
  success &= nodeHandle_.param<int>("memory_checking_period",
                                    memoryCheckingPeriod_, 5);
  success &=
      nodeHandle_.param<bool>("use_helper_frame", useHelperFrame_, false);
  success &= nodeHandle_.param<int>("memory_purge_threshold",
                                    memoryPurgeThreshold_, 750);

  // Depth camera topics
  success &= nodeHandle_.param<std::string>(
      "subscribers/depth_image_1_topic", depth_image_1_topic,
      "/point_cloud_colorizer_ros/depth_image_camera_1");
  success &= nodeHandle_.param<std::string>(
      "subscribers/depth_image_2_topic", depth_image_2_topic,
      "/point_cloud_colorizer_ros/depth_image_camera_2");
  success &= nodeHandle_.param<std::string>(
      "subscribers/depth_image_3_topic", depth_image_3_topic,
      "/point_cloud_colorizer_ros/depth_image_camera_3");

  // RBG camera topics
  success &= nodeHandle_.param<std::string>(
      "subscribers/rgb_image_1_topic", rgb_image_1_topic,
      "/alphasense_driver_ros/cam3/dropped/debayered/slow");
  success &= nodeHandle_.param<std::string>(
      "subscribers/rgb_image_2_topic", rgb_image_2_topic,
      "/alphasense_driver_ros/cam4/dropped/debayered/slow");
  success &= nodeHandle_.param<std::string>(
      "subscribers/rgb_image_3_topic", rgb_image_3_topic,
      "/alphasense_driver_ros/cam5/dropped/debayered/slow");

  // Camera info topics
  success &= nodeHandle_.param<std::string>(
      "subscribers/camera_info_1_topic", camera_info_1_topic,
      "/camera_utils/alphasense_cam3/cameraInfo");
  success &= nodeHandle_.param<std::string>(
      "subscribers/camera_info_2_topic", camera_info_2_topic,
      "/camera_utils/alphasense_cam4/cameraInfo");
  success &= nodeHandle_.param<std::string>(
      "subscribers/camera_info_3_topic", camera_info_3_topic,
      "/camera_utils/alphasense_cam5/cameraInfo");

  success &= nodeHandle_.param<bool>("mesh", mesh_, true);
  success &= nodeHandle_.param<bool>("esdf", esdf_, true);
  success &= nodeHandle_.param<bool>("esdf_2d", esdf_2d_, true);
  success &= nodeHandle_.param<bool>("distance_slice", distance_slice_, true);
  success &= nodeHandle_.param<float>("slice_height", slice_height_, 1.0f);
  success &= nodeHandle_.param<float>("min_height", min_height_, 0.0f);
  success &= nodeHandle_.param<float>("max_height", max_height_, 1.0f);

  success &= nodeHandle_.param<float>("max_tsdf_update_hz", max_tsdf_update_hz_,
                                      10.0f);
  success &= nodeHandle_.param<float>("max_color_update_hz",
                                      max_color_update_hz_, 5.0f);
  success &=
      nodeHandle_.param<float>("max_mesh_update_hz", max_mesh_update_hz_, 5.0f);
  success &=
      nodeHandle_.param<float>("max_esdf_update_hz", max_esdf_update_hz_, 2.0f);

  int maxMeshingHeight{4};
  success &= nodeHandle_.param<int>(
      nodeHandle_.getNamespace() + "/max_meshing_height", maxMeshingHeight, 4);
  converter_.maxHeight_ = std::move(maxMeshingHeight);

  return success;
}

void NvbloxNode::transformCallback(
    const geometry_msgs::TransformStampedConstPtr& transform_msg) {
  transformer_.transformCallback(transform_msg);
}

void NvbloxNode::poseCallback(
    const geometry_msgs::PoseStampedConstPtr& transform_msg) {
  transformer_.poseCallback(transform_msg);
}

void NvbloxNode::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  transformer_.odomCallback(odom_msg);
}

void NvbloxNode::depthImageCallback(
    const sensor_msgs::ImageConstPtr& depth_img_ptr,
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
  timing::Timer ros_total_timer("ros/total");
  // Cache clock_now.
  ros::Time clock_now = depth_img_ptr->header.stamp;

  ROS_DEBUG_STREAM("Depth image received. Image stamp:" << clock_now);

  if (max_tsdf_update_hz_ > 0.0f &&
      (clock_now - last_tsdf_update_time_).toSec() <
          1.0f / max_tsdf_update_hz_) {
    ROS_DEBUG_STREAM(
        "Skipping integrating one depth measurement due to update rate.");
    // Skip integrating this.
    return;
  }

  if (last_tsdf_update_time_ == clock_now) {
    ROS_DEBUG_STREAM(
        "Message with same timestamp arrived skipping this message.");
    // Skip integrating this.
    return;
  }

  last_tsdf_update_time_ = clock_now;

  // Push it into the queue.
  depth_image_queue_.emplace_back(depth_img_ptr, camera_info_msg);
  processDepthQueue();
}

void NvbloxNode::colorImageCallback(
    const sensor_msgs::ImageConstPtr& color_image_ptr,
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
  timing::Timer ros_total_timer("ros/total");
  // Cache clock_now.
  ros::Time clock_now = color_image_ptr->header.stamp;
  ROS_DEBUG_STREAM("RGB image received. Image stamp:" << clock_now);

  if (max_color_update_hz_ > 0.0f &&
      (clock_now - last_color_update_time_).toSec() <
          1.0f / max_color_update_hz_) {
    ROS_DEBUG_STREAM("Skipping integrating one rgb image due to update rate.");
    // Skip integrating this.
    return;
  }
  last_color_update_time_ = clock_now;

  // Push it into the queue.
  color_image_queue_.emplace_back(color_image_ptr, camera_info_msg);
  processColorQueue();
}

void NvbloxNode::processDepthQueue() {
  timing::Timer ros_tsdf_timer("ros/tsdf");

  auto it = depth_image_queue_.begin();
  auto it_first_valid = depth_image_queue_.end();
  auto it_last_valid = depth_image_queue_.begin();

  while (++it != depth_image_queue_.end()) {
    sensor_msgs::ImageConstPtr depth_img_ptr = it->first;
    sensor_msgs::CameraInfoConstPtr camera_info_msg = it->second;

    ros::Time clock_now = depth_img_ptr->header.stamp;

    timing::Timer transform_timer("ros/tsdf/transform");
    // Get the TF for this image.
    Transform T_S_C;

    std::string target_frame =
        (useHelperFrame_) ? (depth_img_ptr->header.frame_id + "_helper")
                          : (depth_img_ptr->header.frame_id);

    // This transform either uses tf or provided transform.
    if (!transformer_.lookupTransformToGlobalFrame(
            target_frame, depth_img_ptr->header.stamp, &T_S_C)) {
      ROS_ERROR("No tf transform available");
      continue;
    }
    transform_timer.Stop();

    timing::Timer conversions_timer("ros/tsdf/conversions");
    // Convert camera info message to camera object.
    Camera camera = converter_.cameraFromMessage(*camera_info_msg);

    // Convert the depth image.
    if (!converter_.depthImageFromImageMessage(depth_img_ptr, &depth_image_)) {
      ROS_ERROR("Failed to transform depth image.");
      continue;
    }
    conversions_timer.Stop();

    // Integrate
    timing::Timer integration_timer("ros/tsdf/integrate");
    mapper_->integrateDepth(depth_image_, T_S_C, camera);
    integration_timer.Stop();
    ros_tsdf_timer.Stop();

    // Esdf integrator (if enabled)
    if (esdf_) {
      // Check if it's been long enough since the last frame.
      if (max_esdf_update_hz_ <= 0.0f ||
          (clock_now - last_esdf_update_time_).toSec() >=
              1.0f / max_esdf_update_hz_) {
        last_esdf_update_time_ = clock_now;

        // Then do the update.
        // Otherwise do nothing.
        updateEsdf(depth_img_ptr->header.stamp);
      }
    }

    // Mesh integrator
    if (mesh_) {
      if (max_mesh_update_hz_ <= 0.0f ||
          (clock_now - last_mesh_update_time_).toSec() >=
              1.0f / max_mesh_update_hz_) {
        last_mesh_update_time_ = clock_now;
        updateMesh(depth_img_ptr->header.stamp);
      }
    }

    if (it_first_valid == depth_image_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != depth_image_queue_.end()) {
    // Actually erase from the beginning of the queue.
    depth_image_queue_.erase(depth_image_queue_.begin(), ++it_last_valid);
  }
}

void NvbloxNode::processColorQueue() {
  timing::Timer ros_color_timer("ros/color");

  auto it = color_image_queue_.begin();
  auto it_first_valid = color_image_queue_.end();
  auto it_last_valid = color_image_queue_.begin();

  while (++it != color_image_queue_.end()) {
    sensor_msgs::ImageConstPtr color_image_ptr = it->first;
    sensor_msgs::CameraInfoConstPtr camera_info_msg = it->second;

    timing::Timer transform_timer("ros/color/transform");

    // Get the TF for this image.
    std::string target_frame =
        (useHelperFrame_) ? (color_image_ptr->header.frame_id + "_helper")
                          : (color_image_ptr->header.frame_id);
    Transform T_S_C;

    if (!transformer_.lookupTransformToGlobalFrame(
            target_frame, color_image_ptr->header.stamp, &T_S_C)) {
      ROS_ERROR("No tf transform available");
      continue;
    }

    transform_timer.Stop();

    timing::Timer color_convert_timer("ros/color/conversion");

    // Convert camera info message to camera object.
    Camera camera = converter_.cameraFromMessage(*camera_info_msg);

    // Convert the color image.
    if (!converter_.colorImageFromImageMessage(color_image_ptr,
                                               &color_image_)) {
      ROS_ERROR("Failed to transform color image.");
      continue;
    }
    color_convert_timer.Stop();

    // Integrate.
    timing::Timer color_integrate_timer("ros/color/integrate");
    mapper_->integrateColor(color_image_, T_S_C, camera);
    color_integrate_timer.Stop();

    if (it_first_valid == color_image_queue_.end()) {
      it_first_valid = it;
    }
    if (it_last_valid <= it) {
      it_last_valid = it;
    }
  }

  // Now we have 2 iterators pointing to what we want to delete.
  if (it_first_valid != color_image_queue_.end()) {
    // Actually erase from the beginning of the queue.
    color_image_queue_.erase(color_image_queue_.begin(), ++it_last_valid);
  }
}

void NvbloxNode::updateEsdf(const ros::Time& timestamp) {
  timing::Timer ros_esdf_timer("ros/esdf");

  timing::Timer esdf_integration_timer("ros/esdf/integrate");

  if (esdf_2d_) {
    mapper_->updateEsdfSlice(min_height_, max_height_, slice_height_);
  } else {
    mapper_->updateEsdf();
  }

  esdf_integration_timer.Stop();

  timing::Timer esdf_output_timer("ros/esdf/output");

  if (pointcloud_publisher_.getNumSubscribers() > 0u) {
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

    converter_.pointcloudFromLayerInAABB(mapper_->esdf_layer(), aabb,
                                         &pointcloud_msg);

    pointcloud_msg.header.frame_id = global_frame_;
    pointcloud_msg.header.stamp = timestamp;
    pointcloud_publisher_.publish(pointcloud_msg);

    output_pointcloud_timer.Stop();
  }

  // Also publish the map slice.
  if (distance_slice_ && map_slice_publisher_.getNumSubscribers() > 0u) {
    timing::Timer output_map_slice_timer("ros/esdf/output/map_slice");

    nvblox_msgs::DistanceMapSlice map_slice;

    converter_.distanceMapSliceFromLayer(mapper_->esdf_layer(), slice_height_,
                                         &map_slice);
    map_slice.header.frame_id = global_frame_;
    map_slice.header.stamp = timestamp;
    map_slice_publisher_.publish(map_slice);
  }
}

void NvbloxNode::updateMesh(const ros::Time& timestamp) {
  timing::Timer ros_mesh_timer("ros/mesh");

  timing::Timer mesh_integration_timer("ros/mesh/integrate_and_color");
  const std::vector<Index3D> mesh_updated_list = mapper_->updateMesh();
  mesh_integration_timer.Stop();

  // Publish the mesh updates.
  timing::Timer mesh_output_timer("ros/mesh/output");
  size_t new_subscriber_count = mesh_publisher_.getNumSubscribers();
  if (new_subscriber_count > 0u) {
    nvblox_msgs::Mesh mesh_msg;
    // In case we have new subscribers, publish the ENTIRE map once.
    if (new_subscriber_count > mesh_subscriber_count_) {
      ROS_INFO("Got a new subscriber, sending entire map.");

      converter_.meshMessageFromMeshBlocks(
          mapper_->mesh_layer(), mapper_->mesh_layer().getAllBlockIndices(),
          &mesh_msg);
      mesh_msg.clear = true;
    } else {
      converter_.meshMessageFromMeshBlocks(mapper_->mesh_layer(),
                                           mesh_updated_list, &mesh_msg);
    }
    mesh_msg.header.frame_id = global_frame_;
    mesh_msg.header.stamp = timestamp;
    mesh_publisher_.publish(mesh_msg);
  }
  mesh_subscriber_count_ = new_subscriber_count;

  mesh_output_timer.Stop();
}

void NvbloxNode::memoryTimerCallback(const ros::TimerEvent& /*event*/) {
  int start_free_gpu_memory_mb{0};
  std::tie(start_free_gpu_memory_mb, std::ignore) = getFreeGPUMemory();

  ROS_DEBUG_STREAM(
      "Timed Memory check. Free memory: " << start_free_gpu_memory_mb);

  // Update the free memory holder if we have more available memory.
  freeMemoryAfterLastSave = (start_free_gpu_memory_mb > freeMemoryAfterLastSave)
                                ? start_free_gpu_memory_mb
                                : freeMemoryAfterLastSave;

  // Check whether we have a big memory diff.
  if (freeMemoryAfterLastSave - start_free_gpu_memory_mb >
      memoryPurgeThreshold_) {
    // Generate mesh save path.
    const std::string nameOfFile{"/" + std::to_string(mapSaveCounter_) +
                                 "_nvblox_mesh" + ".ply"};
    ROS_INFO_STREAM("Requested save folder:  " << output_dir_);

    // Currently we are saving to /home/<user>/.ros/maps folder. User is
    // automatically detected.
    // const std::string
    // topDirectoryPath{static_cast<std::string>(std::filesystem::current_path())
    // + "/maps"};

    /*
    // First check and create the top level directory within ./ros folder.
    if (!std::filesystem::exists(topDirectoryPath)) {
      bool success{std::filesystem::create_directory(topDirectoryPath)};
      if (!success) {
        ROS_ERROR("Asked folder does not exist.");
        return;
      }
      // Give permission to all to read, write and execute to the new directory.
      std::filesystem::permissions(topDirectoryPath,
    std::filesystem::perms::all);
    }
    */

    // Generate the file name itself.
    const std::string fileName{output_dir_ + "/" + nameOfFile};

    // Actual save.
    bool success = io::outputMeshLayerToPly(mapper_->mesh_layer(), fileName);

    if (!success) {
      ROS_ERROR("Couldn't Save Mesh File.!");
      return;
    }

    // Purge the mesh at cache,
    mapper_->mesh_layer().clear();
    mapper_->esdf_layer().clear();

    // Update Saved Map Counter
    mapSaveCounter_ += 1u;

    // Re-query memory
    std::tie(freeMemoryAfterLastSave, std::ignore) = getFreeGPUMemory();

    ROS_INFO_STREAM("Nvblox: Saved a local mesh file named "
                    << nameOfFile << ". Free Space from "
                    << start_free_gpu_memory_mb << " MB to "
                    << freeMemoryAfterLastSave << " MB.");
  }
}

bool NvbloxNode::savePly(std_srvs::Empty::Request& /*request*/,
                         std_srvs::Empty::Response& /*response*/) {
  int start_free_gpu_memory_mb;
  std::tie(start_free_gpu_memory_mb, std::ignore) = getFreeGPUMemory();

  ROS_INFO_STREAM("Free Memory before save : " << start_free_gpu_memory_mb);

  // Commented out since jetpack version < 5.0 does not support c++17 utilities.
  // const std::string
  // topDirectoryPath{static_cast<std::string>(std::filesystem::current_path())
  // +
  // "/maps"};

  io::outputMeshLayerToPly(mapper_->mesh_layer(),
                           output_dir_ + "/manual_mesh_nvblox.ply");
  ROS_INFO_STREAM("Output PLY files to " << output_dir_);

  mapper_->mesh_layer().clear();
  mapper_->esdf_layer().clear();

  int end_free_gpu_memory_mb;
  std::tie(end_free_gpu_memory_mb, std::ignore) = getFreeGPUMemory();

  ROS_INFO_STREAM("Free Memory after save : " << end_free_gpu_memory_mb);

  return true;
}

std::pair<int, float> NvbloxNode::getFreeGPUMemory() {
  size_t free_bytes;
  size_t total_bytes;
  cudaMemGetInfo(&free_bytes, &total_bytes);
  const int free_mb = free_bytes / 1e6;
  const int total_mb = total_bytes / 1e6;
  const float free_percent =
      static_cast<float>(free_mb) * 100.0f / static_cast<float>(total_mb);
  return {free_mb, free_percent};
}

}  // namespace nvblox

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "nvblox_node");
  ros::NodeHandle nh("~");

  // Warmup CUDA so it doesn't affect our timings *as* much for the first
  // CUDA call.
  nvblox::warmupCuda();

  nvblox::NvbloxNode NvbloxNode(nh);

  ros::spin();

  return EXIT_SUCCESS;
}
