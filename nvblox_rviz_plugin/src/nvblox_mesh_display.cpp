/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
//#include <rviz/display.h>
#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

#include "nvblox_rviz_plugin/nvblox_mesh_display.h"

namespace nvblox_rviz_plugin {

NvbloxMeshDisplay::NvbloxMeshDisplay() {
  cut_ceiling_property_ = new rviz::BoolProperty(
      "Cut Ceiling", false,
      "If set to true, will not visualize anything above a certain z value.",
      this, SLOT(updateCeilingOptions()));

  ceiling_height_property_ = new rviz::FloatProperty(
      "Ceiling Height", 0.0,
      "Height above which the visualization will be cut off.", this,
      SLOT(updateCeilingOptions()));
}

void NvbloxMeshDisplay::updateCeilingOptions() {
  if (visual_ != nullptr) {
    visual_->setCeilingCutoff(cut_ceiling_property_->getBool(),
                              ceiling_height_property_->getFloat());
  }
}

void NvbloxMeshDisplay::onInitialize() { MFDClass::onInitialize(); }

NvbloxMeshDisplay::~NvbloxMeshDisplay() {}

void NvbloxMeshDisplay::reset() {
  MFDClass::reset();
  visual_.reset();
}

void NvbloxMeshDisplay::processMessage(const nvblox_msgs::Mesh::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_ERROR("Error transforming from frame '%s' to frame '%s'",msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }
  

  if (visual_ == nullptr) {
    visual_.reset(new NvbloxMeshVisual(context_->getSceneManager(), scene_node_));
    visual_->setCeilingCutoff(cut_ceiling_property_->getBool(),
                              ceiling_height_property_->getFloat());
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace nvblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nvblox_rviz_plugin::NvbloxMeshDisplay,
                       rviz::Display)
