/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
 
#pragma once

#include <memory>

//#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
//#include <rviz/default_plugin/tools/move_tool.h>
//#include <rviz/display_context.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

#include <nvblox_msgs/Mesh.h>

#include "nvblox_rviz_plugin/nvblox_mesh_visual.h"

namespace nvblox_rviz_plugin {

class NvbloxMeshVisual;

class NvbloxMeshDisplay : public rviz::MessageFilterDisplay<nvblox_msgs::Mesh> {
      
  Q_OBJECT
 public:
  NvbloxMeshDisplay();
  virtual ~NvbloxMeshDisplay();

 public Q_SLOTS:
  virtual void updateCeilingOptions();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const nvblox_msgs::Mesh::ConstPtr& msg) override;

  rviz::BoolProperty* cut_ceiling_property_;
  rviz::FloatProperty* ceiling_height_property_;

  std::unique_ptr<NvbloxMeshVisual> visual_;
};

}  // namespace nvblox_rviz_plugin
