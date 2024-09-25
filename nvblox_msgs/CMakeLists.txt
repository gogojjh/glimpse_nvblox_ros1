# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.5)
project(nvblox_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

set(BUILD_EXPERIMENTS OFF)
set(BUILD_TESTS OFF)
set(BUILD_TESTING OFF)

# find dependencies
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(
  FILES
    Index3D.msg
    MeshBlock.msg
    Mesh.msg
    DistanceMapSlice.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
