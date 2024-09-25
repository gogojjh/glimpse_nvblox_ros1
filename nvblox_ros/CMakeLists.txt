# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.10)
#add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(nvblox_ros LANGUAGES CXX CUDA)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Default to release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

################
# DEPENDENCIES #
################

list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include")

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldMagenta "${Esc}[1;35m")
  set(Magenta     "${Esc}[35m")
endif()

find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  nvblox_msgs
  cv_bridge
  message_generation
)

add_service_files(
  FILES
  StatusInfo.srv
)

generate_messages(DEPENDENCIES)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_eigen
    nvblox_msgs
    message_generation
    message_runtime
)

# Process nvblox
set(NVBLOX_DIR ../nvblox/nvblox)

# Disable tests and experiments
set(BUILD_EXPERIMENTS OFF)
set(BUILD_TESTS OFF)

# Direct the child cmake to install eigen to the devel space.
# Note that we put it in the include/nvblox_ros directory, which should stop it
# colliding with other versions of eigen used by other catkin packages.
set(NVBLOX_EIGEN_CATKIN_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}")
set(EIGEN_INCLUDE_DESTINATION "${NVBLOX_EIGEN_CATKIN_INCLUDE_DESTINATION}")
set(STDGPU_INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX})

########
# CUDA #
########
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr -Xcudafe --display_error_number --disable-warnings ")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --compiler-options -fPIC")

##########
# NVBLOX #
##########
add_subdirectory(${NVBLOX_DIR} nvblox)

include_directories(AFTER include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${NVBLOX_DIR}/include
                    ${NVBLOX_DIR}/executables/include 
                    ${NVBLOX_DIR}/experiments/include)

message("${Magenta}INFO: ${CMAKE_MODULE_PATH}.${ColourReset}")
message("${Magenta}INFO: ${CMAKE_PREFIX_PATH}.${ColourReset}")
message("${Magenta}INFO: ${CMAKE_FIND_ROOT_PATH}.${ColourReset}")
message("${Magenta}INFO: ${CMAKE_INCLUDE_PATH}.${ColourReset}")

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}_lib SHARED
  src/conversions.cpp
  src/cuda/conversions.cu
  src/transformer.cpp
  src/rosbag_helper.cpp
  src/monitor_helper.cpp
)

target_link_libraries(${PROJECT_NAME}_lib 
  nvblox_interface
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME}_lib
  ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}_lib SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS})  

############
# BINARIES #
############
# nvblox_node
add_executable(nvblox_node src/nvblox_node.cpp)
target_link_libraries(nvblox_node ${PROJECT_NAME}_lib)
add_dependencies(nvblox_node ${catkin_EXPORTED_TARGETS})
target_include_directories(nvblox_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

# nvblox_lidar_node
add_executable(nvblox_lidar_node src/nvblox_lidar_node.cpp)
target_link_libraries(nvblox_lidar_node ${PROJECT_NAME}_lib)
add_dependencies(nvblox_lidar_node ${catkin_EXPORTED_TARGETS})
target_include_directories(nvblox_lidar_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

# 3dmatch_node
add_executable(nvblox_3dmatch_node src/3dmatch_node.cpp
  ${NVBLOX_DIR}/executables/src/datasets/3dmatch.cpp
  ${NVBLOX_DIR}/executables/src/fuser_rgbd.cpp
)
target_link_libraries(nvblox_3dmatch_node ${PROJECT_NAME}_lib)
add_dependencies(nvblox_3dmatch_node ${catkin_EXPORTED_TARGETS})
target_include_directories(nvblox_3dmatch_node SYSTEM PUBLIC
  include ${catkin_INCLUDE_DIRS}
)

# 62 Should be TX2, jetson
set_target_properties(${PROJECT_NAME}_lib PROPERTIES CUDA_ARCHITECTURES "62;72;75;80;86;87")

###########
# INSTALL #
###########
# catkin_simple doesn't manage everything in our case, where we're installing
# not only the ros package nvblox_ros, but it's dependancies from nvblox.
# We therefore export the nvblox target along with the nvblox_ros stuff.
get_target_property(nvblox_INCLUDES nvblox_interface INTERFACE_INCLUDE_DIRECTORIES)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_lib nvblox_lib nvblox_gpu_hash nvblox_cuda_check
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install nodes
install(
  TARGETS nvblox_node nvblox_3dmatch_node
  DESTINATION lib/${PROJECT_NAME}
)
