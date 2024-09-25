# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.5)
project(nvblox_rviz_plugin)

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

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)
# find dependencies
find_package(catkin REQUIRED COMPONENTS
  rviz
  pluginlib
  nvblox_msgs
)
# Qt Stuff
find_package(Qt5 COMPONENTS Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    Qt5::Widgets
  CATKIN_DEPENDS
    rviz
    pluginlib
    nvblox_msgs
)

set(QT_INCLUDE_DIRS 
  ${Qt5Widgets_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} SHARED
  include/nvblox_rviz_plugin/nvblox_hash_utils.h
  include/nvblox_rviz_plugin/nvblox_mesh_display.h
  include/nvblox_rviz_plugin/nvblox_mesh_visual.h
  src/nvblox_mesh_display.cpp
  src/nvblox_mesh_visual.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

##########
# EXPORT #
##########
#target_compile_definitions(${PROJECT_NAME}  PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  FILES plugins_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons" 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION include/${PROJECT_NAME}/
)