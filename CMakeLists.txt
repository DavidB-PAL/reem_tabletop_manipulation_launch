cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/test)

# Create sub directory
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/test)

# Tabletop Perception test
rosbuild_add_executable(tabletop_perception_test src/test/tabletop_perception_test.cpp)

# Table Manipulation test
rosbuild_add_executable(tabletop_grasping_test src/test/tabletop_grasping_test.cpp)



