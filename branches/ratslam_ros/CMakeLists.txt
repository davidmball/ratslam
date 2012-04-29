cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Irrlicht REQUIRED)
find_package(OpenGL REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(ratslam src/ratslam/Experience_Map.cpp src/ratslam/Pose_Cell_Network.cpp src/ratslam/Visual_Template_Match.cpp)

rosbuild_add_boost_directories()

add_definitions("-DHAVE_IRRLICHT")

rosbuild_add_executable(ratslam_vt src/main_vt.cpp)
target_link_libraries(ratslam_vt ratslam Irrlicht GL)
rosbuild_link_boost(ratslam_vt serialization)

rosbuild_add_executable(ratslam_pc src/main_pc.cpp)
target_link_libraries(ratslam_pc ratslam Irrlicht GL)
rosbuild_link_boost(ratslam_pc serialization)

rosbuild_add_executable(ratslam_em src/main_em.cpp)
target_link_libraries(ratslam_em ratslam Irrlicht GL)
rosbuild_link_boost(ratslam_em serialization)
