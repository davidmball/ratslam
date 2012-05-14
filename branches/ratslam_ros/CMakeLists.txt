cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

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
rosbuild_add_library(ratslam src/ratslam/experience_map.cpp src/ratslam/posecell_network.cpp src/ratslam/local_view_match.cpp src/ratslam/visual_odometry.cpp)

rosbuild_add_boost_directories()

# uncomment is you don't have irrlicht installed
add_definitions("-DHAVE_IRRLICHT")

rosbuild_add_executable(ratslam_lv src/main_lv.cpp)
target_link_libraries(ratslam_lv ratslam Irrlicht GL opencv_core opencv_imgproc opencv_highgui)
rosbuild_link_boost(ratslam_lv serialization)

rosbuild_add_executable(ratslam_pc src/main_pc.cpp)
target_link_libraries(ratslam_pc ratslam Irrlicht GL)
rosbuild_link_boost(ratslam_pc serialization)

rosbuild_add_executable(ratslam_em src/main_em.cpp)
target_link_libraries(ratslam_em ratslam Irrlicht GL)
rosbuild_link_boost(ratslam_em serialization)

rosbuild_add_executable(ratslam_vo src/main_vo.cpp)
target_link_libraries(ratslam_vo ratslam opencv_core opencv_imgproc opencv_highgui)
rosbuild_link_boost(ratslam_vo serialization)
