# RatSLAM ROS (OpenRatSLAM)

[![Build Status](https://travis-ci.org/sem23/ratslam.svg?branch=ratslam_ros)](https://travis-ci.org/sem23/ratslam)

## Package Summary

This package contains the RatSLAM sources including ROS wrappers. The package provides a biological inspired monocular
vision SLAM (Simultaneous Localization and Mapping) system. Using this system you can create a topological map of the
environment, which means there are just paths and **no** gridmap, like an occupancy grid. As a gimmick, the system is
able to perfom pathplanning on the current map.

Authors Documentation:<br>
The ROS version of RatSLAM has been designed as a more modular version than the other library in this project - the algorithm has been divided into 3 individual RatSLAM ROS nodes and one visual odometry node. This version is referred to as OpenRatSLAM.

OpenRatSLAM is based on the RatSLAM C++ library written by David Ball and Scott Heath, with support and additions by Michael Milford. The RatSLAM algorithm was originally designed and implemented by Michael Milford and Gordon Wyeth.

OpenRatSLAM has only been tested on Ubuntu and as ROS support for platforms other than Linux is still limited, this will probably not change.

* Maintainer status: maintained
* Maintainers: David M. Ball \<davidmichaelball AT gmail DOT com\>, Peter Rudolph \<semael23 AT gmail DOT com\> 
* Authors: Michael Milford and Gordon Wyeth (RatSLAM), David Ball and Scott Heath (RatSLAM C++), David M. Ball (OpenRatSLAM)
* License: GNU General Public License v3.0
* Source: git https://github.com/sem23/ratslam.git (branch: ratslam_ros)

## 1. External Documentation

[David Ball, Scott Heath, Janet Wiles, Gordon Wyeth, Peter Corke, Michael Milford: OpenRatSLAM: an open source brain-based SLAM system, Autonomous Robots, 2013](https://link.springer.com/article/10.1007%2Fs10514-012-9317-9).

## 2. Hardware Requirements

To get RatSLAM working you need a properly setup monocular camera and an odometry source. The odometry can be generated
using the visual odometry node, which is provided by the package. But note, that a wheel based odometry is much more
accurate in most cases.

## 3. Example

Several examples are provided [here](https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md#running-ratslam).

## 4. Nodes

### 4.1 ratslam_em

The experience map is used to store the experiences made by the robot. This node provides a topological map, loop closing capabilites
and a dijkstra pathplanning algorithm.

#### 4.1.1 Subscribed Topics

"odom" ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))
> The current odometry.

topic_root + "/PoseCell/TopologicalAction" (ratslam_ros/TopologicalAction)
> The topological action.

topic_root + "/ExperienceMap/SetGoalPose" ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
> The goal pose to where path will be planned.

#### 4.1.2 Published Topics

topic_root + "/ExperienceMap/Map" (ratslam_ros/TopologicalMap)
> The experience map in RatSLAM format.

topic_root + "/ExperienceMap/MapMarker" ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
> The experience map as ROS visualization markers for displaying puposes.

topic_root + "/ExperienceMap/RobotPose" ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
> The current pose of the robot in map.

topic_root + "/ExperienceMap/PathToGoal" ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
> The current planned path (published if triggered to plan).

#### 4.1.3 Parameters

**Note:** Only ROS parameters are mentioned. All others are explained in external documentation. The parameter "topic_root" can be found there!

"map_frame" (Default: "map")
> The name of the map frame.

"odom_frame" (Default: "odom")
> The name of the odometry frame.

"base_frame" (Default: "base_link")
> The name of the base frame.

"tf_update_rate" (Default: 20.0)
> The update rate [hz] of the map_frame -> odom_frame transformation broadcaster.

"map_save_period" (Default: 10.0)
> The period [seconds] after which a map is saved.

"map_file_path" (Default: "ratslam-latest.bmap")
> The file path where map is saved.

#### 4.1.4 Required tf Transforms

odom_frame -> base_frame
> Usually provided by the odometry system (e.g., the driver for the mobile base).

#### 4.1.5 Provided tf Transforms

map_frame -> odom_frame
> The current estimate of the robot's pose within the map frame.

### 4.2 ratslam_pc

The pose cell network is used to track odometry information based on movement, vision and recognition.

#### 4.2.1 Subscribed Topics

"odom" ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))
> The current odometry.

topic_root + "/LocalView/Template" (ratslam_ros/ViewTemplate)
> The current matched template.

#### 4.2.2 Published Topics

topic_root + "/PoseCell/TopologicalAction" (ratslam_ros/TopologicalMap)
> The topological action performed by network.

#### 4.2.3 Parameters

**Note:** Only ROS parameters are mentioned. All others are explained in external documentation.

"pcn_save_period" (Default: 10.0)
> The period [seconds] after which a pose cell network state is saved.

"pcn_file_path" (Default: "ratslam-latest.bpcn")
> The file path where pose cell network state is saved.

### 4.3 ratslam_lv

The local view matcher is used to match the camera templates against previous experienced ones. This node provides the matching of images.

#### 4.3.1 Subscribed Topics

"image" ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> ImageTransport image subscriber.

"camera_info" ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> ImageTransport camera info subscriber.

#### 4.3.2 Published Topics

topic_root + "/LocalView/Template" (ratslam_ros/ViewTemplate)
> The current matched template.

#### 4.3.3 Parameters

**Note:** Only ROS parameters are mentioned. All others are explained in external documentation. The parameter "topic_root" can be found there!

"lvm_save_period" (Default: 10.0)
> The period [seconds] after which a local view matcher state is saved.

"lvm_file_path" (Default: "ratslam-latest.blvm")
> The file path where local view matcher state is saved.

### 4.4 ratslam_vo

The visual odometry node. This node can be used to generate an odometry source from monocular camera input.
**Note:** Using wheel based odometry is more accurate in most cases.

#### 4.4.1 Subscribed Topics

"image" ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
> ImageTransport image subscriber.

"camera_info" ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
> ImageTransport camera info subscriber.

#### 4.4.2 Published Topics

"odom" ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))
> The current odometry generated from vision source.
