The ROS version of RatSLAM has been designed as a more modular version than the other library in this project - the algorithm has been divided into 3 individual RatSLAM ROS nodes and one visual odometry node. This version is referred to as OpenRatSLAM.

OpenRatSLAM is based on the RatSLAM C++ library written by David Ball and Scott Heath, with support and additions by Michael Milford. The RatSLAM algorithm was originally designed and implemented by Michael Milford and Gordon Wyeth.

OpenRatSLAM has only been tested on Ubuntu and as ROS support for platforms other than Linux is still limited, this will probably not change.

For a full description of ROS terminology have a look [here](http://www.ros.org).

## Dependencies ##

OpenRatSLAM depends on ROS packages:
opencv2 and topological\_nav\_msgs
and also on 3D graphics library [Irrlicht](http://irrlicht.sourceforge.net)
Irrlicht can be installed on Ubuntu with apt-get
```
sudo apt-get install libirrlicht-dev libopencv-dev ros-melodic-tf ros-melodic-image-transport
```
OpenRatSLAM currently depends on Ubuntu's OpenCV package. It can also be installed with apt-get
```
sudo apt-get install libopencv-dev
```

## Build Instructions ##

This instructions describe how to build ratslam\_ros with catkin. The old rosbuild version can be found [here](RatSLAMROS_rosbuild.md). These instructions have last been successfully tested with Ubuntu 12.04 and ROS Groovy.

Setup ROS catkin workspace by typing:
```
source /opt/ros/groovy/setup.bash
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd..
catkin_make
source devel/setup.bash
```
The last command setup your development environment, defining all the needed environment variables. A more detailed description of how to create a catkin workspace can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Checkout the source from SVN into ~/catkin\_ws/src:
```
cd ~/catkin_ws/src
```
~~svn checkout http://ratslam.googlecode.com/svn/branches/ratslam_ros ratslam_ros~~
```
git clone https://github.com/davidmball/ratslam.git ratslam_ros
cd ratslam_ros
git checkout ratslam_ros
```

Then build OpenRatSLAM:
```
cd ~/catkin_ws
catkin_make
```

## Running RatSLAM ##

To use one of the provided pre-packaged bag files, download the bag file for either:
  * iRat in Aus
  * Car in St Lucia
  * Oxford new college dataset

all are available [here](https://wiki.qut.edu.au/display/cyphy/OpenRatSLAM+datasets)

and place the dataset in the OpenRatSLAM directory. Note these are moderately large files (~1 Gig).

Run the dataset and RatSLAM by typing either
```
roslaunch ratslam_ros irataus.launch
rosbag play irat_aus_28112011.bag
```
or
```
roslaunch ratslam_ros stlucia.launch
rosbag play stlucia_2007.bag
```
or
```
roslaunch ratslam_ros oxford_newcollege.launch
rosbag play oxford_newcollege.bag
```

Note that after you run the first line, a number of blank windows should pop up. Then after you play back the bagfile (second line), these windows should become populated with useful RatSLAM visualizations.

## Using Custom Bag Files (or Real Robots) ##

Bag files (or real robots) must provide at least images for RatSLAM on topics /`<`topic\_root`>`/camera/image `[sensor_msgs::Image]` or /`<`topic\_root`>`/camera/image/compressed `[sensor_msgs::CompressedImage]` (refer to [ROS image transport](http://www.ros.org/wiki/image_transport)).
Odometry may also be provided on topic /`<`topic\_root`>`/odom `[nav_msgs::Odometry]`, or can be estimated using visual flow from the image data.
Aside from the bag file, two other files are required: a config file and a launch file. An example config file is shown below (used for the St. Lucia data set):
```
[general]
topic_root=stlucia
                                                	
[ratslam]
image_crop_x_min=40
image_crop_x_max=600
image_crop_y_min=150
image_crop_y_max=300
vt_match_threshold = 0.085
template_x_size=60
template_y_size=10
vt_shift_match = 5
vt_step_match = 1
vt_normalisation = 0.4
pc_vt_restore=0.04

pc_cell_x_size = 2
pc_dim_xy = 30
exp_delta_pc_threshold = 1.0
pc_vt_inject_energy = 0.15
vt_active_decay = 1.0

exp_loops=20


[visual_odometry]
vtrans_image_x_min=195
vtrans_image_x_max=475
vtrans_image_y_min=270
vtrans_image_y_max=430

vrot_image_x_min=195
vrot_image_x_max=475
vrot_image_y_min=75
vrot_image_y_max=240

camera_fov_deg=53
camera_hz = 10
vtrans_scaling = 1000
vtrans_max=20

[draw]
enable=1
vt_window_width=640
vt_window_height=900
exp_map_size=500
posecells_size=250
media_path=../src/media
image_file=irat_sm.tga
```

The topic\_root in a config file may be set to any value as long as it matches the `<`topic\_root`>` paths specified in the bag file. The config file also contains RatSLAM options, visual odometry options and drawing options.

The launch file specifies which nodes of RatSLAM should be run and which config file to use. An example launch file is shown below(also used for the St. Lucia dataset):
```

<launch>

	<machine name="local_alt" address="localhost" default="true" />
	
	<node name="RatSLAMLocalViewCells" pkg="ratslam_ros" type="ratslam_lv" args="$(find ratslam_ros)/config/config_stlucia.txt _image_transport:=compressed" cwd="node" required="true" />
	<node name="RatSLAMPoseCells" pkg="ratslam_ros" type="ratslam_pc" args="$(find ratslam_ros)/config/config_stlucia.txt _image_transport:=compressed" cwd="node" required="true" />
	<node name="RatSLAMExperienceMap" pkg="ratslam_ros" type="ratslam_em" args="$(find ratslam_ros)/config/config_stlucia.txt _image_transport:=compressed" cwd="node" required="true" />
	<node name="RatSLAMVisualOdometry" pkg="ratslam_ros" type="ratslam_vo" args="$(find ratslam_ros)/config/config_stlucia.txt _image_transport:=compressed" cwd="node" required="true" />
	
	<node pkg="rqt_plot" type="rqt_plot" name="plot_vt_em" args="/stlucia/LocalView/Template/current_id,/stlucia/PoseCell/TopologicalAction/dest_id" />
	<node pkg="rosbag" type="record" name="record" args="/stlucia/ExperienceMap/Map /stlucia/ExperienceMap/RobotPose /stlucia/LocalView/Template /stlucia/PoseCell/TopologicalAction -O ratslam_out.bag" />

</launch>
```

The config file name must be set to the config file used. The line with:
```
	<node name="RatSLAMVisualOdometry" pkg="ratslam_ros" type="ratslam_vo" args="$(find ratslam_ros)/config/config_stlucia.txt _image_transport:=compressed" cwd="node" required="true" />
```
needs to be commented out if providing odometry.

### Creating a bag file ###

**Loading a dataset into a bag file**

The easiest way to tune RatSLAM is using an offline dataset. Any robot providing camera images as sensor\_msgs/CompressedImage and odometry as nav\_msgs/Odometry can be used. The images and odometry must be in the form `<`my\_robot`>`/camera/image and `<`my\_robot`>`/odom.

To convert topic names to the correct format run:
```
rostopic echo <path/to/my/robot/camera> | rostopic pub <my_robot>/camera/image sensor_msgs/CompressedImage &
rostopic echo <path/to/my/robot/odom> | rostopic pub <my_robot>/odom nav_msgs/Odometry &
```

Start recording into a bag file:
```
rosbag record -O <my_robot>.bag <my_robot>/camera/image <my_robot>/odom
```

Start the robot and collect the dataset. Press Ctrl-C at the terminal to finish recording.


### Running the bag file ###

To run a custom bag file, a new config file and launch file are required.

**Creating a new config file**

In a terminal type

```
cd ratslam_ros
cp config/config_stlucia.txt config/config_<my_robot>.txt
gedit config/config_<my_robot>.txt
```

Change the first line
```
topic_root=stlucia
```
to
```
topic_root=<my_robot>
```

**Creating a new launch file**

In the same terminal type

```
cp stlucia.launch <my_robot>.launch
gedit <my_robot>.launch
```

Replace all references to "$(find ratslam\_ros)/config/config\_stlucia.txt" with "$(find ratslam\_ros)/config/config`_<`my\_robot`>`.txt"

Comment out the visual odometry node to prevent it from running.
Replace
```
<node name="RatSLAMVisualOdometry" pkg="ratslam_ros" type="ratslam_vo" args="$(find ratslam_ros)/config/config_<my_robot>.txt _image_transport:=compressed" cwd="node" required="true" />
```

with
```
<!-- <node name="RatSLAMVisualOdometry" pkg="ratslam_ros" type="ratslam_vo" args="$(find ratslam_ros)/config/config_<my_robot>.txt _image_transport:=compressed" cwd="node" required="true" />-->
```

**Running your dataset**

Your dataset can now be run the same way is the provided datsets:
```
roslaunch <my_robot>.launch
rosbag play <my_robot>.bag
```

**Using rviz**

The map created by OpenRatSLAM will be periodically published to rviz. To run rviz:
```
rosrun rviz rviz
```

Click on the "Add" button down the bottom left of the window.
Choose `"MarkerArray"` from the list.
In the field "Marker Array Topic" on the left, click on the button with 3 dots. Choose the topic `<my_robot>/ExperienceMap/MapMarker`


### Tuning parameters ###

Open the created config file
```
gedit config/config_<my_robot>.txt
```

Tweak...
