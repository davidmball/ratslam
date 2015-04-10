**RatSLAM** is a bio-inspired simultaneous localisation and mapping (SLAM) system. Based on continous attractor network dynamics, RatSLAM is capable of mapping by closing loops to correct odometry error.

This version was written by David Ball and Scott Heath for use with the [iRat](http://itee.uq.edu.au/~dball/iRat/) in the paper [A Navigating Rat Animat](http://eprints.qut.edu.au/40389/1/c40389.pdf).

The original RatSLAM algorithm was designed and implemented on Pioneer robots by Michael Milford and Gordon Wyeth (see [RatSLAM: a hippocampal model for simultaneous localization and mapping](http://eprints.qut.edu.au/37593/1/c37593.pdf)).

The C++ RatSLAM implementation is currently being used to power the iRat robot when it is [online](http://ratslam.itee.uq.edu.au/live.html) and in recent ports of the [Lingodroids project](http://itee.uq.edu.au/~ruth/Lingodroids.htm) to use the iRat.

Despite only testing this RatSLAM version with the iRat, the library interfaces have been designed to be portable to other robots.

## Dependencies ##

The RatSLAM implementation depends only on [boost](http://www.boost.org) although the graphics library depends on [Irrlicht](http://irrlicht.sourceforge.net) and the example files use [OpenCV](http://opencv.willowgarage.com) to read videos.

To build RatSLAM and the example program you will need:
  * [CMake](http://www.cmake.org/)
  * [boost](http://www.boost.org), in particular boost\_date\_time and boost\_serialization (must be version 1.41 or later)
  * [Irrlicht](http://irrlicht.sourceforge.net)
  * [OpenCV](http://opencv.willowgarage.com), with FFmpeg support for video formats to be used.

## Build Instructions ##

Checkout the source from svn:

```
svn checkout http://ratslam.googlecode.com/svn/trunk/ ratslam-read-only
```

Launch the CMake GUI. Point the sources directory to the checkout directory and make a new directory somewhere for the build files (just create a new 'build' next to CMakeLists.txt in the checkout directory). Point the build directory at the new folder.

Press the configure button and select compilers. If boost or Irrlicht or OpenCV is not found, set the include and library directories for these.

Press configure again, and if that worked, press the generate button. Now either run make in the build directory, or if using visual studio open the project file and build the solution.

We are aware that the build system needs some more work, so if finding dependencies is not working properly, a workaround is to edit the CMakeLists.txt file, remove the `FindPackage` lines that are not working and manually set the include and lib directories by adding:

```
include_directories(dir1 dir2 ...)
link_directories(dir1 dir2 ...)
```

Certain codecs are missing from the FFmpeg binaries that ship with Ubuntu, so OpenCV and FFmpeg may have to be installed from source. There is a guide to installing the latest FFmpeg [here](http://ubuntuforums.org/showthread.php?t=786095).

## Fast Tutorial ##

After loading properties and then instantiating the RatSLAM class:

```
boost::property_tree::ptree properties;
boost::property_tree::read_ini("properties_file.txt", properties);
ratslam::Ratslam ratslam = new ratslam::Ratslam(properties);
```

then just three functions are used to control RatSLAM:

```
ratslam->set_view_rgb(const unsigned char * view_rgb); // where view_rgb is an array width x height x 3 of image pixels
ratslam->set_odom(double vtrans, double vrot); // where vtrans is translational speed of robot and vrot is angular velocity
ratslam->set_delta_time(double delta_time_s); // where delta_time_s is the time between this view and the last view
```

a single function call runs the RatSLAM algorithm:

```
ratslam->process();
```

finally position and mapping information may be obtained from the experience map:

```
ratslam::Experience_Map experience_map = ratslam->get_experience_map();
int current_experience_id = experience_map->get_current_id();
int experience_count = experience_map->get_num_experiences();

ratslam::Experience * robot_experience = experience_map->get_experience(current_experience_id);

printf("MAP: Robot at %f, %f\n", robot_experience->x_m, robot_experience->y_m);

for (int i = 0; i < experience_count; i++)
{
    ratslam::Experience * experience1 = experience_map->get_experience(i);

    // nodes in the experience map
    printf("MAP: Experience at %f, %f\n", experience1->x_m, experience1->y_m);

    for (int j = 0; j < experience1->links_from.size(); j++)
    {
        // links in the experience map
        ratslam::Link * link = experience_map->get_link( experience1->links_from[j]);
        ratslam::Experience * experience2 = experience_map->get_experience(link->exp_to_id);

        printf("MAP: Link to experience at %f, %f\n", experience2->x_m, experience2->y_m);
        
    }
}

```