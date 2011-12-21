RatSLAM lite C/C++/MATLAB versions
written and copyright 2011 
by Dr David Ball (dball@itee.uq.edu.au) and Mr Scott Heath (scott.heath@uqconnect.edu.au)
This program is free software distributed under the terms of the GNU GPL. See license.txt for full details.

**About**

**Citing the work**
Publication to come

**Using**
All parameters should be settable in the config.txt file.

This program has been tested on Windows 7 and Ubuntu.

The program depends on other work including Boost, OpenCV, irrlicht, ffmpeg, and our custom gri library.

**Ground Truth Tracking**
The program has the capability of ground truth tracking of the robot from an overhead camera. This capability can be set by defining GROUND_TRUTH_TRACKING.
This work depends on cvBlobs and OpenCV.

**Future**
Major features to be added:
* Goal Memory

**RatSLAM**
The RatSLAM algorithm is by Dr Michael Milford and Prof Gordon Wyeth ([michael.milford, gordon.wyeth]@qut.edu.au)

There are two relevant papers to cite depending on your application:
For outdoor vision only SLAM: 
Milford, M.J., Wyeth, G.F. (2008) Mapping a Suburb With a Single Camera Using a Biologically Inspired SLAM System, IEEE Transactions on Robotics.

For robot SLAM:
Milford, M.J., Wyeth, G.F. (2010) Persistent Navigation and Mapping using a Biologically Inspired SLAM System, the International Journal of Robotics Research