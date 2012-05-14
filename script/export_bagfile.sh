#!/bin/bash    

rostopic echo -b ratslam_out.bag -p /stlucia/LocalView/Template > vt_id.dat
rostopic echo -b ratslam_out.bag -p /stlucia/PoseCell/TopologicalAction > em_id.dat
rostopic echo -b ratslam_out.bag -p /stlucia/ExperienceMap/RobotPose > pose.dat
rostopic echo -b ratslam_out.bag -p /stlucia/ExperienceMap/Map > map.dat

