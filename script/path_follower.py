#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This file implements a naive path following controller.

Note:   As the controller does not care about collision
        avoidance so the path must be carefully planned
        and so to say "drivable"! Changing environment
        and moving object will be a game-killer.

Copyright {2017} {Peter Rudolph}

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""

# ROS
import numpy as np
import rospy
from tf import ExtrapolationException, LookupException
from tf.listener import TransformListener

# ROS msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


class PathFollowController(object):
    """
    This implements a naive path following controller.

    """

    def __init__(self):
        # vars
        self._accept_path = True
        self._path = Path()

        # params
        self._update_rate = rospy.get_param('~update_rate', 10)
        self._base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self._min_lin_vel = rospy.get_param('~min_lin_vel', 0.2)
        self._max_lin_vel = rospy.get_param('~max_lin_vel', 1.0)
        self._min_ang_vel = rospy.get_param('~min_ang_vel', 15. * np.pi / 180.)
        self._max_ang_vel = rospy.get_param('~max_ang_vel', 45. * np.pi / 180.)
        self._min_goal_dist = rospy.get_param('~min_goal_dist', 0.05)
        self._min_drive_angle = rospy.get_param('~min_drive_angle', 25.0 * np.pi / 180.)

        # tf
        self._tfl = TransformListener()

        # pubs
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # subs
        self._path_sub = rospy.Subscriber('path', Path, self._path_cb, queue_size=1)

    def _path_cb(self, msg):
        """
        The path callback updating the current path.

        :type   msg:    Path
        :param  msg:    The path message.

        """
        if self._accept_path:
            # setup current path
            self._path = msg
            # do not accept new paths
            self._accept_path = False
            # log
            rospy.loginfo('Received new path!')

    def run(self, update_rate=20):
        """
        Continuously drives to goals in current path.

        If goal is reached it is removed from current path.

        :type   update_rate:    int
        :param  update_rate:    The loop update rate.

        """
        r = rospy.Rate(update_rate)
        while not rospy.is_shutdown():

            # do nothing else than accepting new paths
            # if we do not have a current path
            if len(self._path.poses) == 0:
                # accept new paths
                self._accept_path = True
                r.sleep()
                continue

            # get current goal
            goal_pose_ = self._path.poses[-1]

            # try transform goal to base frame
            try:
                goal_pose = self._tfl.transformPose(self._base_frame, goal_pose_)
            except (ExtrapolationException, LookupException):
                rospy.logwarn('Could not get transform {} -> {}.'.format(goal_pose_.header.frame_id, self._base_frame))
                r.sleep()
                continue

            # distance to goal pose
            x = goal_pose.pose.position.x
            y = goal_pose.pose.position.y
            dist = np.sqrt(x**2+y**2)
            angle = np.math.atan2(y, x)

            # remove goal if we are close too
            # TODO: turn towards last goals heading
            if dist < self._min_goal_dist:
                self._path.poses.pop()

            # setup commanded velocities
            cmd_vel = Twist()
            cmd_vel.angular.z = angle

            # limit turn rate (angular velocity)
            if abs(angle) > self._max_ang_vel:
                cmd_vel.angular.z = max(min(dist, self._max_ang_vel), self._min_ang_vel) * np.sign(cmd_vel.angular.z)

            # only drive if we are almost facing the goal
            if abs(angle) < self._min_drive_angle:
                # limit speed to distance or max speed
                cmd_vel.linear.x = max(min(dist, self._max_lin_vel), self._min_lin_vel)

            # publish velocities
            self._cmd_vel_pub.publish(cmd_vel)

            # sleep
            r.sleep()

# initialize ROS node
rospy.init_node('naive_path_follower')
# initialize controller
pfc = PathFollowController()
# loop
pfc.run()
