/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
using namespace std;

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>

// em
#include "ratslam/Experience_Map.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include <topological_nav_msgs/TopologicalGraph.h>

#ifdef HAVE_IRRLICHT
#include "graphics/ExperienceMapScene.hpp"
ratslam::ExperienceMapScene *ems;
bool use_graphics;
#endif

using namespace ratslam;

void odo_callback(nav_msgs::OdometryConstPtr odo, ratslam::Experience_Map *em, ros::Publisher * pub_em)
{
  ROS_DEBUG_STREAM("EM:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();
    em->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
  }

  prev_time = odo->header.stamp;
  em->iterate();

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
	ems->update_scene();
	ems->draw_all();
  }
#endif
}

void action_callback(ratslam_ros::TopologicalActionConstPtr action, ratslam::Experience_Map *em, ros::Publisher * pub_em)
{
  ROS_DEBUG_STREAM("EM:action_callback{" << ros::Time::now() << "} action=" << action->action << " src=" << action->src_id << " dst=" << action->dest_id);
//  cout << "EM:action_callback{" << ros::Time::now() << "} action=" << action->action << " src=" << action->src_id << " dst=" << action->dest_id;

  switch (action->action)
  {
    case ratslam_ros::TopologicalAction::CREATE_NODE:
      em->on_create_experience(action->dest_id);
      em->on_set_experience(action->dest_id);
      break;

    case ratslam_ros::TopologicalAction::CREATE_EDGE:
      em->on_create_link(action->src_id, action->dest_id);
      em->on_set_experience(action->dest_id);
      break;

    case ratslam_ros::TopologicalAction::SET_NODE:
      em->on_set_experience(action->dest_id);
      break;

  }

  em->iterate();

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
	ems->update_scene();
	ems->draw_all();
  }
#endif


}

int main(int argc, char * argv[])
{
  if (argc < 2)
  {
    std::cout << "USAGE: " << argv[0] << " <config_file>" << std::endl;
    std::cin.get();
    exit(-1);
  }

  boost::property_tree::ptree settings, ratslam_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMExperienceMap");
  }
  ros::NodeHandle node;

  std::string topic_root = "";

  // em main
  ratslam::Experience_Map * em = new ratslam::Experience_Map(ratslam_settings);
  ros::Publisher pub_em = node.advertise<topological_nav_msgs::TopologicalGraph>(topic_root + "/ExperienceMap/Graph", 1);
  // TODO also publishes robot's pose

  ros::Subscriber sub_odometry2 = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 1, boost::bind(odo_callback, _1, em, &pub_em), ros::VoidConstPtr(),
                                                                     ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_action = node.subscribe<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 0, boost::bind(action_callback, _1, em, &pub_em),
                                                                              ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
	ems = new ratslam::ExperienceMapScene(draw_settings, em);
  }
#endif

  // TODO somewhere we need a   get_experience_map()->set_kidnapped();

  // TODO: the draw all should go in here, only the updates in teh callback
  ros::spin();

  return 0;
}
