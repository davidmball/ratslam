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

// pc
#include "ratslam/Pose_Cell_Network.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>

#if HAVE_IRRLICHT
#include "graphics/PoseCellsScene.hpp"
ratslam::PoseCellsScene *pcs;
bool use_graphics;
#endif

#include <ratslam_ros/ViewTemplate.h>


using namespace ratslam;

ratslam_ros::TopologicalAction pc_output;

void odo_callback(nav_msgs::OdometryConstPtr odo, ratslam::Pose_Cell_Network *pc, ros::Publisher * pub_pc)
{
  ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();

    pc_output.src_id = pc->get_current_exp_id();
    //pc->on_odo(0.1, 0.2, 1.0/20.0);
    pc->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
    pc_output.action = pc->get_action();
    if (pc_output.action != ratslam::Pose_Cell_Network::NO_ACTION)
    {
      pc_output.header.stamp = ros::Time::now();
      pc_output.header.seq++;
      pc_output.dest_id = pc->get_current_exp_id();
      pub_pc->publish(pc_output);
      ROS_DEBUG_STREAM("PC:action_publish{odo}{" << ros::Time::now() << "} action{" << pc_output.header.seq << "}=" <<  pc_output.action << " src=" << pc_output.src_id << " dest=" << pc_output.dest_id);
    }

#ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
		pcs->update_scene();
		pcs->draw_all();
	}
#endif
  }
  prev_time = odo->header.stamp;
}

void template_callback(ratslam_ros::ViewTemplateConstPtr vt, ratslam::Pose_Cell_Network *pc, ros::Publisher * pub_pc)
{
  ROS_DEBUG_STREAM("PC:vt_callback{" << ros::Time::now() << "} seq=" << vt->header.seq << " id=" << vt->current_id);

  pc_output.src_id = pc->get_current_exp_id();
  pc->on_view_template(vt->current_id);
  pc_output.action = pc->get_action();

//  cout << "Action: " << pc_output.action << " src=" << pc_output.src_id << " dst=" << pc_output.dest_id << endl;

  if (pc_output.action != ratslam::Pose_Cell_Network::NO_ACTION)
  {
    pc_output.dest_id = pc->get_current_exp_id();
    pc_output.header.stamp = ros::Time::now();
    pc_output.header.seq++;
    pub_pc->publish(pc_output);
    ROS_DEBUG_STREAM("PC:action_publish{vt}{" << ros::Time::now() << "} action{" << pc_output.header.seq << "}=" <<  pc_output.action << " src=" << pc_output.src_id << " dest=" << pc_output.dest_id);
  }

#ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
		pcs->update_scene();
		pcs->draw_all();
	}
#endif

}

int main(int argc, char * argv[])
{
  if (argc < 2)
  {
    std::cout << "USAGE: " << argv[0] << " <config_file>" << std::endl;
    exit(-1);
  }

  boost::property_tree::ptree settings, ratslam_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);
 

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMPoseCells");
  }
  ros::NodeHandle node;

  std::string topic_root = "";

  // pc main
  ratslam::Pose_Cell_Network * pc = new ratslam::Pose_Cell_Network(ratslam_settings);
  ros::Publisher pub_pc = node.advertise<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 0);

  ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 1, boost::bind(odo_callback, _1, pc, &pub_pc), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_template = node.subscribe<ratslam_ros::ViewTemplate>(topic_root + "/ViewTemplate/Template", 0, boost::bind(template_callback, _1, pc, &pub_pc),
                                                                           ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
	pcs = new ratslam::PoseCellsScene(draw_settings, pc);
  }
#endif
  // TODO: the draw all should go in here, only the updates in teh callback
  ros::spin();

  return 0;
}
