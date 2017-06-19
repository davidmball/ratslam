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

#include <boost/property_tree/ini_parser.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ratslam_ros/TopologicalAction.h>
#include <ratslam_ros/ViewTemplate.h>

#include "utils/utils.h"
#include "ratslam/posecell_network.h"

#if HAVE_IRRLICHT

#include "graphics/posecell_scene.h"

ratslam::PosecellScene* pcs;
bool use_graphics;
#endif

using namespace std;
using namespace ratslam;

ratslam::PosecellNetwork* pc;
ros::Publisher pub_pc;

ratslam_ros::TopologicalAction pc_output;
std::string pcn_file_path = std::string("ratslam-latest.bpcn");
double pcn_save_period = 10.0;

void odo_callback(nav_msgs::OdometryConstPtr odo)
{
  ROS_DEBUG_STREAM("PC:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq
                                      << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);

  static ros::Time prev_time(0);

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();

    pc_output.src_id = pc->get_current_exp_id();
    pc->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
    pc_output.action = pc->get_action();
    if (pc_output.action != ratslam::PosecellNetwork::NO_ACTION)
    {
      pc_output.header.stamp = ros::Time::now();
      pc_output.header.seq++;
      pc_output.dest_id = pc->get_current_exp_id();
      pc_output.relative_rad = pc->get_relative_rad();
      pub_pc.publish(pc_output);
      ROS_DEBUG_STREAM("PC:action_publish{odo}{" << ros::Time::now() << "} action{" << pc_output.header.seq
                                                 << "}=" << pc_output.action << " src=" << pc_output.src_id
                                                 << " dest=" << pc_output.dest_id);
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

void template_callback(ratslam_ros::ViewTemplateConstPtr vt)
{
  ROS_DEBUG_STREAM("PC:vt_callback{" << ros::Time::now() << "} seq=" << vt->header.seq << " id=" << vt->current_id
                                     << " rad=" << vt->relative_rad);

  pc->on_view_template(vt->current_id, vt->relative_rad);

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
    pcs->update_scene();
    pcs->draw_all();
  }
#endif
}

void save_pcn_timer_callback(const ros::TimerEvent& event)
{
  // create and open a character archive for output
  std::ofstream ofs(pcn_file_path.c_str());

  // save map to archive file
  {
    boost::archive::binary_oarchive binary_archive(ofs);
    // write class instance to archive
    pc->save(binary_archive, 1);
    // archive and stream closed when destructors are called
  }
}

bool load_pcn(const std::string& file_path)
{
  try
  {
    // create and open an archive for input
    std::ifstream ifs(file_path.c_str());
    boost::archive::binary_iarchive binary_archive(ifs);
    // read class state from archive
    pc->load(binary_archive, 1);
    // archive and stream closed when destructors are called
    return true;
  }
  catch (...)
  {
    return false;
  }
}

int main(int argc, char* argv[])
{
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";
  boost::property_tree::ptree settings, ratslam_settings, general_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  // initialize ROS node
  ros::init(argc, argv, "RatSLAMPoseCells");

  // setup public and private nodehandles
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  // params
  priv_node.param("pcn_save_period", pcn_save_period, pcn_save_period);
  priv_node.param("pcn_file_path", pcn_file_path, pcn_file_path);

  // check backward compatibility with configs that have topic_root set
  std::string odom_topic = "odom";
  if (!topic_root.empty())
  {
    odom_topic = topic_root + "/odom";
  }

  // create PoseCellNetwork object
  pc = new ratslam::PosecellNetwork(ratslam_settings);

  // try to load pcn
  if (!load_pcn(pcn_file_path))
  {
    ROS_WARN("Could not load PoseCellNetwork from file \"%s\"", pcn_file_path.c_str());
  }

  // pubs
  pub_pc = node.advertise<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 0);

  // subs
  ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(odom_topic, 0, odo_callback, ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_template =
      node.subscribe<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0, template_callback,
                                                ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  // timers
  ros::Timer pcn_save_timer = node.createTimer(ros::Duration(pcn_save_period), save_pcn_timer_callback);

#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
    pcs = new ratslam::PosecellScene(draw_settings, pc);
  }
#endif

  ros::spin();

  return 0;
}
