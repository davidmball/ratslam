/*
 * openRatSLAM
 *
 * main_lv - ROS interface bindings for the local view cells
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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ratslam_ros/ViewTemplate.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>

#include "ratslam/local_view_match.h"

#if HAVE_IRRLICHT

#include "graphics/local_view_scene.h"

ratslam::LocalViewScene* lvs = NULL;
bool use_graphics;
#endif

ros::Publisher pub_vt;

using namespace ratslam;
ratslam::LocalViewMatch* lv = NULL;
std::string lvm_file_path = std::string("ratslam-latest.blvm");
double lvm_save_period = 10.0;

void image_callback(sensor_msgs::ImageConstPtr image)
{
  ROS_DEBUG_STREAM("LV:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

  static ratslam_ros::ViewTemplate vt_output;

  lv->on_image(&image->data[0], !(image->encoding == "bgr8"), image->width, image->height);

  vt_output.header.stamp = ros::Time::now();
  vt_output.header.seq++;
  vt_output.current_id = lv->get_current_vt();
  vt_output.relative_rad = lv->get_relative_rad();

  pub_vt.publish(vt_output);

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
    lvs->draw_all();
  }
#endif
}

void save_lvm_timer_callback(const ros::TimerEvent& event)
{
  // create and open a character archive for output
  std::ofstream ofs(lvm_file_path.c_str());

  // save map to archive file
  {
    boost::archive::binary_oarchive binary_archive(ofs);
    // write class instance to archive
    binary_archive << *lv;
    // archive and stream closed when destructors are called
  }
}

bool load_lvm(const std::string& file_path)
{
  try
  {
    // create and open an archive for input
    std::ifstream ifs(file_path.c_str());
    boost::archive::binary_iarchive binary_archive(ifs);
    // read class state from archive
    binary_archive >> *lv;
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

  get_setting_child(general_settings, settings, "general", true);
  // backward compatibility, namespace or private nodehandle should do it more ROS like
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
  get_setting_child(ratslam_settings, settings, "ratslam", true);

  // initialize ROS node
  ros::init(argc, argv, "RatSLAMViewTemplate");

  // setup public and private nodehandles
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  // params
  priv_node.param("lvm_save_period", lvm_save_period, lvm_save_period);
  priv_node.param("lvm_file_path", lvm_file_path, lvm_file_path);

  // check backward compatibility with configs that have topic_root set
  std::string image_topic = "image";
  if (!topic_root.empty())
  {
    // prepend with topic with topic_root setting
    image_topic = topic_root + "/camera/image";
  }

  lv = new ratslam::LocalViewMatch(ratslam_settings);

  // try to load lvm
  if (!load_lvm(lvm_file_path))
  {
    ROS_WARN("Could not load LocalViewMatch from file \"%s\"", lvm_file_path.c_str());
  }

  // pubs
  pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0);

  // image transport
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe(image_topic, 0, image_callback);

  // timers
  ros::Timer lvm_save_timer = node.createTimer(ros::Duration(lvm_save_period), save_lvm_timer_callback);

#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
    lvs = new ratslam::LocalViewScene(draw_settings, lv);
#endif

  ros::spin();

  return 0;
}
