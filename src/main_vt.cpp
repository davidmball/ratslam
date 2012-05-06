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

#include "ratslam/Visual_Template_Match.h"

#if HAVE_IRRLICHT
#include "graphics/ViewTemplateScene.hpp"
ratslam::ViewTemplateScene *vts = NULL;
bool use_graphics;
#endif

boost::property_tree::ptree settings;

using namespace ratslam;

void image_callback(sensor_msgs::ImageConstPtr image, ros::Publisher * pub_vt)
{
  static ratslam_ros::ViewTemplate vt_output;
  static ratslam::Visual_Template_Match * vt = NULL;

  ROS_DEBUG_STREAM("VT:camera_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

  if (!vt)
  {
    boost::property_tree::ptree ratslam_settings;
    get_setting_child(ratslam_settings, settings, "ratslam", true);
    ratslam_settings.put("image_width", image->width);
    ratslam_settings.put("image_height", image->height);
    vt = new ratslam::Visual_Template_Match(ratslam_settings);

#ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    if (!::vts && use_graphics)
    ::vts = new ratslam::ViewTemplateScene(draw_settings, vt);
#endif
  }

  if (image->encoding == "bgr8")
    vt->on_image(&image->data[0], false);
  else
    vt->on_image(&image->data[0], true);

  vt_output.header.stamp = ros::Time::now();
  vt_output.header.seq++;
  vt_output.current_id = vt->get_current_vt();

  pub_vt->publish(vt_output);

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
    vts->draw_all();
  }
#endif
}

int main(int argc, char * argv[])
{

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";

  boost::property_tree::ptree general_settings;
  read_ini(argv[1], settings);

  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMViewTemplate");
  }
  ros::NodeHandle node;

  ros::Publisher pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/ViewTemplate/Template", 0);

  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe(topic_root + "/camera/image", 1, boost::bind(image_callback, _1, &pub_vt));

  ros::spin();

  return 0;
}
