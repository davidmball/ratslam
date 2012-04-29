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

#include "ratslam/Visual_Template_Match.h"

#if HAVE_IRRLICHT
#include "graphics/ViewTemplateScene.hpp"
ratslam::ViewTemplateScene *vts = NULL;
bool use_graphics;
#endif

ratslam::Visual_Template_Match * vt = NULL;


boost::property_tree::ptree settings;

using namespace ratslam;

void camera_callback(sensor_msgs::ImageConstPtr image, ratslam::Visual_Template_Match *vt, ros::Publisher * pub_vt)
{
  static ratslam_ros::ViewTemplate vt_output;

  ROS_DEBUG_STREAM("VT:camera_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

  if (!::vt)
  {
    boost::property_tree::ptree ratslam_settings;
    get_setting_child(ratslam_settings, settings, "ratslam", true);
    ratslam_settings.put("image_width", image->width);
    ratslam_settings.put("image_height", image->height);
    ::vt = new ratslam::Visual_Template_Match(ratslam_settings);
        
#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    if (!::vts && use_graphics)
      ::vts = new ratslam::ViewTemplateScene(draw_settings, ::vt);
#endif
  }

  if (image->encoding == "bgr8")
    ::vt->on_image(&image->data[0], false);
  else
    ::vt->on_image(&image->data[0], true);

  vt_output.header.stamp = ros::Time::now();
  vt_output.header.seq++;
  vt_output.current_id = ::vt->get_current_vt();

  pub_vt->publish(vt_output);

#ifdef HAVE_IRRLICHT
	if (use_graphics)
	{
		vts->draw_all();
	}
#endif
}

void compcamera_callback(sensor_msgs::CompressedImageConstPtr image, ratslam::Visual_Template_Match *vt, ros::Publisher * pub_vt)
{
  static ratslam_ros::ViewTemplate vt_output;

  ROS_DEBUG_STREAM("VT:camera_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

  cv::Mat image_mat_jpeg(image->data.size(), 1, CV_8UC1, (void*) &image->data[0]);
  cv::Mat imdecoded = cv::imdecode(image_mat_jpeg, 1);

  if (!::vt)
  {
    boost::property_tree::ptree ratslam_settings;
    get_setting_child(ratslam_settings, settings, "ratslam", true);
    ratslam_settings.put("image_width", imdecoded.cols);
    ratslam_settings.put("image_height", imdecoded.rows);
    ::vt = new ratslam::Visual_Template_Match(ratslam_settings);

#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    if (!::vts && use_graphics)
      ::vts = new ratslam::ViewTemplateScene(draw_settings, ::vt);
#endif
  }

  ::vt->on_image(imdecoded.data, false);

#if 0
  if (image->encoding == "bgr8")
    ::vt->on_image(&image->data[0], false);
  else
    ::vt->on_image(&image->data[0], true);
#endif

  vt_output.header.stamp = ros::Time::now();
  vt_output.header.seq++;
  vt_output.current_id = ::vt->get_current_vt();

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
    std::cout << "USAGE: " << argv[0] << " <config_file>" << std::endl;
    exit(-1);
  }

//  boost::property_tree::ptree settings, ratslam_settings, draw_settings;
  read_ini(argv[1], settings);

 // get_setting_child(ratslam_settings, settings, "ratslam", true);
 // get_setting_child(draw_settings, settings, "draw", true);

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMViewTemplate");
  }
  ros::NodeHandle node;

  std::string topic_root = "";

  ros::Publisher pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/ViewTemplate/Template", 0);

  ros::Subscriber sub_camera = node.subscribe<sensor_msgs::Image>(topic_root + "/camera/image_rect", 1,
                                                                  boost::bind(camera_callback, _1, vt, &pub_vt),
                                                                  ros::VoidConstPtr(),
                                                                  ros::TransportHints().tcpNoDelay());

  ros::Subscriber sub_compcamera = node.subscribe<sensor_msgs::CompressedImage>(topic_root + "/camera/compressedimage_rect", 1,
                                                                  boost::bind(compcamera_callback, _1, vt, &pub_vt),
                                                                  ros::VoidConstPtr(),
                                                                  ros::TransportHints().tcpNoDelay());

  // TODO: the draw all should go in here, only the updates in teh callback
  ros::spin();

  return 0;
}
