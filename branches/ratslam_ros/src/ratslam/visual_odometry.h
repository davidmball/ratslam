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

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ratslam {

class VisualOdometry
{
public:

  VisualOdometry(ptree settings);

  void on_image(const unsigned char * data, bool greyscale, unsigned int image_width, unsigned int image_height, double *vtrans_ms, double *vrot_rads);

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & IMAGE_HEIGHT;
      ar & IMAGE_WIDTH;

      ar & VTRANS_IMAGE_X_MIN;
      ar & VTRANS_IMAGE_X_MAX;
      ar & VTRANS_IMAGE_Y_MIN;
      ar & VTRANS_IMAGE_Y_MAX;

      ar & VROT_IMAGE_X_MIN;
      ar & VROT_IMAGE_X_MAX;
      ar & VROT_IMAGE_Y_MIN;
      ar & VROT_IMAGE_Y_MAX;

      ar & CAMERA_FOV_DEG;
      ar & CAMERA_HZ;

      ar & VTRANS_SCALING;
      ar & VTRANS_MAX;

      ar & vtrans_profile;
      ar & vrot_profile;

      ar & vtrans_prev_profile;
      ar & vrot_prev_profile;

      ar & first;
    }

private:

  VisualOdometry() {;}

  int IMAGE_HEIGHT;
  int IMAGE_WIDTH;

  int VTRANS_IMAGE_X_MIN;
  int VTRANS_IMAGE_X_MAX;
  int VTRANS_IMAGE_Y_MIN;
  int VTRANS_IMAGE_Y_MAX;

  int VROT_IMAGE_X_MIN;
  int VROT_IMAGE_X_MAX;
  int VROT_IMAGE_Y_MIN;
  int VROT_IMAGE_Y_MAX;

  double CAMERA_FOV_DEG;
  double CAMERA_HZ;

  double VTRANS_SCALING;
  double VTRANS_MAX;

  std::vector<double> vtrans_profile;
  std::vector<double> vrot_profile;

  std::vector<double> vtrans_prev_profile;
  std::vector<double> vrot_prev_profile;

  bool first;

  void visual_odo(double *data, unsigned short width, double *olddata, double *vtrans_ms, double *vrot_rads);

  void convert_view_to_view_template(double *current_view, const unsigned char *view_rgb, bool grayscale, int X_RANGE_MIN, int X_RANGE_MAX, int Y_RANGE_MIN, int Y_RANGE_MAX);

};

}; // namespace ratslam
