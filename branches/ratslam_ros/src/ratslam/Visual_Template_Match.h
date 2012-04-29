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
/*
 * The Visual_Template_Match class is responsible for looking up images
 * and finding the best matching image and a score for how well the image
 * matches.
 */

#ifndef _VISUAL_TEMPLATE_MATCH_H_
#define _VISUAL_TEMPLATE_MATCH_H_

#include <cstring>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include <vector>
#include <fstream>

#define _USE_MATH_DEFINES
#include "math.h"

// todo: replace this with iostream. update - replace all this with the ros stream macros
#include <stdio.h>

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;


#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

namespace ratslam
{
	
// TODO as this is now an id and data consider replacing with a vector
typedef struct td_visual_template
{
	unsigned int id;
	std::vector<double> data;

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & id;
	ar & data;
	}

} Visual_Template;


class Visual_Template_Match
{
public:
	friend class ViewTemplateScene;

	Visual_Template_Match(ptree settings);
	
	~Visual_Template_Match();

	// create and add a visual template to the collection
	int create_template();
	
	// compare a visual template to all the stored templates, allowing for 
	// slen pixel shifts in each direction
	// returns the matching template and the MSE
	void compare(double &vt_err, unsigned int &vt_match_id);

	void on_image(const unsigned char * view_rgb, bool greyscale);

	int get_number_of() { return templates.size(); }


    double get_vt_match_threshold() const { return VT_MATCH_THRESHOLD; }

	int get_current_vt() { return current_vt; }

        void set_current_vt(int id) {
                if (current_vt != id)
                   prev_vt = current_vt;

                current_vt = id;
        }


//	const unsigned char * get_view_rgb() {	return view_rgb; }
	void convert_view_to_view_template(bool grayscale = false);

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & VT_SHIFT_MATCH;
	ar & VT_STEP_MATCH;
	ar & VT_MATCH_THRESHOLD;
	ar & TEMPLATE_SIZE;
	ar & IMAGE_WIDTH;
	ar & IMAGE_HEIGHT;
	ar & IMAGE_VT_X_RANGE_MIN;
	ar & IMAGE_VT_X_RANGE_MAX;
	ar & IMAGE_VT_Y_RANGE_MIN;
	ar & IMAGE_VT_Y_RANGE_MAX;
	ar & TEMPLATE_X_SIZE;
	ar & TEMPLATE_Y_SIZE;
	ar & PATCH_NORMALISATION;
	ar & MIN_PATCH_NORMALISATION_STD;

	ar & templates;
	ar & current_view;
	ar & image_size;
	ar & current_vt;
	ar & vt_error;
	ar & prev_vt;

	}



private:
    friend class boost::serialization::access;

	Visual_Template_Match() {;}
	void clip_view_x_y(int &x, int &y);

	int VT_SHIFT_MATCH;
	int VT_STEP_MATCH;

	double VT_MATCH_THRESHOLD;
	int TEMPLATE_SIZE;
	int IMAGE_WIDTH;
	int IMAGE_HEIGHT;
	int IMAGE_VT_X_RANGE_MIN;
	int IMAGE_VT_X_RANGE_MAX;
	int IMAGE_VT_Y_RANGE_MIN;
	int IMAGE_VT_Y_RANGE_MAX;
	int TEMPLATE_X_SIZE;
	int TEMPLATE_Y_SIZE;
	int PATCH_NORMALISATION;
	double MIN_PATCH_NORMALISATION_STD;

	std::vector<Visual_Template> templates;
	std::vector<double> current_view;

	int image_size;
	int current_vt;
	double vt_error;
	int prev_vt;

	const unsigned char *view_rgb;
	bool greyscale;

};
}

#endif // _VISUAL_TEMPLATE_MATCH_H_
