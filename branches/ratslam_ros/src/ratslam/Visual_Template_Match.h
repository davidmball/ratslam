/*
 * This file is part of the RatSLAM C/C++/MATLAB lite versions.
 *
 * This version copyright (C) 2011
 * David Ball (d.ball@itee.uq.edu.au), Scott Heath (scott.heath@uqconnect.edu.au)
 *
 * RatSLAM algorithm by:
 * Michael Milford and Gordon Wyeth ([michael.milford, gordon.wyeth]@qut.edu.au)
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

#pragma warning( disable: 4275 ) // problem between std::vector and log4cxx
#pragma warning( disable: 4251 ) // problem between std::vector and log4cxx

#include <cstring>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iostream>
#include <vector>
#include <fstream>

#define _USE_MATH_DEFINES
#include "math.h"

// todo: replace this with iostream
#include <stdio.h>

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;


#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>

//#define SUPER_EXPERIENCES

namespace ratslam
{
	

typedef struct td_visual_template
{
	unsigned int id;
	double x_pc, y_pc, th_pc;
	std::vector<double> column_sum;
	double decay;
	std::vector<int> exps;

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & id;
	ar & x_pc & y_pc & th_pc;
	ar & column_sum;
	ar & decay;
	ar & exps;
	}

} Visual_Template;

class ExperienceMapScene;


class Visual_Template_Match
{
public:
	friend class ViewTemplateScene;

	Visual_Template_Match(ptree settings);
	
	~Visual_Template_Match();

	// compares raw values not view templates
	double compare_views(double *vt1, double *vt2, int size);

	// create and add a visual template to the collection
	int create_template(double x_pc, double y_pc, double th_pc);
	
	// compare a visual template to all the stored templates, allowing for 
	// slen pixel shifts in each direction
	// returns the matching template and the MSE
	void compare(double &vt_err, unsigned int &vt_match_id);

	double compare_two(int vt_id1, int vt_id2);
	
	void do_log(int frame, double *column_sum, int IMAGE_WIDTH);

	int get_number_of() { return templates.size(); }

    int get_vt_shift_match() const { return VT_SHIFT_MATCH; }
    int get_vt_step_match() const { return VT_STEP_MATCH; }

    double get_vt_active_delay() const { return VT_ACTIVE_DECAY; }
    double get_vt_match_threshold() const { return VT_MATCH_THRESHOLD; }

	double get_decay(int id) { return templates[id].decay; }

	int get_current_vt() { return current_vt; }
	void set_current_vt(int id) { 
		if (current_vt != id)
		{
			prev_vt = current_vt;
			templates[id].decay = VT_ACTIVE_DECAY;	
		} else
			templates[id].decay += VT_ACTIVE_DECAY;

		current_vt = id; 
	} 

	unsigned int get_current_exp_size() { return templates[current_vt].exps.size(); }
	unsigned int get_current_exp_link(int id) { return templates[current_vt].exps[id]; }
	void add_exp_to_current(unsigned int id) { templates[current_vt].exps.push_back(id); }

	double get_current_x_pc() { return templates[current_vt].x_pc; }
	double get_current_y_pc() { return templates[current_vt].y_pc; }
	double get_current_z_pc() { return templates[current_vt].th_pc; }

	void set_view_rgb(const unsigned char * view_rgb) { this->view_rgb = view_rgb; }
//	const unsigned char * get_view_rgb() {	return view_rgb; }
	void convert_view_to_view_template();

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & VT_SHIFT_MATCH;
	ar & VT_STEP_MATCH;
	ar & VT_ACTIVE_DECAY;
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

//	ar & log_vt;
//	ar & log_views;
	}

private:
    friend class boost::serialization::access;

	Visual_Template_Match() {;}
	void clip_view_x_y(int &x, int &y);

	int VT_SHIFT_MATCH;
	int VT_STEP_MATCH;
	double VT_ACTIVE_DECAY;
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



	// unsupported for the moment until fix for serialisation
	// although prob don't need either
	// log vt stuff can go into the ratslam log and log views can be its own program
//	std::ofstream log_vt;
//	std::ofstream log_views;
};
}

#endif // _VISUAL_TEMPLATE_MATCH_H_
