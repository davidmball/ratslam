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


#include <Visual_Template_Match.h>
#include <gri_util.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
using namespace std;
#include <boost/foreach.hpp>

namespace ratslam
{

Visual_Template_Match::Visual_Template_Match(ptree settings)
{
	gri::get_setting_from_ptree(MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double) 0);
	gri::get_setting_from_ptree(PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
	gri::get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
	gri::get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
	gri::get_setting_from_ptree(VT_ACTIVE_DECAY, settings, "vt_active_decay", 1.0);
	gri::get_setting_from_ptree(VT_MATCH_THRESHOLD, settings, "vt_match_threshold", 0.03);
	gri::get_setting_from_ptree(IMAGE_WIDTH, settings, "image_width", 416);
	gri::get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", IMAGE_WIDTH);
	gri::get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
	gri::get_setting_from_ptree(IMAGE_HEIGHT, settings, "image_height", 240);
	gri::get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_vt_x_range0", 0);
	gri::get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_vt_x_range1", IMAGE_WIDTH);
	gri::get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_vt_y_range0", 0);
	gri::get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_vt_y_range1", IMAGE_HEIGHT);

	TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;

	templates.reserve(10000);
	
	current_view.reserve(TEMPLATE_SIZE);

	current_vt = 0;
	prev_vt = 0;
}

Visual_Template_Match::~Visual_Template_Match()
{

}


void Visual_Template_Match::clip_view_x_y(int &x, int &y)
{
	if (x < 0) x = 0;
	else if (x > TEMPLATE_X_SIZE-1) x = TEMPLATE_X_SIZE-1;
	else x = x;
	
	if (y < 0) y = 0;
	else if (y > TEMPLATE_Y_SIZE-1) y = TEMPLATE_Y_SIZE-1;
	else y = y;
}

void Visual_Template_Match::convert_view_to_view_template()
{
	int column_sum_next = 0;
	current_view.clear();
	current_view.resize(TEMPLATE_X_SIZE*TEMPLATE_Y_SIZE);

	int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
	int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
	int x_block_size = sub_range_x/TEMPLATE_X_SIZE; 
	int y_block_size = sub_range_y/TEMPLATE_Y_SIZE;
	int pos;
	for (int y_block=IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
	{
		for (int x_block=IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
		{
			for (int x = x_block; x < (x_block + x_block_size); x++)
			{
				for (int y = y_block; y < (y_block + y_block_size); y++)
				{
					pos = (x+y*IMAGE_WIDTH)*3;
					current_view[column_sum_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos+1]) + (double)(view_rgb[pos+2]));
				}
			}
			current_view[column_sum_next] /= (255.0*3.0);
			current_view[column_sum_next] /= (x_block_size*y_block_size);
			column_sum_next++;
		}
	}

	// now do patch normalisation
	// make configurable
	// +- patch size on teh pixel, ie 4 will give a 9x9
	if (PATCH_NORMALISATION > 0)
	{
		int patch_size = PATCH_NORMALISATION;
		int patch_total = (patch_size*2 + 1)*(patch_size*2 + 1);
		double patch_sum;
		double patch_mean;
		double patch_std;
		int patch_x_clip;
		int patch_y_clip;

		// first make a copy of the view
		std::vector<double> current_view_copy;
		current_view_copy.resize(current_view.size());
		for (int i = 0; i < current_view.size(); i++)
			current_view_copy[i] = current_view[i];

		// this code could be significantly optimimised ....
		for (int x = 0; x < TEMPLATE_X_SIZE; x++)
		{
			for (int y = 0; y < TEMPLATE_Y_SIZE; y++)
			{
				patch_sum = 0;
				for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
				{
					for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
					{
						patch_x_clip = patch_x;
						patch_y_clip = patch_y;
						clip_view_x_y(patch_x_clip, patch_y_clip);

						patch_sum += current_view_copy[patch_x_clip+patch_y_clip*TEMPLATE_X_SIZE];
					}
				}
				patch_mean = patch_sum/patch_total;

				patch_sum = 0;
				for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)
				{
					for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)
					{
						patch_x_clip = patch_x;
						patch_y_clip = patch_y;
						clip_view_x_y(patch_x_clip, patch_y_clip);

						patch_sum += ((current_view_copy[patch_x_clip+patch_y_clip*TEMPLATE_X_SIZE] - patch_mean)*(current_view_copy[patch_x_clip+patch_y_clip*TEMPLATE_X_SIZE] - patch_mean));
					}
				}
			
				patch_std = sqrt(patch_sum/patch_total);

				if (patch_std < MIN_PATCH_NORMALISATION_STD)
					current_view[x+y*TEMPLATE_X_SIZE] = 0.5;
				else
					current_view[x+y*TEMPLATE_X_SIZE] = max((double)0, min(1.0, (current_view_copy[x+y*TEMPLATE_X_SIZE] - patch_mean)/patch_std));
			}
		}
	}
}

// compares raw values not view templates
double Visual_Template_Match::compare_views(double *vt1, double *vt2, int size)
{	
	double cdiff = 0;

	for (int index = 0; index < size; index++)
	{
		cdiff += abs(vt1[index] - vt2[index]);
	}

	cdiff /= (double)(size);

	return cdiff;
}

// create and add a visual template to the collection
int Visual_Template_Match::create_template(double x_pc, double y_pc, double th_pc)
{
	templates.resize(templates.size()+1);
	Visual_Template * new_template = &(*(templates.end()-1));

	new_template->id = templates.size()-1;
	double * column_sum_ptr = &current_view[0];
	new_template->column_sum.reserve(TEMPLATE_SIZE);
	for (int i = 0; i < TEMPLATE_SIZE; i++)
		new_template->column_sum.push_back(*(column_sum_ptr++));
		
	new_template->x_pc = x_pc;
	new_template->y_pc = y_pc;
	new_template->th_pc = th_pc;
	new_template->decay = VT_ACTIVE_DECAY;

	return templates.size() - 1;
}

// compare a visual template to all the stored templates, allowing for 
// slen pixel shifts in each direction
// returns the matching template and the MSE
void Visual_Template_Match::compare(double &vt_err, unsigned int &vt_match_id)
{
	if (templates.size() == 0)
	{
		vt_err = DBL_MAX;
		vt_error = vt_err;
		return;
	}

	double *column_sum = &current_view[0];
	double mindiff, cdiff;
	mindiff = DBL_MAX;

	vt_err = DBL_MAX;
	int min_template = 0;

	double *template_ptr;
	double *column_ptr;
	double *template_row_ptr;
	double *column_row_ptr;
	double *template_start_ptr;
	double *column_start_ptr;
	int row_size;
	int sub_row_size;
	double *column_end_ptr;
	Visual_Template vt;

	int offset;

	BOOST_FOREACH(vt, templates)
	{
		// for each vt try matching the view at different offsets
		// try to fast break based on error alrady great than previous errors
		// handles 2d images shifting only in the x direction
		// note I haven't tested on a 1d yet.
		for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH)
		{
			cdiff = 0;
			template_start_ptr = &vt.column_sum[0] + offset;
			column_start_ptr = &column_sum[0] + VT_SHIFT_MATCH;
			row_size = TEMPLATE_X_SIZE;
			column_end_ptr = &column_sum[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
			sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

			for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
			{
				for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
				{
					cdiff += abs(*column_ptr - *template_ptr);
				}

				// fast breaks
				if (cdiff > mindiff)
					break;
			}

			if (cdiff < mindiff)
			{
				mindiff = cdiff;
				min_template = vt.id;
			}
		}

	}

	vt_err =  mindiff/(double)(TEMPLATE_SIZE - 2*VT_SHIFT_MATCH*TEMPLATE_Y_SIZE);
	vt_match_id = min_template;

	vt_error = vt_err;
}

double Visual_Template_Match::compare_two(int vt_id1, int vt_id2)
{
	double cdiff = 0;
	int index;
	Visual_Template * template1, * template2;

	template1 = &templates[vt_id1];
	template2 = &templates[vt_id2];


	for (index = 0; index < TEMPLATE_SIZE; index++)
	{
		cdiff += abs(template2->column_sum[index] - template1->column_sum[index]);
	}

	// cdiff = sum(cdiff) / (cwl - offset);
	cdiff /= (double)(TEMPLATE_SIZE);

	return cdiff;
}


void Visual_Template_Match::do_log(int frame, double *column_sum, int IMAGE_WIDTH)
{

}

}
