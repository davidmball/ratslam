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

#include <Ratslam.hpp>

#include <Visual_Template_Match.h>
#include <Pose_Cell_Network.h>
#include <Experience_Map.h>
#include <gri_util.h>
using namespace gri;

#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdio>
#include <iostream>
#include <iomanip>

#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp> 
#include <boost/date_time/gregorian/gregorian.hpp> 

using namespace std;

extern "C" {int pose_cell_negative(void * vphandle);}

boost::prof::stats_map boost::prof::default_stats_policy::stats;

namespace ratslam
{

 Ratslam::Ratslam(boost::property_tree::ptree & settings)
	: vtrans(0), vrot(M_PI/2.0), prev_vt(0), frame_count(0), debug_level(0), time_s(0)
{
	get_setting_from_ptree(PC_VT_INJECT_ENERGY, settings, "pc_vt_inject_energy", 0.15);
	get_setting_from_ptree(POSECELL_VTRANS_SCALING, settings, "posecell_vtrans_scaling", 5.0);
	get_setting_from_ptree(EXP_DELTA_PC_THRESHOLD, settings, "exp_delta_pc_threshold", 2.0);
	get_setting_from_ptree(debug_level, settings, "debug", 0);

	// create all the state objects
	templates = new Visual_Template_Match(settings);
	network = new Pose_Cell_Network(settings);
	map = new Experience_Map(settings);
	
	// create an initial experience in the map
	map->create_experience(network->x(), network->y(), network->th(), time_diff);

	is_docked = 0;

	gri::get_setting_from_ptree(log_filename, settings, (std::string) "log", (std::string) "");

	open_log_file();

}

void   Ratslam::open_log_file()
{
	if (!log_filename.empty())
	{
		 boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time(); 
		log.open((log_filename + to_iso_string(pt)+".csv").c_str());
		log << "frame, time_s, vt_id, exp_id, num_vt, num_exp, goal_id, goal_success, trans_m, is_docked" << endl;
	}	
}

 Ratslam::~Ratslam()
{

	if (log.is_open())
		log.close();
	
	delete templates;
	delete network;
	delete map;
}

void   Ratslam::set_view_rgb(const unsigned char * view_rgb) 
{ 
	templates->set_view_rgb(view_rgb); 
}

void   Ratslam::process()
{
	if (debug_level > 0)
		cout << "N[" << setw(4) << frame_count << "] ";

	// read odometry
	// diff = current_time - last_time;
	vtrans = vtrans * time_diff;
	vrot = vrot * time_diff;


	/*
	** visual template matching
	*/
	gri::profiler profile_RS_VT("ratslam VT");
	templates->convert_view_to_view_template();
	prev_vt = templates->get_current_vt();
	unsigned int vt_match_id;
	gri::profiler profile_RS_VT_compare("RS VT compare");
	templates->compare(vt_error, vt_match_id);
	profile_RS_VT_compare.stop();
	if (vt_error <= templates->get_vt_match_threshold())
	{
		templates->set_current_vt((int) vt_match_id);
		
		if (debug_level > 0)
			cout << "VTM[" << setw(4) << templates->get_current_vt() << "] ";

		double energy = PC_VT_INJECT_ENERGY * 1.0/30.0 * (30.0 - exp(1.2 * templates->get_decay(templates->get_current_vt())));
		if (energy > 0)
		{
			network->inject((int) templates->get_current_x_pc(), (int) templates->get_current_y_pc(), (int) templates->get_current_z_pc(), energy);
		} else {
			energy = 0;
		}
		if (debug_level > 0)
			cout << "EN=" << setw(5) << setprecision(2) << energy << "] ";
	}
	else
	{
		templates->set_current_vt(templates->create_template(network->x(), network->y(), network->th()));
		if (debug_level > 0)
			cout << "VTN[" << setw(4) << templates->get_current_vt() << "] ";

		if (debug_level > 0)
			cout << "EN=" << setw(5) << setprecision(2) << 0 << "] ";
	}

//	templates->do_log(get_current_frame_count(), &column_sum[0], IMAGE_WIDTH);
	profile_RS_VT.stop();

	gri::profiler profile_RS_PC("ratslam PC");
	/* 
	** pose cell iteration
	*/
	network->excite();
	network->inhibit();
	network->global_inhibit();
	network->normalise();
	// todo: the pc network should really know how to convert vtrans ... do this by telling it the distance between posecells 
	network->path_integration(vtrans*POSECELL_VTRANS_SCALING, vrot);
	network->find_best();
	if (debug_level > 0)
		cout << "PC[" << setw(4) << network->x() << "," << setw(4) << network->y() << "," << setw(4) << network->th() << "] ";
	profile_RS_PC.stop();

	gri::profiler profile_RS_EXP("ratslam EXP");
	/*
	** experience map iteration.
	*/
	Experience * experience;
	double delta_pc;
	int new_exp;

	map->integrate_position(vtrans, vrot);

	experience = map->get_experience(map->get_current_id());
	delta_pc = network->get_delta_pc(experience->x_pc, experience->y_pc, experience->th_pc);
	if (debug_level > 0)
		cout << "D_PC=" << setw(4) << delta_pc << " ";

	if (templates->get_current_exp_size() == 0)
	{
		new_exp = map->create_experience(network->x(), network->y(), network->th(), time_diff);
		map->set_current_id(new_exp);
		templates->add_exp_to_current(new_exp);
		if (debug_level > 0)
			cout << "EXPV[" << setw(3) << map->get_current_id() << "] ";
	}
	else if (delta_pc > EXP_DELTA_PC_THRESHOLD || templates->get_current_vt() != prev_vt) 
	{
		// go through all the exps associated with the current view and find the one with the closest delta_pc

		int matched_exp_id = -1;
		unsigned int i;
		int min_delta_id;
		double min_delta = DBL_MAX;
		double delta_pc;

		// find the closest experience in pose cell space
		for (i = 0; i < templates->get_current_exp_size(); i++)
		{
			experience = map->get_experience(templates->get_current_exp_link(i));
			delta_pc = network->get_delta_pc(experience->x_pc, experience->y_pc, experience->th_pc);

			if (delta_pc < min_delta)
			{
				min_delta = delta_pc;
				min_delta_id = templates->get_current_exp_link(i);
			}
		}

		// if an experience is closer than the thres create a link
		if (min_delta < EXP_DELTA_PC_THRESHOLD)
		{
			matched_exp_id = min_delta_id;
			map->create_link(map->get_current_id(), matched_exp_id, time_diff);
		}

		if (map->get_current_id() != matched_exp_id)
		{
			if (matched_exp_id == -1)
			{
				  new_exp = map->create_experience(network->x(), network->y(), network->th(), time_diff);
					map->set_current_id(new_exp);
				if (debug_level > 0)
					cout << "EXPD[" << setw(3) << map->get_current_id() << "] ";
				templates->add_exp_to_current(new_exp);
			}
			else
			{
				map->set_current_id(matched_exp_id);
				if (debug_level > 0)
					cout << "EXPC[" << setw(3) << map->get_current_id() << "] ";
			}
		} else if (templates->get_current_vt() == prev_vt) {
			new_exp = map->create_experience(network->x(), network->y(), network->th(), time_diff);
			if (debug_level > 0)
				cout << "EXPD[" << setw(3) << map->get_current_id() << "] ";
			map->set_current_id(new_exp);
			templates->add_exp_to_current(new_exp);
		}
	}
	else
	{
		if (debug_level > 0)
			cout << "EXPO[" << setw(3) << map->get_current_id() << "] ";
	}
	map->iterate();

	map->calculate_path_to_goal(time_s);
	map->get_goal_waypoint();

	profile_RS_EXP.stop();

	frame_count++;

	if (debug_level > 0)
		cout << endl;

}


void   Ratslam::do_log()
{
	if (log.is_open())
	{
		
//		log << "frame, time_s, vt_id, exp_id, num_vt, num_exp, goal_id, goal_success, trans_m" << endl;
		log << frame_count << "," << time_s << "," << templates->get_current_vt() << "," << map->get_current_id() << ","
			<< templates->get_number_of() << "," << map->get_num_experiences() << "," << map->get_current_goal_id() << "," << map->get_goal_success() << ","
			<< vtrans * time_diff << ","  << is_docked << endl;
	}

}

void  Ratslam::set_kidnapped() 
{ 
	get_experience_map()->set_kidnapped(); 
}

}
