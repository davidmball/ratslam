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

#ifndef RATSLAM_HPP_
#define RATSLAM_HPP_

#ifdef WIN32
#pragma warning( disable: 4275 ) // problem between std::vector and log4cxx
#pragma warning( disable: 4251 ) // problem between std::vector and log4cxx
#endif

#if defined RATSLAM_EXPORTS && defined WIN32
#define RATSLAM_API __declspec(dllexport)
#elif defined RATSLAM_IMPORTS && defined WIN32
#define RATSLAM_API __declspec(dllimport)
#else
#define RATSLAM_API
#endif

#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <vector>
#include <fstream>


#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>

namespace ratslam
{

class Visual_Template_Match;
class Pose_Cell_Network;
class Experience_Map;


class RATSLAM_API Ratslam
{
public:

	Ratslam(boost::property_tree::ptree & settings);
	~Ratslam();

	void set_debug_level(int level) { this->debug_level = level; }
	
	// set intputs
	void set_view_rgb(const unsigned char * view_rgb);
	void set_odom(double vtrans, double vrot) {	this->vtrans = vtrans; this->vrot = vrot; }
	void set_delta_time(double delta_time_s) { this->time_diff = delta_time_s; this->time_s += delta_time_s; }
	
	// process a frame
	void process();

    // current visual template info
//	const unsigned char * get_current_frame() {	return frame; }
//	double * get_current_view() { return &column_sum[0]; }
	
	// current experience map info

	int get_current_frame_count() { return frame_count; }
	double get_time() { return this->time_s; }
	
	// state accessors
	Visual_Template_Match * get_visual_template_collection()	{ return templates;	}
	Pose_Cell_Network * get_posecell_network() { return network; }
	Experience_Map * get_experience_map() { return map; }
	
	// constants
    double get_exp_delta_pc_threshold() const { return EXP_DELTA_PC_THRESHOLD; }
    double get_posecell_vtrans_scaling() const { return POSECELL_VTRANS_SCALING; }
    double get_pc_vt_inject_energy() const { return PC_VT_INJECT_ENERGY; }

	void do_log();

	void set_is_docked(int state) {is_docked = state;}

	void set_kidnapped();
	
	template <typename Archive>
	void save(Archive& ar, const unsigned int version) const
	{
	ar & PC_VT_INJECT_ENERGY;
	ar & POSECELL_VTRANS_SCALING;
	ar & EXP_DELTA_PC_THRESHOLD;
	ar & log_filename;

	ar & network;
	ar & map;
	ar & vtrans & vrot;
	ar & prev_vt;

	ar & frame_count;

	ar & time_diff;
	ar & time_s;
	ar & vt_error;

	ar & debug_level;

	ar & is_docked;

	// frame and log not included
	}

	template <typename Archive>
	void load(Archive& ar, const unsigned int version)
	{
	ar & PC_VT_INJECT_ENERGY;
	ar & POSECELL_VTRANS_SCALING;
	ar & EXP_DELTA_PC_THRESHOLD;
	ar & log_filename;

	ar & network;
	ar & map;
	ar & vtrans & vrot;
	ar & prev_vt;

	ar & frame_count;

	ar & time_diff;
	ar & time_s;
	ar & vt_error;

	ar & debug_level;

	ar & is_docked;

	open_log_file();
	}

	void open_log_file();

	BOOST_SERIALIZATION_SPLIT_MEMBER()
private:
    friend class boost::serialization::access;

	double PC_VT_INJECT_ENERGY;
	double POSECELL_VTRANS_SCALING;

	double EXP_DELTA_PC_THRESHOLD;
	std::string log_filename;

	// state objects
	ratslam::Visual_Template_Match * templates;
	ratslam::Pose_Cell_Network * network;
	ratslam::Experience_Map * map;

	// current trans / rot
	double vtrans, vrot;

	// current view

	int prev_vt;

	// the number of frames that have been
	int frame_count;

	// the time difference between frames
	double time_diff;
	double time_s;

	// the visual template error
	double vt_error;

	const unsigned char * frame;

	int debug_level;

	int is_docked;

	std::ofstream log;


	Ratslam() { ;}
  Ratslam(const Ratslam & other);
  const Ratslam & operator=(const Ratslam & other);
};



}

#endif

