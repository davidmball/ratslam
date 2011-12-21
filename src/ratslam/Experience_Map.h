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
 * The Experience_Map class describes an undirected graph that is used
 * as the map component of RatSLAM. The Experience_Map class also handles
 * some aspects of goal based navigation.
 */

#ifndef _EXPERIENCE_MAP_H_
#define _EXPERIENCE_MAP_H_

#define _USE_MATH_DEFINES
#include "math.h"

#pragma warning( disable: 4275 ) // problem between std::vector and log4cxx
#pragma warning( disable: 4251 ) // problem between std::vector and log4cxx

// todo: replace this with iostream
#include <stdio.h>
#include <vector>
#include <deque>

#include <iostream>

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;




#include <boost/serialization/access.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/deque.hpp>

namespace ratslam
{

/*
 * The Link structure describes a link
 * between two experiences.
 */

typedef struct td_link
{
    double d;
    double heading_rad;
    double facing_rad;
    int exp_to_id;
	int exp_from_id;
	double delta_time_s;

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & d;
	ar & heading_rad;
	ar & facing_rad;
	ar & exp_to_id;
	ar & exp_from_id;
	ar & delta_time_s;
	}

} Link;

/*
 * The Experience structure describes
 * a node in the Experience_Map.
 */
typedef struct td_experience
{
	int id; // its own id

	double x_m, y_m, th_rad;
    double x_pc, y_pc, th_pc;
    int vt_id;
    
	std::vector<unsigned int> links_from;		// links from this experience
	std::vector<unsigned int> links_to;		// links to this experience
	
	bool dock_visible;

	// goal navigation
	double time_from_current_s;
	unsigned int goal_to_current, current_to_goal;

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & id;
	ar & x_m & y_m & th_rad;
	ar & x_pc & y_pc & th_pc;
	ar & vt_id;
	ar & links_from & links_to;
	ar & dock_visible;
	ar & time_from_current_s;
	ar & goal_to_current & current_to_goal;
	}

} Experience;

class ExperienceMapScene;

class Experience_Map
{

public:
	friend class ExperienceMapScene;

	Experience_Map(ptree settings);
	~Experience_Map();

	// create a new experience for a given position 
	int create_experience(double x, double y, double th, double delta_time_s);
	bool create_link(int exp_id_from, int exp_id_to, double delta_time_s);

	Experience *get_experience(int id) { return &experiences[id]; }
	Link * get_link(int id) { return &links[id]; }

	// update the current position of the experience map
	// since the last experience
	void integrate_position(double vtrans, double vrot);

	// update the map by relaxing the graph
	bool iterate();
	
	// change the current experience
	int set_current_id(int new_exp_id);

	int get_num_experiences() { return experiences.size(); }

	int get_current_id() { return current_exp_id; }

    double get_exp_correction() const { return EXP_CORRECTION; }
    int get_exp_loops() const { return EXP_LOOPS; }

	// functions for setting and handling goals.
	void add_goal(double x_m, double y_m);
	void add_goal(int id) { goal_list.push_back(id); std::cout << "Added goal #" << goal_list.size() << std::endl; }
	bool calculate_path_to_goal(double time_s);
	bool get_goal_waypoint();
	void clear_goal_list() { goal_list.clear(); }
	int get_current_goal_id() { return (goal_list.size() == 0) ? -1 : (int) goal_list.front(); }
	void delete_current_goal() { goal_list.pop_front(); }
	bool get_goal_success() { return goal_success; }
	double get_subgoal_m() const;
	double get_subgoal_rad() const;
	const std::deque<int> &get_goals() const { return goal_list; }

    // docking functions, docking experiences can be used as goals
	void set_dock_is_visible()	{ experiences[current_exp_id].dock_visible = true; }
	void goto_dock();

	// calculate distance between two experiences using djikstras algorithm
	// can be very slow for many experiences
	double dijkstra_distance_between_experiences(int id1, int id2);

	// if this function is called, the experience map will not try and
	// join the next new node to the rest of the map
	void set_kidnapped() { kidnapped = 1; }

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
	ar & EXP_LOOPS;
	ar & EXP_CORRECTION;
	ar & MAX_GOALS;

	ar & experiences;
	ar & links;
	ar & goal_list;
  
    ar & current_exp_id & prev_exp_id;

    ar & accum_delta_facing;
    ar & accum_delta_x;
    ar & accum_delta_y;

	ar & waypoint_exp_id;
	ar & goal_success;
	ar & goal_timeout_s;
	ar & goal_path_final_exp_id;

	ar & kidnapped;
	}


private:
    friend class boost::serialization::access;

	Experience_Map() {;}

	int EXP_LOOPS;
	double EXP_CORRECTION;
	unsigned int MAX_GOALS;

	std::vector<Experience> experiences;
	std::vector<Link> links;
	std::deque<int> goal_list;
  
    int current_exp_id, prev_exp_id;

    double accum_delta_facing;
    double accum_delta_x;
    double accum_delta_y;

	int waypoint_exp_id;
	bool goal_success;
	double goal_timeout_s;
	unsigned int goal_path_final_exp_id;

	int kidnapped;
};


}

#endif // _EXPERIENCE_MAP_H_
