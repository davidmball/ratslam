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
 * The Experience_Map class describes an undirected graph that is used
 * as the map component of RatSLAM. The Experience_Map class also handles
 * some aspects of goal based navigation.
 */

#ifndef _EXPERIENCE_MAP_H_
#define _EXPERIENCE_MAP_H_

#define _USE_MATH_DEFINES
#include "math.h"

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

struct Link
{
  double d;
  double heading_rad;
  double facing_rad;
  int exp_to_id;
  int exp_from_id;
  double delta_time_s;

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & d;
      ar & heading_rad;
      ar & facing_rad;
      ar & exp_to_id;
      ar & exp_from_id;
      ar & delta_time_s;
    }

};

/*
 * The Experience structure describes
 * a node in the Experience_Map.
 */
struct Experience
{
  int id; // its own id

  double x_m, y_m, th_rad;
  int vt_id;

  std::vector<unsigned int> links_from; // links from this experience
  std::vector<unsigned int> links_to; // links to this experience


  // goal navigation
  double time_from_current_s;
  unsigned int goal_to_current, current_to_goal;

  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & id;
      ar & x_m & y_m & th_rad;
      ar & vt_id;
      ar & links_from & links_to;
      ar & time_from_current_s;
      ar & goal_to_current & current_to_goal;
    }

};

class ExperienceMapScene;

class ExperienceMap
{

public:
  friend class ExperienceMapScene;

  ExperienceMap(ptree settings);
  ~ExperienceMap();

  // create a new experience for a given position
  int on_create_experience(unsigned int exp_id);
  bool on_create_link(int exp_id_from, int exp_id_to, double rel_rad);

  Experience *get_experience(int id)
  {
    return &experiences[id];
  }
  Link * get_link(int id)
  {
    return &links[id];
  }

  // update the current position of the experience map
  // since the last experience
  void on_odo(double vtrans, double vrot, double time_diff_s);

  // update the map by relaxing the graph
  bool iterate();

  // change the current experience
  int on_set_experience(int new_exp_id, double rel_rad);

  int get_num_experiences()
  {
    return experiences.size();
  }

  int get_num_links()
  {
    return links.size();
  }

  int get_current_id()
  {
    return current_exp_id;
  }

  // functions for setting and handling goals.
  void add_goal(double x_m, double y_m);
  void add_goal(int id)
  {
    goal_list.push_back(id);
  }
  bool calculate_path_to_goal(double time_s);
  bool get_goal_waypoint();
  void clear_goal_list()
  {
    goal_list.clear();
  }
  int get_current_goal_id()
  {
    return (goal_list.size() == 0) ? -1 : (int)goal_list.front();
  }
  void delete_current_goal()
  {
    goal_list.pop_front();
  }
  bool get_goal_success()
  {
    return goal_success;
  }
  double get_subgoal_m() const;
  double get_subgoal_rad() const;

  const std::deque<int> &get_goals() const
  {
    return goal_list;
  }

  unsigned int get_goal_path_final_exp()
  {
          return goal_path_final_exp_id;
  }


  template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
      ar & EXP_LOOPS;
      ar & EXP_CORRECTION;
      ar & MAX_GOALS;
      ar & EXP_INITIAL_EM_DEG;

      ar & experiences;
      ar & links;
      ar & goal_list;

      ar & current_exp_id & prev_exp_id;

      ar & accum_delta_facing;
      ar & accum_delta_x;
      ar & accum_delta_y;
      ar & accum_delta_time_s;

      ar & waypoint_exp_id;
      ar & goal_success;
      ar & goal_timeout_s;
      ar & goal_path_final_exp_id;
  	  
      ar & relative_rad;

    }

private:
  friend class boost::serialization::access;

  ExperienceMap()
  {
    ;
  }
  // calculate distance between two experiences using djikstras algorithm
  // can be very slow for many experiences
  double dijkstra_distance_between_experiences(int id1, int id2);


  int EXP_LOOPS;
  double EXP_CORRECTION;
  unsigned int MAX_GOALS;
  double EXP_INITIAL_EM_DEG;

  std::vector<Experience> experiences;
  std::vector<Link> links;
  std::deque<int> goal_list;

  int current_exp_id, prev_exp_id;

  double accum_delta_facing;
  double accum_delta_x;
  double accum_delta_y;
  double accum_delta_time_s;

  double relative_rad;

  int waypoint_exp_id;
  bool goal_success;
  double goal_timeout_s;
  unsigned int goal_path_final_exp_id;

};

}

#endif // _EXPERIENCE_MAP_H_
