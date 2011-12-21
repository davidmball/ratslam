/*
 * This file is part of the Generic Robot Interface (GRI).
 *
 * Copyright (C) 2011
 * David Ball (d.ball@itee.uq.edu.au), Scott Heath (scott.heath@uqconnect.edu.au)
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

#ifndef _GRI_UTIL_H
#define _GRI_UTIL_H

#include <boost/property_tree/ptree.hpp>
using boost::property_tree::ptree;

#define _USE_MATH_DEFINES
#include "math.h"

#include <profiler.hpp>


namespace gri {



	typedef class boost::prof::basic_profiler
<
boost::prof::empty_logging_policy, // don't log events
boost::prof::default_stats_policy,
boost::high_resolution_timer
>
profiler;


template<typename T>
inline void get_setting_from_ptree(T & var, boost::property_tree::ptree & settings, std::string name, T default_value)
{
	try
	{
		var = settings.get<T>(name);
	}
	catch(boost::property_tree::ptree_bad_path pbp)
	{
		var = default_value;
		std::cout << "SETTINGS(warning): " << name << " not found so default (" << default_value << ") used." << std::endl;
	}
}

inline bool get_setting_child(boost::property_tree::ptree & child, boost::property_tree::ptree & settings, std::string name, bool pause_on_error = true)
{
	try
	{
		child = settings.get_child(name);
	}
	catch(boost::property_tree::ptree_bad_path pbp)
	{
		std::cout << "SETTINGS(error): " << name << " child not found." << std::endl;
		if (pause_on_error)
			std::cin.get();
		return false;
	}
	return true;
}

// % Clip the input angle to between 0 and 2pi radians
inline double clip_rad_360(double angle)
{
    while (angle < 0)
        angle += 2.0 * M_PI;

    while (angle >= 2.0 * M_PI)
        angle -= 2.0 * M_PI;
 
    return angle;
}

// % Clip the input angle to between -pi and pi radians
inline double clip_rad_180(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;

    while (angle <= -M_PI)
        angle += 2.0 * M_PI;
    
    return angle;
}

//% Get the signed delta angle from angle1 to angle2 handling the wrap from 2pi
//% to 0.
inline double get_signed_delta_rad(double angle1, double angle2)
{
    double dir = clip_rad_180(angle2 - angle1);

    double delta_angle = clip_rad_360(angle1) - clip_rad_360(angle2);
	delta_angle = abs(delta_angle);

    if (delta_angle < 2.0 * M_PI - delta_angle)
    {
        if (dir > 0)
            return delta_angle;
        else
            return -delta_angle;
    }
    else
    {
        if (dir > 0)
            return 2.0 * M_PI - delta_angle;
        else
            return -(2.0 * M_PI - delta_angle);
    }
}

}; // namspace gri

#endif // _GRI_UTIL_H
