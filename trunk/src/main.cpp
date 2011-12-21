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
#include <iostream>
using namespace std;

#include "cpp/Ratslam.hpp"
#include "cpp/Visual_Template_Match.h"
#include "cpp/Pose_Cell_Network.h"
#include "cpp/Experience_Map.h"
#include "draw/draw.h"

#include "gri/gri.h"
#include "gri/irr_util.h"
#include "ir_behaviours/ir_behaviours.h"
#include "ir_behaviours/keyboard_control.h"

//#define DOCKING
#ifdef DOCKING
#include "dock/src/visual_dock.h"
#endif

//#define GROUND_TRUTH_TRACKING
#ifdef GROUND_TRUTH_TRACKING
#include "tracking/tracking.h"
#endif

#include <boost/property_tree/ini_parser.hpp>
using boost::property_tree::ptree;
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> 
#include <boost/date_time/gregorian/gregorian.hpp> 

#define _USE_MATH_DEFINES
#include "math.h"

#include <geometry_msgs/Vector3Stamped.h>

void draw_ratslam(ratslam::Ratslam *ratslam);
void print_start_message();

IrrEventReceiver Event_Receiver;

#ifdef WEB_INTERFACE
void add_goal(ratslam::Ratslam ** ratslam, geometry_msgs::Vector3StampedConstPtr goal)
{

	double x_m, y_m;
	if (goal->vector.z == 1)
	{
		draw_exp_screen_to_world_phone((int)goal->vector.x, (int) goal->vector.y, &x_m, &y_m);
	}
	else
	{
		draw_exp_screen_to_world((int)goal->vector.x, (int) goal->vector.y, &x_m, &y_m);
	}
	(*ratslam)->get_experience_map()->add_goal(x_m, y_m);
	printf("added a goal at %f, %f\n", x_m, y_m);
}
#endif

int main(int argc, char * argv[])
{
	print_start_message();

	if (argc < 2)
	{
		cout << "USAGE: " << argv[0] << " <config_file>" << endl;
		std::cin.get();
		exit(-1);
	}

	boost::property_tree::ptree settings;
	read_ini(argv[1], settings);
	gri::Node *gri_node;
	boost::property_tree::ptree gri_settings;
	gri::get_setting_child(gri_settings, settings, "gri");
	int live = 0;
	int reset_ratslam = 0;
	gri::get_setting_from_ptree(live, gri_settings, "live", 0);
	gri::get_setting_from_ptree(reset_ratslam, gri_settings, "reset_ratslam", 0);
	if (live)
		gri_node = new gri::ROS_Node(gri_settings);
	else
		gri_node = new gri::Log_Node(gri_settings);

	gri::Robot *irat_robot = gri_node->get_robot(gri_settings.get<std::string>("robot_name"));
	if (!irat_robot->get_valid())
	{	
		cout << "ERROR: " << gri_settings.get<std::string>("robot_name") << " not valid" << endl;
		std::cin.get();
		return 1;
	}

#ifdef GROUND_TRUTH_TRACKING
	gri::Camera *overhead_camera = gri_node->get_camera(gri_settings.get<std::string>("tracking_name"));
	if (!overhead_camera->get_valid())
	{	
		cout << "ERROR: " << gri_settings.get<std::string>("tracking_name") << " not valid" << endl;
		std::cin.get();
		return 1;
	}

//	Tracking *robot_tracker = new Tracking(settings);  
//	robot_tracker->set_debug(settings.get<int>("tracking.debug"));
//	std::vector<Robot_Global> tracked_robots;
#endif
	
	//gri::Camera *overhead_camera = gri_node->get_camera(gri_settings.get<std::string>("tracking_name"));
	const unsigned char * overhead_image = NULL;

	ratslam::Ratslam * ratslam;
	boost::property_tree::ptree ratslam_settings;
	
	std::string load_ratslam_filename;
	gri::get_setting_from_ptree(load_ratslam_filename, gri_settings, (std::string) "load_ratslam_filename", (std::string) "");
	if (!load_ratslam_filename.empty())
	{
		std::ifstream ifs(load_ratslam_filename.c_str());
		boost::archive::text_iarchive ia(ifs);
		ia >> ratslam;	
		ratslam->set_kidnapped();
	} else {
		gri::get_setting_child(ratslam_settings, settings, "ratslam");
		ratslam = new ratslam::Ratslam(ratslam_settings);
	}

	double current_time = 0; 
	bool pause_mode = false;

	ptree wall_following_settings;
	gri::get_setting_child(wall_following_settings, settings, "wall_following");
	IR_Wall_Follow wall_following(wall_following_settings, *irat_robot);

	ptree draw_settings;
	gri::get_setting_child(draw_settings, settings, "draw");
	int draw_enabled = 1;
	gri::get_setting_from_ptree(draw_enabled, draw_settings, "enable", 1);
	if (draw_enabled)
		draw_init(draw_settings, ratslam->get_posecell_network()->get_pc_dim_xy(), ratslam->get_posecell_network()->get_pc_dim_th(), &Event_Receiver, ratslam, irat_robot);

	// todo add a is_fresh to the node that checks all the robots and cameras
#ifdef GROUND_TRUTH_TRACKING
	cout << "Connecting to the iRat and overhead...";
	while (!irat_robot->is_fresh() || !overhead_camera->is_fresh())
#else
	cout << "Connecting to the " << gri_settings.get<std::string>("robot_name");
	while (!irat_robot->is_fresh())
#endif
	{
		if (!gri_node->update())
		{
			exit(-1);
		}
		gri_node->sleep();
		cout << ".";
	}
	cout << "Connected" << endl;
	wall_following.set_goal(1,0);

#ifdef DOCKING
	ptree docking_settings;
	gri::get_setting_child(docking_settings, settings, "dock");
	Visual_Dock docking(docking_settings, *irat_robot);	
#endif
	bool is_manual_control = false;
	ptree manual_control_settings;
	bool is_settings = gri::get_setting_child(manual_control_settings, settings, "manual_control", false);

	Keyboard_Control manual_control((is_settings ? manual_control_settings : settings), *irat_robot, &Event_Receiver);

#ifdef WEB_INTERFACE
	ros::NodeHandle node_handle;
	ros::Subscriber sub_goal_xy = node_handle.subscribe<geometry_msgs::Vector3Stamped>("/ratslam/goals", 1, boost::bind(add_goal, &ratslam, _1), ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
#endif


	while(gri_node->update() && ((draw_enabled && draw_update()) || (!draw_enabled)) )
	{
		gri::profiler profile_MainLoop("Main Loop");
		if (ratslam->get_current_frame_count() % 100 == 0)
			cout << "RatSLAM progress: " << ratslam->get_current_frame_count() << endl;

		if (draw_enabled && irat_robot->is_fresh())
			draw_begin();		

		if (irat_robot->is_fresh())
		{
			printf("f");
			fflush(stdout);


			ratslam->set_view_rgb(irat_robot->get_camera()->get_rgb_data());

			ratslam->set_is_docked((irat_robot->get_charging()?1:0));
#ifdef DOCKING
			if (ratslam->get_experience_map()->get_num_experiences() > 500)
				docking.force_goto_dock_state(Visual_Dock::FORCE_AUTO);
			else
				docking.force_goto_dock_state(Visual_Dock::FORCE_FALSE);
#endif

#ifdef DOCKING
//			if (!docking.is_docking())
//			{
				if (gri_settings.get<int>("live") == 1)
				{
					if (!is_manual_control)
					{
						if (Event_Receiver.IsKeyDown(irr::KEY_KEY_D))
							docking.force_goto_dock_state(Visual_Dock::FORCE_TRUE);
						if (Event_Receiver.IsKeyDown(irr::KEY_KEY_C))
							docking.force_goto_dock_state(Visual_Dock::FORCE_FALSE);
						if (Event_Receiver.IsKeyDown(irr::KEY_KEY_A))
							docking.force_goto_dock_state(Visual_Dock::FORCE_AUTO);
						if (Event_Receiver.IsKeyDown(irr::KEY_KEY_O))
							docking.manual_get_off_dock();
						if (docking.goto_dock())
						{
							cout << "Going to dock" << endl;
							ratslam->get_experience_map()->goto_dock();
						}
					}
				}
#endif
				
				ratslam->set_odom(irat_robot->get_trans_vel(), irat_robot->get_rot_vel());
				cout << "trans_vel: " << irat_robot->get_trans_vel() << " ";
				current_time += irat_robot->get_delta_time();
				ratslam->set_delta_time(irat_robot->get_delta_time());
				ratslam->process();


#ifdef DOCKING
//			}
#endif
			// perhaps integrate this into ratslam
#ifdef DOCKING
			if (!is_manual_control)
				docking.run_loop();

			if (docking.is_dock_visible())
			{
				ratslam->get_experience_map()->set_dock_is_visible();
			}
#endif

			if (gri_settings.get<int>("live") == 1)
			{
				if(Event_Receiver.IsKeyDown(irr::KEY_KEY_M))
					is_manual_control = !is_manual_control;


				if (is_manual_control) {

					manual_control.run_loop();
				}
#ifdef DOCKING
				 else if (irat_robot->get_charging())
				 {
					 cout << "Robot is charging" << endl;
				 }
				 else if (docking.is_docking())
				{
					cout << "Robot is docking" << endl;
				}

				 else {
#endif


					if (is_manual_control)
					{
						manual_control.run_loop();
					}
					else
					{
						double waypoint_m = ratslam->get_experience_map()->get_subgoal_m();
						double waypoint_rad = ratslam->get_experience_map()->get_subgoal_rad();
						if (waypoint_m == 0 && waypoint_rad == 0)
						{
							if ((double) rand()/RAND_MAX < 0.003)
							{
								double random_value = (double) rand()/RAND_MAX;
								double wall;
								if (random_value > 0.66) 
									wall = -1.0;
								else if (random_value > 0.33)
									wall = 1.0;
								else
									wall = 1.0;
								cout << "Setting wall to follow as " << wall << endl;
								wall_following.set_goal(1, wall);							
							}
						} else {
							wall_following.set_goal(waypoint_m, waypoint_rad);
						}
						//wall_following.set_goal(1, 1);
				//		cout << "wall following" << endl;
						wall_following.run_loop();
					}
#ifdef DOCKING
				}
#endif
			}


			if (draw_enabled)
			{
				draw_ratslam(ratslam);
				/*if (overhead_camera->get_valid() && overhead_camera->is_fresh())
				{
					overhead_image = overhead_camera->get_rgb_data();
				}

				if (overhead_image)
				{
					//draw_image(overhead_image, "overhead_camera", false, 240, 240, overhead_camera->get_width(), overhead_camera->get_height(), 0.5);
				}*/
			}
		}
		else
		{
			printf("n");
			fflush(stdout);
		}

#ifdef GROUND_TRUTH_TRACKING	
//		tracked_robots.clear();
		if (overhead_camera->is_fresh())
		{
//			robot_tracker->process(ratslam->get_current_frame_count(), const_cast<unsigned char *>(overhead_camera->get_rgb_data()), overhead_camera->get_width(), overhead_camera->get_height(), tracked_robots);
		//	draw_image(const_cast<unsigned char *>(overhead_camera->get_rgb_data()), "overhead_camera", false, 0, 600-overhead_camera->get_height()/2, overhead_camera->get_width(), overhead_camera->get_height(), 0.5);
//			draw_image(ratslam->get_current_frame(), "robot_camera", false, 0, 0, ratslam->get_image_width(), ratslam->get_image_height(), 320.0f/416.0f);

		}
#endif
		if (draw_enabled && irat_robot->is_fresh())
		{
			gri::profiler profile_Draw("render draw_end");
			draw_end();	
			profile_Draw.stop();
		}


		if (ratslam->get_current_frame_count() % 1000 == 0)
		{
			gri::profiler::generate_report();
			gri::profiler::restart_all();
		}

		gri_node->do_log();

		profile_MainLoop.stop();
		gri_node->sleep();

		if (Event_Receiver.IsKeyDown(irr::KEY_KEY_P))
			pause_mode = !pause_mode;

		if (pause_mode)
			std::cin.get(); 

		if (ratslam->get_current_frame_count() !=0 && ratslam->get_current_frame_count() == reset_ratslam)
		{
			delete ratslam;
			if (!load_ratslam_filename.empty())
			{
				std::ifstream ifs(load_ratslam_filename.c_str());
				boost::archive::text_iarchive ia(ifs);
				ia >> ratslam;	
			} else {
				ratslam = new ratslam::Ratslam(ratslam_settings);
			}
			draw_update_ratslam_ptr(ratslam);
		}
	}

	if (draw_enabled)
		draw_shutdown();

	std::string save_ratslam_filename;
	gri::get_setting_from_ptree(save_ratslam_filename, gri_settings, (std::string) "save_ratslam_filename", (std::string) "");
	if (!save_ratslam_filename.empty())
	{
		boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time(); 
		std::ofstream ofs((save_ratslam_filename + to_iso_string(pt)+".sav").c_str());
		boost::archive::text_oarchive oa(ofs);
		oa << ratslam;
	}

	if (!gri_node->update())
	{
		delete ratslam;
		delete gri_node;
		printf("Program completed successfully\n");
		if (pause_mode)
			std::cin.get();
	} else {
		delete ratslam;
		delete gri_node;
		printf("Program exited by user\n");
	}

	return 1;
}


void draw_ratslam(ratslam::Ratslam *ratslam)
{
	gri::profiler Draw_VT("draw_all_vt");

	Draw_VT.stop();

	draw_exp_frame(ratslam->get_current_frame_count(), ratslam->get_experience_map()->get_current_id());
	draw_vt_frame(ratslam->get_current_frame_count(), ratslam->get_visual_template_collection()->get_current_vt());

	double x_m, y_m;
	irr::core::position2d<irr::s32> MouseCoords;
	if (Event_Receiver.IsMouseDoubleClick(irr::EMIE_LMOUSE_DOUBLE_CLICK, MouseCoords))
	{
		draw_exp_screen_to_world(MouseCoords.X, MouseCoords.Y, &x_m, &y_m);
//		cout << MouseCoords.X << " " << MouseCoords.Y << " " << x_m << " " << y_m << endl;
		ratslam->get_experience_map()->add_goal(x_m, y_m);
	}


}


void print_start_message()
{
	cout << "RatSLAM lite - Bio-inspired SLAM navigation system." << endl;
	cout << "Program Copyright 2011 David Ball, Scott Heath, The University of Queensland." << endl;
	cout << "RatSLAM algorithm by Michael Milford and Gordon Wyeth." << endl;
	cout << "This is free software distributed under the GNU GPL." << endl;
	cout << endl;
}
