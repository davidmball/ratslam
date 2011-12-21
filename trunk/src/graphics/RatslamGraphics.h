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

#ifndef RATSLAM_GRAPHICS_H_
#define RATSLAM_GRAPHICS_H_

#include <gri_util.h>
#include <irr_util.h>
#include <Ratslam.hpp>
#include <deque>

class RealTimeGraphNode;

namespace cv
{
class VideoWriter;
}

namespace ratslam
{
class ExperienceMapScene;
class PoseCellsScene;
class ViewTemplateScene;


class RATSLAM_API RatslamGraphics
{
public:


	RatslamGraphics(ptree & settings, irr::IEventReceiver *receiver, ratslam::Ratslam *ratslam);
	~RatslamGraphics();

	// begin and end drawing
	bool begin();
	void end();

	int exp_goal_path(int exp1_id, int exp2_id);
	void exp_screen_to_world(int x, int y, double * x_m, double * y_m);
	void exp_screen_to_world_phone(int x, int y, double * x_m, double * y_m);

	void exp_frame(int frame, int exp);
	void vt_frame(int frame, int vt);

	// called when ratslam resets
	void reset(ratslam::Ratslam * new_ratslam_ptr);

private:

	void draw_all(bool draw_pose_cells, bool draw_exp_map, bool draw_images, bool draw_status_bars);

	irr::IrrlichtDevice *device;
	irr::video::IVideoDriver * driver;
	irr::scene::ISceneManager * scene;


	irr::gui::IGUIEnvironment * gui;

	wchar_t window_caption_buffer[200];

	RealTimeGraphNode * exp_frame_graph;

	ExperienceMapScene * experience_map_scene;
	PoseCellsScene * pose_cells_scene;
	ViewTemplateScene * view_template_scene;



	int prev_exp;

	irr::video::ITexture * rt;
	irr::video::ITexture * rt_phone;


	cv::VideoWriter * video_writer;

	unsigned char * image_buffer;

	ptree settings;

};
}
#endif

