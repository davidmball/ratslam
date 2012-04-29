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

#include <RatslamGraphics.h>
#include <map>
#include <string>
#include <vector>
#include <float.h>
#include <iostream>

#include <RealTimeGraph.h>
#include <PathGraphNode.h>

#include <queue>

#include <opencv2/highgui/highgui.hpp>

#include <ExperienceMapScene.hpp>
#include <PoseCellsScene.hpp>
#include <ImagesScene.hpp>
#include <ViewTemplateScene.hpp>

#include <Ratslam.hpp>
#include <gri_util.h>

namespace boost
{
	namespace prof
	{
		stats_map default_stats_policy::stats;
	}
}

namespace ratslam
{

RatslamGraphics::RatslamGraphics(ptree & settings, irr::IEventReceiver *receiver, ratslam::Ratslam *ratslam)
{
	device = NULL;
	driver = NULL;
	scene = NULL;
	prev_exp = -1;

	this->settings = settings;
	int width, height, phone_width, phone_height;
	int exp_map_x, exp_map_y, exp_map_size;
	int posecells_x, posecells_y, posecells_size;

	std::string media_path;
	gri::get_setting_from_ptree(width, settings, "width", 1024);
	gri::get_setting_from_ptree(height, settings, "height", 768);
	gri::get_setting_from_ptree(media_path, settings, "media_path", (std::string) "");
	gri::get_setting_from_ptree(phone_width, settings, "phone_width", 320);
	gri::get_setting_from_ptree(phone_height, settings, "phone_height", 480);
	
	gri::get_setting_from_ptree(exp_map_x, settings, "exp_map_x", 460);
	gri::get_setting_from_ptree(exp_map_y, settings, "exp_map_y", 0);
	gri::get_setting_from_ptree(exp_map_size, settings, "exp_map_size", 500);

	gri::get_setting_from_ptree(posecells_x, settings, "posecells_x", 420);
	gri::get_setting_from_ptree(posecells_y, settings, "posecells_y", 0);
	gri::get_setting_from_ptree(posecells_size, settings, "posecells_size", 240);

#ifdef WIN32
	// directx is a lot faster on windows than opengl
	device = irr::createDevice( irr::video::EDT_DIRECT3D9, irr::core::dimension2d<irr::u32>(width, height), 32, false, false, false, receiver);
#else
  device = irr::createDevice( irr::video::EDT_OPENGL, irr::core::dimension2d<irr::u32>(width, height), 32, false, false, false, receiver);
#endif
  if (!device)
  {
  }
  
  driver = device->getVideoDriver();
  scene = device->getSceneManager();
  gui = device->getGUIEnvironment();

  experience_map_scene = new ExperienceMapScene(scene, settings, ratslam->get_experience_map());
  experience_map_scene->update_viewport(exp_map_x, exp_map_y, exp_map_size, exp_map_size);

  pose_cells_scene = new PoseCellsScene(scene, settings, ratslam->get_posecell_network());
  pose_cells_scene->update_viewport(posecells_x, posecells_y, posecells_size, posecells_size);

  view_template_scene = new ViewTemplateScene(scene, settings, ratslam->get_visual_template_collection());
  //view_template_scene->update_viewport(0, 0, 416, 240+240);
  view_template_scene->update_viewport(0, 0, width, height);

  irr::scene::ICameraSceneNode * camera = scene->addCameraSceneNode(NULL, irr::core::vector3df((irr::f32) (0.5*width), (irr::f32) (0.5*height), -10), irr::core::vector3df((irr::f32) (0.5*width), (irr::f32) (0.5*height), 0));
  irr::core::matrix4 proj;
  proj.buildProjectionMatrixOrthoLH((irr::f32) width, (irr::f32) height, (irr::f32) 0.01, (irr::f32) 100);
  camera->setProjectionMatrix(proj, true);

#if 0
  exp_frame_graph = new RealTimeGraphNode(scene, 10000);
  //exp_frame_graph->setRegion(0, 800.0f, 0, 300.0f);
  exp_frame_graph->setRegion(-500.0f, 0.0f, 0, 300.0f);
  //exp_frame_graph->setGraphRange(0, 1000);
  exp_frame_graph->setChannels(2);
  exp_frame_graph->setScale(irr::core::vector3df(1, -1, 1));
  exp_frame_graph->setPosition(irr::core::vector3df((irr::f32) (width - 10.0), (irr::f32) (height - 10.0), 0.0));
  exp_frame_graph->setChannelColor(irr::video::SColor(255, 255, 0, 0), 0);
  exp_frame_graph->setChannelColor(irr::video::SColor(255, 0, 0, 255), 1);
  //exp_frame_graph->setPosition(0.1, 0.1, 0.1);
#endif 

  rt = driver->addRenderTargetTexture(irr::core::dimension2d<irr::u32>(width, height), "RTT1", irr::video::ECF_A8R8G8B8);
  rt_phone = driver->addRenderTargetTexture(irr::core::dimension2d<irr::u32>(phone_width, phone_height), "RTT1", irr::video::ECF_A8R8G8B8);

	 video_writer = new cv::VideoWriter();
	std::string movie_filename;
	gri::get_setting_from_ptree(movie_filename, settings, (std::string) "movie_filename", (std::string) "");
	if (!movie_filename.empty())
	{
	  video_writer = new cv::VideoWriter();
	  video_writer->open(movie_filename.c_str(), CV_FOURCC('X', 'V', 'I', 'D'), 20, cv::Size(width, height));
	  if (!video_writer->isOpened())
	  {
		  printf("failed to open video writer\n");
	  }


	}
	image_buffer = new unsigned char[width*height*3];
}

void RatslamGraphics::reset(ratslam::Ratslam * new_ratslam_ptr)
{
  experience_map_scene->update_ptr(new_ratslam_ptr->get_experience_map());
  pose_cells_scene->update_ptr(new_ratslam_ptr->get_posecell_network());
  view_template_scene->update_ptr(new_ratslam_ptr->get_visual_template_collection());
}

bool RatslamGraphics::begin()
{
	if (!device || !device->run())
	{
	  return false;
	}

	swprintf(window_caption_buffer, 200, L"[ratslam] - %ifps", driver->getFPS());
	device->setWindowCaption(window_caption_buffer);

	driver->beginScene(true, true, irr::video::SColor(255, 0, 0, 0));

	return true;
}

void RatslamGraphics::draw_all(bool draw_pose_cells, bool draw_exp_map, bool draw_images, bool draw_status_bars)
{
	if (draw_pose_cells)
	{
		gri::profiler draw_pc("render pc   ");
		pose_cells_scene->set_viewport();
		pose_cells_scene->update_scene();
		pose_cells_scene->draw_all();
		draw_pc.stop();
	}

	if (draw_exp_map)
	{
		gri::profiler draw_exp("render exp   ");
		experience_map_scene->set_viewport();
		experience_map_scene->update_scene();
		experience_map_scene->draw_all();
		draw_exp.stop();
	}

//	driver->setViewPort(irr::core::rect<irr::s32>(0, 0, driver->getScreenSize().Width, driver->getScreenSize().Height));
//	if (draw_exp_map)
//	{
//		gri::profiler draw_exp("render exp   ");
		view_template_scene->set_viewport();
		view_template_scene->update_scene();
		view_template_scene->draw_all();
//		draw_exp.stop();
//	}


	if (draw_images)
	{
		//driver->setViewPort(irr::core::rect<irr::s32>(0, 0, rt_phone->getSize().Width, rt_phone->getSize().Height));
		//images_scene->camera_for_phone();

	}
	scene->drawAll();


}

void RatslamGraphics::end()
{
	driver->setRenderTarget(rt, true, true, irr::video::SColor(255, 0, 0, 0));
	draw_all(true, true, true, false);
	driver->setRenderTarget(0, true, true, irr::video::SColor(255, 0, 0, 0));

  if (video_writer->isOpened())
   {
	  driver->setRenderTarget(rt, true, true, irr::video::SColor(255, 0, 0, 0));
	  draw_all(false, true, true, true);
	  driver->setRenderTarget(0, true, true, irr::video::SColor(255, 0, 0, 0));
   }

  driver->draw2DImage(rt, irr::core::rect<irr::s32>(0, 0, rt->getSize().Width, rt->getSize().Height),
		  irr::core::rect<irr::s32>(0, 0, rt->getSize().Width, rt->getSize().Height));
//  draw_all(true, true, true, true);

  driver->endScene();

  if (video_writer->isOpened())
  {
	  unsigned char * image_ptr = image_buffer;
	  unsigned char * texture_ptr = (unsigned char*)rt->lock(true);
	  unsigned char * texture_ptr_end = texture_ptr + rt->getSize().getArea() * 4;
	  while (texture_ptr < texture_ptr_end)
	  {
		  *(image_ptr++) = *(texture_ptr++);
		  *(image_ptr++) = *(texture_ptr++);
		  *(image_ptr++) = *(texture_ptr++);
		  texture_ptr++;
	  }
	  rt->unlock();
	  *video_writer << cv::Mat(rt->getSize().Height, rt->getSize().Width, CV_8UC3, (void*) image_buffer);
  }
}

void RatslamGraphics::exp_screen_to_world(int x, int y, double * x_m, double * y_m)
{
	experience_map_scene->screen_to_world(x, y, *x_m, *y_m);
}

RATSLAM_API RatslamGraphics::~RatslamGraphics()
{
  begin();

  driver->setRenderTarget(rt, true, true, irr::video::SColor(255, 0, 0, 0));
  draw_all(true, true, true, true);
  driver->setRenderTarget(0, true, true, irr::video::SColor(255, 0, 0, 0));

  driver->endScene();

  irr::video::IImage * image = driver->createImageFromData(rt->getColorFormat(), rt->getSize(), rt->lock(true), true, false);
  driver->writeImageToFile(image, "screenshot.tga");
  image->drop();
  rt->unlock();

  if (device)
  {
    device->drop();
    device = NULL;
  }

  if (video_writer->isOpened() && video_writer)
  {
	  delete video_writer;
  }
}


void RatslamGraphics::exp_frame(int frame, int exp)
{
	//exp_frame_graph->addPointBack((float) exp, 0);
}

void RatslamGraphics::vt_frame(int frame, int vt)
{
	//exp_frame_graph->addPointBack((float) vt, 1);
}


}
