/*
 * PoseCellScene.hpp
 *
 *  Created on: 08/06/2011
 *      Author: scott
 */

#ifndef VIEW_TEMPLATE_SCENE_HPP_
#define VIEW_TEMPLATE_SCENE_HPP_

#include <irrlicht/irrlicht.h>

#include <gri_util.h>

#include <Visual_Template_Match.h>
#include <ImagesScene.hpp>

namespace ratslam
{

class ViewTemplateScene
{
public:
	ViewTemplateScene(irr::scene::ISceneManager * scene, ptree & settings, Visual_Template_Match *in_vt)
	{
		update_ptr(in_vt);
		view_template_scene = scene->createNewSceneManager(false);

		images_scene = new ImagesScene(scene, settings);
	}

	~ViewTemplateScene()
	{

	}

	void update_viewport(int x, int y, int width, int height)
	{
		viewport_x = x;
		viewport_y = y;
		viewport_width = width;
		viewport_height = height;

	}


	void set_viewport()
	{
		view_template_scene->getVideoDriver()->setViewPort(irr::core::rect<irr::s32>((irr::s32) viewport_x, (irr::s32) viewport_y, (irr::s32) (viewport_x+viewport_width), (irr::s32) (viewport_y+viewport_height)));
	}

	void update_scene()
	{
		images_scene->update_image(vtm->view_rgb, "robot_camera", false, 0, 0, vtm->IMAGE_WIDTH, vtm->IMAGE_HEIGHT, 1.0);
#ifdef SUPER_EXPERIENCES
		images_scene->update_image((const double*) &vtm->templates[vtm->current_vt].column_sum[0], "template", true, 0, 400, vtm->templates[vtm->current_vt].column_sum.size()/vtm->TEMPLATE_Y_SIZE, vtm->TEMPLATE_Y_SIZE, (float) (416.0/(double) vtm->TEMPLATE_X_SIZE), (float) (240.0/(double) vtm->TEMPLATE_Y_SIZE));
#else
		images_scene->update_image((const double*) &vtm->templates[vtm->current_vt].column_sum[0], "template", true, 0, 400, vtm->TEMPLATE_X_SIZE, vtm->TEMPLATE_Y_SIZE, (float) (416.0/(double) vtm->TEMPLATE_X_SIZE), (float) (240.0/(double) vtm->TEMPLATE_Y_SIZE));
#endif
		images_scene->update_image((const double*) &vtm->current_view[0], "view", true, 0, 240, vtm->TEMPLATE_X_SIZE, vtm->TEMPLATE_Y_SIZE, (float) (416.0/(double) vtm->TEMPLATE_X_SIZE), (float) (240.0/(double) vtm->TEMPLATE_Y_SIZE));
	}

	void draw_all()
	{
		images_scene->camera_for_normal();
		images_scene->draw_all();
		view_template_scene->drawAll();
	}

	void update_ptr(Visual_Template_Match *vt_in)
	{ 
		vtm = vt_in;
	}


private:

	Visual_Template_Match *vtm;
	irr::scene::ISceneManager * view_template_scene;
	ImagesScene * images_scene;

	double viewport_x, viewport_y, viewport_width, viewport_height;
};

}; // namespace ratslam

#endif /* VIEW_TEMPLATE_SCENE_HPP_ */
