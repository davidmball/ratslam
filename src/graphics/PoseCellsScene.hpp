/*
 * PoseCellScene.hpp
 *
 *  Created on: 08/06/2011
 *      Author: scott
 */

#ifndef POSE_CELLS_SCENE_HPP_
#define POSE_CELLS_SCENE_HPP_

#include <irrlicht/irrlicht.h>

#include <gri_util.h>

#include <Pose_Cell_Network.h>

namespace ratslam
{

class PoseCellsScene
{
public:
	PoseCellsScene(irr::scene::ISceneManager * scene, ptree & settings, Pose_Cell_Network *in_pc) :
		pose_cells_scene(NULL), particles(NULL), position_line(NULL), pose_cell_history(NULL), posecells(in_pc)
	{
		gri::get_setting_from_ptree(media_path, settings, "media_path", (std::string) "");
		pose_cells_scene = scene->createNewSceneManager(false);

		particles = new irr::scene::IBillboardSceneNode*[NUM_PARTICLES];

		for (int i = 0; i < NUM_PARTICLES; i++)
		{
			particles[i] = pose_cells_scene->addBillboardSceneNode(pose_cells_scene->getRootSceneNode(), irr::core::dimension2d<irr::f32>(2, 2));
			particles[i]->setMaterialFlag(irr::video::EMF_LIGHTING, false);
			particles[i]->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
			particles[i]->setMaterialType(irr::video::EMT_TRANSPARENT_ADD_COLOR);
			particles[i]->setMaterialTexture(0, pose_cells_scene->getVideoDriver()->getTexture((media_path + "/particle.bmp").c_str()));

		}

		position_line = pose_cells_scene->addMeshSceneNode(
			  pose_cells_scene->getGeometryCreator()->createCylinderMesh(0.5, 1.0, 5, irr::video::SColor(255, 255, 0, 0)),
			  pose_cells_scene->getRootSceneNode());
		position_line->getMaterial(0).Lighting = false;
		position_line->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 0, 0);

		  pose_cell_history = new PathNode(pose_cells_scene->getRootSceneNode());
		  pose_cell_history->getMaterial(0).EmissiveColor = irr::video::SColor(255, 0, 255, 0);
		  pose_cell_history->setPrimitiveType(irr::scene::EPT_POINTS);

		  PathNode * pose_cell_boundary = new PathNode(pose_cells_scene->getRootSceneNode());
		  pose_cell_boundary->setPrimitiveType(irr::scene::EPT_LINES);
		  pose_cell_boundary->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
		  pose_cell_boundary->getMaterial(0).Thickness = 2.0f;
		  // bottom
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));

		  // top
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));

		  // sides
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(-posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, -posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));
		  pose_cell_boundary->addPoint(irr::core::vector3df(posecells->PC_DIM_XY/2.0f, posecells->PC_DIM_TH/2.0f, -posecells->PC_DIM_XY/2.0f));

		  irr::scene::ICameraSceneNode * pose_cell_camera = pose_cells_scene->addCameraSceneNode(NULL, irr::core::vector3df(10, 40, -40), irr::core::vector3df(0, 0, 0));
		  irr::core::matrix4 proj;
		  proj.buildProjectionMatrixOrthoLH((irr::f32) posecells->PC_DIM_XY * 2 + 1, (irr::f32) 50, (irr::f32) 0.01, (irr::f32) 100);
		  pose_cell_camera->setProjectionMatrix(proj, true);
	}

	~PoseCellsScene()
	{

	}

	void update_viewport(int x, int y, int width, int height)
	{
		viewport_x = x;
		viewport_y = y;
		viewport_width = width;
		viewport_height = height;

	}

	void update_pose_cells(double * pose_cells, int PC_DIM_XY, int PC_DIM_TH)
	{
		irr::core::matrix4 mat;
		irr::core::vector3df trans;
		int next_particle = 0;

		int next = 0;
		for (int k = 0; k < PC_DIM_TH; k++)
		{
		for (int j = 0; j < PC_DIM_XY; j++)
		{
		  for (int i = 0; i < PC_DIM_XY; i++)
		  {
			if (pose_cells[next] > 0.002)
			{
			  if (next_particle < NUM_PARTICLES)
			  {
				particles[next_particle]->setPosition(irr::core::vector3df((irr::f32) (i-posecells->PC_DIM_XY/2),
						(irr::f32) (k - posecells->PC_DIM_TH/2),
						(irr::f32) (j - posecells->PC_DIM_XY/2) ));
				particles[next_particle]->setVisible(true);
				next_particle++;
			  }
			}
			next++;
		  }
		}
		}

		for (; next_particle < NUM_PARTICLES; next_particle++)
		{
		particles[next_particle]->setVisible(false);
		  }
	}

	void update_position(double x, double y, double th, int PC_DIM_XY, int PC_DIM_TH)
	{
		position_line->setPosition(irr::core::vector3df((irr::f32) (x-(double)PC_DIM_XY/2),
			  (irr::f32) (th - (double)PC_DIM_TH/2),
	  		  (irr::f32) (y - (double)PC_DIM_XY/2) ));

		position_line->setScale(irr::core::vector3df((irr::f32) 0.5, (irr::f32) -th, (irr::f32) 0.5));

	  pose_cell_history->addPoint(irr::core::vector3df((irr::f32) (x-(double)PC_DIM_XY/2),
			  (irr::f32) (-(double)PC_DIM_TH/2),
	  		(irr::f32) (y - (double)PC_DIM_XY/2) ));
	}

	void set_viewport()
	{
		pose_cells_scene->getVideoDriver()->setViewPort(irr::core::rect<irr::s32>((irr::s32) viewport_x, (irr::s32) viewport_y,
							(irr::s32) (viewport_x+viewport_width), (irr::s32) (viewport_y+viewport_height)));
	}

	void update_scene()
	{
		update_pose_cells(posecells->posecells_memory, posecells->PC_DIM_XY, posecells->PC_DIM_TH); 
		update_position(posecells->best_x, posecells->best_y, posecells->best_th, posecells->PC_DIM_XY, posecells->PC_DIM_TH);
	}

	void clear_history()
	{
		pose_cell_history->clearPoints();
	}

	void draw_all()
	{
		pose_cells_scene->drawAll();
	}

	void update_ptr(Pose_Cell_Network *pc_in)
	{ 
		clear_history();
		posecells = pc_in;
	}


private:
	static const int NUM_PARTICLES = 500;
	irr::scene::ISceneManager * pose_cells_scene;
	irr::scene::IBillboardSceneNode ** particles;
	irr::scene::IMeshSceneNode * position_line;
	PathNode * pose_cell_history;
	Pose_Cell_Network *posecells;

	std::string media_path;

	double viewport_x, viewport_y, viewport_width, viewport_height;
};

}; // namespace ratslam

#endif /* POSECELLSCENE_HPP_ */
