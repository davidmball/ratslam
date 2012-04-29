
#ifndef EXPERIENCE_MAP_SCENE_HPP
#define EXPERIENCE_MAP_SCENE_HPP

#include <irrlicht/irrlicht.h>
#include <Experience_Map.h>
#include <gri_util.h>
#define _USE_MATH_DEFINES
#include <cmath>

namespace ratslam
{

class ExperienceMapScene
{
public:
	ExperienceMapScene(irr::scene::ISceneManager * scene, ptree & settings, Experience_Map *in_map) :
		exp_map_scene(NULL), exp_map_path(NULL), exp_map_goal_path(NULL), exp_map_exps(NULL), map(in_map)
	{
		exp_map_scene = scene->createNewSceneManager(false);

		// the path as lines
		exp_map_path = new PathNode(exp_map_scene->getRootSceneNode());
		exp_map_path->setPrimitiveType(irr::scene::EPT_LINES);
		exp_map_path->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
		exp_map_path->getMaterial(0).Thickness = 2.0f;

		// the path as lines
		exp_map_goal_path = new PathNode(exp_map_scene->getRootSceneNode());
		exp_map_goal_path->setPrimitiveType(irr::scene::EPT_LINES);
		exp_map_goal_path->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 0, 0);
		exp_map_goal_path->getMaterial(0).Thickness = 2.0f;

		gri::get_setting_from_ptree(media_path, settings, "media_path", (std::string) "");

		// add the irat texture
		irr::video::ITexture * irat_texture = exp_map_scene->getVideoDriver()->getTexture((media_path + "/irat_sm.tga").c_str());
		if (irat_texture == 0)
		{
			std::cout << "ERROR: Unable to load texture: " << (media_path + "irat_sm.tga") << std::endl;
			std::cin.get();
			exit(1);
		}

		// add a cube node and texture it with the irat
		irat_node = exp_map_scene->addMeshSceneNode(
			  exp_map_scene->getGeometryCreator()->createCubeMesh(irr::core::vector3df((irr::f32) (irat_texture->getSize().Width/8.0), (irr::f32) (irat_texture->getSize().Height/8.0), (irr::f32) 0.1)), NULL);
		irat_node->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
		irat_node->getMaterial(0).setTexture(0, irat_texture);
		irat_node->getMaterial(0).MaterialType = irr::video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF;



	}

	~ExperienceMapScene()
	{
		exp_map_scene->drop();
		delete exp_map_exps;
		delete exp_map_path;
		delete exp_map_goal_path;
	}

	void update_experiences()
	{
		 double x, y; //, facing;
		  unsigned int i;

			min_x = DBL_MAX;
			max_x = -DBL_MAX;
			min_y = DBL_MAX;
			max_y = -DBL_MAX;

		  for (i = 0; i < map->experiences.size(); i++)
		  {
		    x = map->experiences[i].x_m;
		    y = map->experiences[i].y_m;

		    if (x < min_x)
		    {
		      min_x = x;
		    }

		    if (x > max_x)
		    {
		      max_x = x;
		    }

		    if (y < min_y)
		    {
		      min_y = y;
		    }

		    if (y > max_y)
		    {
		      max_y = y;
		    }

		  }
			
		  // account for the size of the robot
		  min_x = min_x - 0.1;
		  min_y = min_y - 0.1;
		  max_x = max_x + 0.1;
		  max_y = max_y + 0.1;


	}

	void update_links()
	{
		int x2d1, y2d1, x2d2, y2d2;
		for (unsigned int i = 0; i < map->links.size(); i++)
		{
			x2d1 = (int) (((map->experiences[map->links[i].exp_from_id].x_m - min_x) / (max_x - min_x) * viewport_width));
			y2d1 = (int) (((map->experiences[map->links[i].exp_from_id].y_m - min_y) / (max_y - min_y) * viewport_height));
			x2d2 = (int) (((map->experiences[map->links[i].exp_to_id].x_m - min_x) / (max_x - min_x) * viewport_width));
			y2d2 = (int) (((map->experiences[map->links[i].exp_to_id].y_m - min_y) / (max_y - min_y) * viewport_height));
			exp_map_path->addPoint(irr::core::vector3df((irr::f32) x2d1, (irr::f32) y2d1, (irr::f32) 0));
			exp_map_path->addPoint(irr::core::vector3df((irr::f32) x2d2, (irr::f32) y2d2, (irr::f32) 0));
		}
	}

	void update_goal_path(int exp1_id, int exp2_id)
	{
		int x2d1, y2d1, x2d2, y2d2;
		x2d1 = (int) (((map->experiences[exp1_id].x_m - min_x) / (max_x - min_x) * viewport_width));
		y2d1 = (int) (((map->experiences[exp1_id].y_m - min_y) / (max_y - min_y) * viewport_height));
		x2d2 = (int) (((map->experiences[exp2_id].x_m - min_x) / (max_x - min_x) * viewport_width));
		y2d2 = (int) (((map->experiences[exp2_id].y_m - min_y) / (max_y - min_y) * viewport_height));
		exp_map_goal_path->addPoint(irr::core::vector3df((irr::f32) x2d1, (irr::f32) y2d1, (irr::f32) -1.0));
		exp_map_goal_path->addPoint(irr::core::vector3df((irr::f32) x2d2, (irr::f32) y2d2, (irr::f32) -1.0));
	}

	void update_goal_list(const std::deque<int> &goals)
	{
		unsigned int i;
		
		for (i = 0; i < numberNodes.size(); i++)
		{
			numberNodes[i]->remove();
		}
		numberNodes.resize(goals.size());
		
		for (i = 0; i < goals.size(); i++)
		{		
			wchar_t buffer[200];
			swprintf(buffer, 200, L"%i", i);
			numberNodes[i] = exp_map_scene->addBillboardTextSceneNode(exp_map_scene->getGUIEnvironment()->getBuiltInFont(), buffer,
				NULL, irr::core::dimension2d<irr::f32>(50, 50), 
				irr::core::vector3df((irr::f32)((map->experiences[goals[i]].x_m - min_x) / (max_x - min_x) * viewport_width), 
				(irr::f32)((map->experiences[goals[i]].y_m  - min_y) / (max_y - min_y) * viewport_height), (irr::f32)0.0),
				-1, -1);
		}
		if (map->waypoint_exp_id != -1)
		{
			numberNodes.resize(goals.size() + 1);
			wchar_t buffer[200];
			swprintf(buffer, 200, L"x");
			numberNodes[i] = exp_map_scene->addBillboardTextSceneNode(exp_map_scene->getGUIEnvironment()->getBuiltInFont(), buffer,
				NULL, irr::core::dimension2d<irr::f32>(50, 50), 
				irr::core::vector3df((irr::f32)((map->experiences[map->waypoint_exp_id].x_m - min_x) / (max_x - min_x) * viewport_width), 
				(irr::f32)((map->experiences[map->waypoint_exp_id].y_m  - min_y) / (max_y - min_y) * viewport_height), (irr::f32)0.0),
				-1, -1);
		}
	}

	void update_dock_list()
	{
		unsigned int i;
		
		for (i = 0; i < numberDocks.size(); i++)
		{
			numberDocks[i]->remove();
		}
		numberDocks.resize(0);
		
		wchar_t buffer[200];
		swprintf(buffer, 200, L"*", i);

		for (i = 0; i < map->experiences.size(); i++)
		{		
			if (map->experiences[i].dock_visible)
			{
			numberDocks.push_back(exp_map_scene->addBillboardTextSceneNode(exp_map_scene->getGUIEnvironment()->getBuiltInFont(), buffer,
				NULL, irr::core::dimension2d<irr::f32>(50, 50), 
				irr::core::vector3df((irr::f32)((map->experiences[i].x_m - min_x) / (max_x - min_x) * viewport_width), 
				(irr::f32)((map->experiences[i].y_m  - min_y) / (max_y - min_y) * viewport_height), (irr::f32)0.0),
				0xFF00FF00, 0xFF00FF00));
			}
		}
	}

	void update_current_experience(int exp)
	{
		float x2d1, y2d1;
		x2d1 = (irr::f32) ((map->experiences[exp].x_m - min_x) / (max_x - min_x) * viewport_width);
		y2d1 = (irr::f32) ((map->experiences[exp].y_m - min_y) / (max_y - min_y) * viewport_height);

		irat_node->setPosition(irr::core::vector3df(x2d1, y2d1, 0));

		while(map->experiences[exp].th_rad > M_PI)
		{
			map->experiences[exp].th_rad -= 2*M_PI;
		}

		while (map->experiences[exp].th_rad < -M_PI)
		{
			map->experiences[exp].th_rad += 2*M_PI;
		}

		double facing_ratio = tan(map->experiences[exp].th_rad);



		if (map->experiences[exp].th_rad > 0 && map->experiences[exp].th_rad < M_PI_2)
		{
			irat_node->setRotation(irr::core::vector3df(0, 0, (irr::f32) (atan2(facing_ratio*viewport_height, viewport_width)*180/M_PI)));
		}
		else if (map->experiences[exp].th_rad > M_PI_2 && map->experiences[exp].th_rad < M_PI)
		{
			irat_node->setRotation(irr::core::vector3df(0, 0, (irr::f32) (atan2(-facing_ratio*viewport_height, -viewport_width)*180/M_PI)));
		}
		else if (map->experiences[exp].th_rad < 0 && map->experiences[exp].th_rad > -M_PI_2)
		{
			irat_node->setRotation(irr::core::vector3df(0, 0, (irr::f32) (atan2(facing_ratio*viewport_height, viewport_width)*180/M_PI)));
		}
		else if (map->experiences[exp].th_rad < -M_PI_2 && map->experiences[exp].th_rad > -M_PI)
		{
			irat_node->setRotation(irr::core::vector3df(0, 0, (irr::f32) (atan2(-facing_ratio*viewport_height, -viewport_width)*180/M_PI)));
		}
	}

	void update_viewport(int x, int y, int width, int height)
	{
		viewport_x = x;
		viewport_y = y;
		viewport_width = width;
		viewport_height = height;

		// setup an orthogonal matrix
		irr::scene::ICameraSceneNode * exp_map_camera = exp_map_scene->addCameraSceneNode(NULL, irr::core::vector3df((irr::f32) (0.5*viewport_width), (irr::f32) (0.5*viewport_height), (irr::f32) -40),
					irr::core::vector3df((irr::f32) (0.5*viewport_width), (irr::f32) (0.5*viewport_height), (irr::f32) 0));
		irr::core::matrix4 proj;
		proj.buildProjectionMatrixOrthoLH((irr::f32) (1.0*viewport_width), (irr::f32) (1.0*viewport_height), (irr::f32) 0.01, (irr::f32) 100);
		exp_map_camera->setProjectionMatrix(proj, true);
	}

	void set_viewport()
	{
		exp_map_scene->getVideoDriver()->setViewPort(irr::core::rect<irr::s32>((irr::s32) viewport_x, (irr::s32) viewport_y,
							(irr::s32) (viewport_x+viewport_width), (irr::s32) (viewport_y+viewport_height)));
	}

	void update_scene()
	{
		exp_map_path->clearPoints();
		exp_map_goal_path->clearPoints();
		int numexps = map->get_num_experiences();

		update_experiences();
		update_links();
		update_current_experience(map->current_exp_id);

		if (map->goal_list.size() && map->waypoint_exp_id != -1)
		{
			unsigned int trace_exp_id = map->goal_list[0];

			while (trace_exp_id != map->goal_path_final_exp_id)
			{
				update_goal_path(trace_exp_id, map->experiences[trace_exp_id].goal_to_current);
				trace_exp_id = map->experiences[trace_exp_id].goal_to_current;
			}
		}

		update_goal_list(map->goal_list);

		update_dock_list();
	}


	void draw_all()
	{
		exp_map_scene->drawAll();
	}
	
	void screen_to_world(int x, int y, double & x_m, double & y_m)
	{
		x_m = (double)(x - viewport_x) / viewport_width * (max_x - min_x) + min_x;
		y_m = (double)max_y - (y - viewport_y) / viewport_height * (max_y - min_y);
	}

	void update_ptr(Experience_Map *in_map)
	{ 
		map = in_map;
	}

private:
	irr::scene::ISceneManager * exp_map_scene;
	PathNode * exp_map_path;
	PathNode * exp_map_exps;
	PathNode * exp_map_goal_path;
	irr::scene::IMeshSceneNode * irat_node;
	std::string media_path;

	double min_x;
	double max_x;
	double min_y;
	double max_y;

	double viewport_x, viewport_y, viewport_width, viewport_height;

	std::vector<irr::scene::IBillboardTextSceneNode*> numberNodes;
	std::vector<irr::scene::IBillboardTextSceneNode*> numberDocks;

	ratslam::Experience_Map *map;
};

}; // namespace ratslam

#endif

