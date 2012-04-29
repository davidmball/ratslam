/*
 * ImagesScene.hpp
 *
 *  Created on: 08/06/2011
 *      Author: scott
 */

#ifndef IMAGES_SCENE_HPP_
#define IMAGES_SCENE_HPP_

#include <irrlicht/irrlicht.h>

#include <gri_util.h>

class ImagesScene
{
public:
	ImagesScene(irr::scene::ISceneManager * scene, ptree & settings) :
		images_scene(NULL), min(DBL_MAX), max(-DBL_MAX)
	{
		  images_scene = scene->createNewSceneManager(false);
		  irr::scene::ICameraSceneNode * image_camera_node = images_scene->addCameraSceneNode(NULL, irr::core::vector3df((irr::f32) (images_scene->getVideoDriver()->getScreenSize().Width*0.5),
				  (irr::f32) (images_scene->getVideoDriver()->getScreenSize().Height*0.5), (irr::f32) -20),
				  irr::core::vector3df((irr::f32) (images_scene->getVideoDriver()->getScreenSize().Width*0.5), (irr::f32) (images_scene->getVideoDriver()->getScreenSize().Height*0.5), (irr::f32) 0));
		  irr::core::matrix4 proj;
		  proj.buildProjectionMatrixOrthoLH((irr::f32) images_scene->getVideoDriver()->getScreenSize().Width, (irr::f32) images_scene->getVideoDriver()->getScreenSize().Height, (irr::f32) 0.01, (irr::f32) 100);
		  image_camera_node->setProjectionMatrix(proj, true);
		  column_sum_path = new PathNode(images_scene->getRootSceneNode());
		  match_path = new PathNode(images_scene->getRootSceneNode());
		  column_sum_path->setPrimitiveType(irr::scene::EPT_LINE_STRIP);
		  match_path->setPrimitiveType(irr::scene::EPT_LINE_STRIP);
		  column_sum_path->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 0, 0);
		  column_sum_path->getMaterial(0).Thickness = 6.0f;
		  match_path->getMaterial(0).EmissiveColor = irr::video::SColor(255, 0, 255, 0);
		  match_path->getMaterial(0).Thickness = 6.0f;
		  normal_camera = image_camera_node;

		  // for phone
		  phone_camera = images_scene->addCameraSceneNode(NULL, irr::core::vector3df((irr::f32) (160),
				  (irr::f32) (240), (irr::f32) -40),
				  irr::core::vector3df((irr::f32) (160), (irr::f32) (240), (irr::f32) 0));
		  proj.buildProjectionMatrixOrthoLH((irr::f32) 320, (irr::f32) 480, (irr::f32) 0.01, (irr::f32) 100);
		  phone_camera->setProjectionMatrix(proj, true);

		  camera_for_normal();
	}

	~ImagesScene()
	{

	}

	void update_column_sum(double * column_sum, double * match, int image_width, int image_height)
	{
		  int i;
		  for (i = 0; i < image_width; i++)
		  {
		    if (column_sum[i] < min)
		    {
		      min = column_sum[i];
		    }

		    if (match && match[i] < min)
		    {
		    	min = match[i];
		    }

		    if (column_sum[i] > max)
		    {
		      max = column_sum[i];
		    }

		    if (match && match[i] > max)
		    {
		    	max = match[i];
		    }
		  }

		  column_sum_path->clearPoints();
		  match_path->clearPoints();

		  for (i = 0; i < image_width; i++)
		  {
			column_sum_path->addPoint(irr::core::vector3df((irr::f32) i, (irr::f32) (images_scene->getVideoDriver()->getScreenSize().Height - (column_sum[i] - min)/(max - min)*image_height), (irr::f32) -1));
		  }

		  if (match)
		  {
		      for (i = 0; i < image_width; i++)
		    {
		      match_path->addPoint(irr::core::vector3df((irr::f32) i, (irr::f32) (images_scene->getVideoDriver()->getScreenSize().Height - (match[i] - min)/(max - min)*image_height), (irr::f32) -1));
		    }
		  }
	}

	void update_image(const double * image, const char *name, bool greyscale, int x, int y, int width, int height, float scale, float scale_y=-1)
	{
		if (scale_y = -1)
			scale_y = scale;
	  irr::scene::IBillboardSceneNode * image_node;
	  irr::video::ITexture * texture;
	  std::map<std::string, irr::scene::IBillboardSceneNode*>::iterator it = nodes.find(name);

	  y = images_scene->getVideoDriver()->getScreenSize().Height - y;

	  if (it == nodes.end())
	  {
	    texture = images_scene->getVideoDriver()->addTexture(irr::core::dimension2d<irr::u32>(width, height), name, irr::video::ECF_A8R8G8B8);
	    memset(texture->lock(), 0, width*height*4);
	    texture->unlock();

	    image_node = images_scene->addBillboardSceneNode(NULL);
	    image_node->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
	    image_node->getMaterial(0).setTexture(0, texture);
		image_node->getMaterial(0).AntiAliasing = 0;
		image_node->getMaterial(0).setFlag(irr::video::EMF_BILINEAR_FILTER, false);
	    nodes[name] = image_node;

	  }
	  else
	  {
		image_node = it->second;
	    texture = image_node->getMaterial(0).getTexture(0);
	  }
	  image_node->setPosition(irr::core::vector3df((irr::f32) (x + width*scale*0.5), (irr::f32) (y - height * scale_y * 0.5), (irr::f32) 0));
	  image_node->setSize(irr::core::dimension2d<irr::f32>(width*scale, height*scale_y));
	  //image_node->setScale(irr::core::vector3df(scale, scale, scale));

	  unsigned char * texture_ptr = (unsigned char*) texture->lock();


	  if (greyscale)
	  {
		  const double * image_ptr = image;
		  const double * image_end = image_ptr + width*height;
		for (;image_ptr < image_end; )
		{
			  *(texture_ptr++) = (unsigned char) (*(image_ptr)*255.0);
			  *(texture_ptr++) = (unsigned char) (*(image_ptr)*255.0);
			  *(texture_ptr++) = (unsigned char) (*(image_ptr++)*255.0);
			  *(texture_ptr++) = 255;
		}
	  } else {
		  const double * image_ptr = image;
		  const double * image_end = image_ptr + width*height*3;
		for (;image_ptr < image_end; )
		{
			  *(texture_ptr++) = (unsigned char) (*(image_ptr++)*255.0);
			  *(texture_ptr++) = (unsigned char) (*(image_ptr++)*255.0);
			  *(texture_ptr++) = (unsigned char) (*(image_ptr++)*255.0);
			  *(texture_ptr++) = 255;
		}
	  }

	  texture->unlock();
	}

	void update_image(const unsigned char * image, const char *name, bool greyscale, int x, int y, int width, int height, float scale, float scale_y=-1)
	{
		if (scale_y = -1)
			scale_y = scale;
	  irr::scene::IBillboardSceneNode * image_node;
	  irr::video::ITexture * texture;
	  std::map<std::string, irr::scene::IBillboardSceneNode*>::iterator it = nodes.find(name);

	  y = images_scene->getVideoDriver()->getScreenSize().Height - y;

	  if (it == nodes.end())
	  {
	    texture = images_scene->getVideoDriver()->addTexture(irr::core::dimension2d<irr::u32>(width, height), name, irr::video::ECF_A8R8G8B8);
	    memset(texture->lock(), 0, width*height*4);
	    texture->unlock();

	    image_node = images_scene->addBillboardSceneNode(NULL);
	    image_node->getMaterial(0).EmissiveColor = irr::video::SColor(255, 255, 255, 255);
	    image_node->getMaterial(0).setTexture(0, texture);
		image_node->getMaterial(0).AntiAliasing = 0;
		image_node->getMaterial(0).setFlag(irr::video::EMF_BILINEAR_FILTER, false);
	    nodes[name] = image_node;

	  }
	  else
	  {
		image_node = it->second;
	    texture = image_node->getMaterial(0).getTexture(0);
	  }
	  image_node->setPosition(irr::core::vector3df((irr::f32) (x + width*scale*0.5), (irr::f32) (y - height * scale_y * 0.5), (irr::f32) 0));
	  image_node->setSize(irr::core::dimension2d<irr::f32>(width*scale, height*scale_y));
	  //image_node->setScale(irr::core::vector3df(scale, scale, scale));

	  unsigned char * texture_ptr = (unsigned char*) texture->lock();


	  if (greyscale)
	  {
		  const unsigned char * image_ptr = image;
		  const unsigned char * image_end = image_ptr + width*height;
		for (;image_ptr < image_end; )
		{
			  *(texture_ptr++) = *(image_ptr);
			  *(texture_ptr++) = *(image_ptr);
			  *(texture_ptr++) = *(image_ptr++);
			  *(texture_ptr++) = 255;
		}
	  } else {
		  const unsigned char * image_ptr = image;
		  const unsigned char * image_end = image_ptr + width*height*3;
		for (;image_ptr < image_end; )
		{
			  *(texture_ptr++) = *(image_ptr++);
			  *(texture_ptr++) = *(image_ptr++);
			  *(texture_ptr++) = *(image_ptr++);
			  *(texture_ptr++) = 255;
		}
	  }

	  texture->unlock();
	}

	void camera_for_normal()
	{
		images_scene->setActiveCamera(normal_camera);
	}

	void camera_for_phone()
	{
		images_scene->setActiveCamera(phone_camera);
	}

	void draw_all()
	{
		images_scene->drawAll();
	}

private:
	irr::scene::ISceneManager * images_scene;
	std::map<std::string, irr::scene::IBillboardSceneNode*> nodes;
	irr::scene::ICameraSceneNode * normal_camera;
	irr::scene::ICameraSceneNode * phone_camera;
	PathNode * column_sum_path;
	PathNode * match_path;

	double min, max;
};

#endif /* IMAGESSCENE_HPP_ */
