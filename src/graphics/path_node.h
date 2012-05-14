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

#ifndef PATHGRAPHNODE_H_
#define PATHGRAPHNODE_H_

#include <irrlicht/irrlicht.h>

class PathNode : public irr::scene::ISceneNode
{
public:

	PathNode(irr::scene::ISceneNode * parent)
	: irr::scene::ISceneNode(parent, parent->getSceneManager())
	{

	}

	~PathNode()
	{

	}


	virtual void OnRegisterSceneNode()
	{
		// if the node is visible then register it for
		// rendering
		if (IsVisible)
		{
			SceneManager->registerNodeForRendering(this);
		}

		// register children
		ISceneNode::OnRegisterSceneNode();
	}

	virtual void render()
	{
		// get the device
		irr::video::IVideoDriver * driver = SceneManager->getVideoDriver();
		driver->setMaterial(mat);
		driver->setTransform(irr::video::ETS_WORLD, AbsoluteTransformation);

		//std::vector<irr::video::S3DVertex>::iterator it1 = vertices.begin();
		//std::vector<irr::video::S3DVertex>::iterator it2;

		if (vertices.size() == 0)
		{
			return;
		}

		irr::u32 primitive_count;

		switch(primitive_type)
		{
		case irr::scene::EPT_LINES:
			primitive_count = indices.size() / 2;
			break;
		case irr::scene::EPT_POINTS:
			primitive_count = indices.size();
			break;
		case irr::scene::EPT_LINE_STRIP:
			primitive_count = indices.size() - 1;
			break;
		default:
			primitive_count = indices.size() / 3;
			break;
		}

		driver->drawVertexPrimitiveList((void*)&vertices[0], vertices.size(),
				&indices[0], primitive_count, irr::video::EVT_STANDARD, this->primitive_type,
				irr::video::EIT_32BIT);

		/*switch (primitive_type)
		{
			case irr::scene::EPT_LINES:
				for (it2 = it1 + 1; it2 != points.end(); it1++, it2++)
				{
					if ((*it1).X == FLT_MAX || (*it2).X == FLT_MAX)
					{
						continue;
					}
					driver->draw3DLine(*it1, *it2, irr::video::SColor(255, 255, 0, 0));
				}
				break;

			default:

		}*/



	}

	virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const
	{
		// not actually using this, but return the box
		// anyway
		return box;
	}

	virtual irr::u32 getMaterialCount()
	{
		// return the material count
		return 1;
	}

	virtual irr::video::SMaterial& getMaterial(irr::u32 i)
	{
		// return the material
		return mat;
	}

	void addPoint(const irr::core::vector3df & point)
	{
		// points.push_back(point);
		irr::video::S3DVertex vertex;
		vertex.Pos = point;
		indices.push_back(vertices.size());
		vertices.push_back(vertex);
	}

	void clearPoints()
	{
		indices.clear();
		vertices.clear();
	}

	/*void addGap()
	{
		irr::core::vector3df gap;
		gap.X = FLT_MAX;
		points.push_back(gap);
	}*/

	void setPrimitiveType(irr::scene::E_PRIMITIVE_TYPE primitive_type)
	{
		this->primitive_type = primitive_type;
	}

private:
	// scene node variables
	irr::core::aabbox3d<irr::f32> box;
	irr::video::SMaterial mat;
	// std::vector<irr::core::vector3df> points;
	std::vector<irr::video::S3DVertex> vertices;
	std::vector<unsigned int> indices;
	irr::scene::E_PRIMITIVE_TYPE primitive_type;
};


#endif /* PATHGRAPHNODE_H_ */
