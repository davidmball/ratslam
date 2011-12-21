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
#ifndef REAL_TIME_GRAPH_H_
#define REAL_TIME_GRAPH_H_

#include <irrlicht/irrlicht.h>
#include <list>


class RealTimeGraphNode : public irr::scene::ISceneNode
{
private:
	float minXValue;
	float maxXValue;
	float minYValue;
	float maxYValue;

	// placement variables
	float minXPlace;
	float maxXPlace;
	float minYPlace;
	float maxYPlace;

	// scene node variables
	irr::core::aabbox3d<irr::f32> box;
	irr::video::SMaterial mat;


	// the capcaity
	unsigned int capacity;

	// numbering
	int numberAmount;
	irr::scene::ITextSceneNode ** numberNodes;

	// channels
	int numberChannels;

	struct Channel
	{
		// values for the channel
		std::list<float> values;

		// color
		irr::video::SColor color;

		// constructor
		Channel()
		{
			color.set(255, 0, 0, 0);
		}

		// to set the color
		void setColor(irr::video::SColor color)
		{
			this->color = color;
		}
	};

	Channel * channels;

	

	void updateNumbers()
	{
		float stepY = (maxYPlace - minYPlace) /
			(maxYValue - minYValue);
		float valueStepY = (maxYValue - minYValue) / (numberAmount - 1);
		float currentValue = minYValue;
		for (int i = 0; i < numberAmount; i++)
		{
			wchar_t buffer[200];
			if (numberNodes[i])
			{
				numberNodes[i]->remove();
				//delete numberNodes[i];
			}
			swprintf(buffer, 200, L"%.2f", currentValue);
			numberNodes[i] = SceneManager->addTextSceneNode(SceneManager->getGUIEnvironment()->getBuiltInFont(), buffer,
				irr::video::SColor(255, 0, 255, 0), this, irr::core::vector3df(minXPlace + 50, (currentValue - minYValue) * stepY + minYPlace, 0));
			currentValue += valueStepY;
		}
	}

public:
	RealTimeGraphNode(irr::scene::ISceneManager * smg, unsigned int capacity) :
		ISceneNode(smg->getRootSceneNode(), smg)
	{
		AutomaticCullingState = irr::scene::EAC_OFF;
		mat.Lighting = false;
		mat.AmbientColor.set(255, 0, 0, 0);
	
		this->capacity = capacity;
		minXValue = 0;
		maxYValue = 0;
		minYValue = 0;
		maxXValue = 0;
		//maxXValue = (float)(capacity - 1);

		setRegion(0, 640, 0, 480);
		setGraphRange(0, 10);


		numberChannels = 1;
		numberAmount = 11;

		numberNodes = new irr::scene::ITextSceneNode*[numberAmount];

		for (int i = 0; i < numberAmount; i++)
		{
			numberNodes[i] = 0;
		}

		channels = 0;

		channels = new Channel[numberChannels];
	}

	void setChannels(int channels)
	{
		if (channels)
		{
			delete [] this->channels;
		}

		numberChannels = channels;
		this->channels = new Channel[numberChannels];

	}

	void setChannelColor(irr::video::SColor color, int channel)
	{
		channels[channel].setColor(color);
	}

	void setRegion(float left, float right, float top, float bottom)
	{
		minXPlace = left;
		maxXPlace = right;
		minYPlace = bottom;
		maxYPlace = top;
	}

	void setGraphRange(float minY, float maxY)
	{
		//minYValue = minY;
		//maxYValue = maxY;
	}
	
	void addPointFront(float dataY, int channel)
	{
		channels[channel].values.push_front(dataY);
		
		if (channels[channel].values.size() > capacity)
		{
			channels[channel].values.pop_back();
		}

		if (dataY > maxYValue)
		{
			maxYValue = dataY;
		}
		else if (dataY < minYValue)
		{
			minYValue = dataY;
		}

		updateNumbers();

	}

	void addPointBack(float dataY, int channel)
	{
		channels[channel].values.push_back(dataY);

		if (channels[channel].values.size() > capacity)
		{
			channels[channel].values.pop_front();
		}

		if (dataY > maxYValue)
		{
			maxYValue = dataY;
		}
		/*else if (dataY < minYValue)
		{
			minYValue = dataY;
		}*/
		if (channels[channel].values.size() > maxXValue)
		{
			maxXValue = (float) channels[channel].values.size();
		}

		updateNumbers();
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

		// draw the border
		driver->draw3DLine(irr::core::vector3df(minXPlace, maxYPlace, 0),
				irr::core::vector3df(maxXPlace, maxYPlace, 0), irr::video::SColor(255, 255, 255, 255));
		driver->draw3DLine(irr::core::vector3df(maxXPlace, maxYPlace, 0),
				irr::core::vector3df(maxXPlace, minYPlace, 0), irr::video::SColor(255, 255, 255, 255));
		driver->draw3DLine(irr::core::vector3df(maxXPlace, minYPlace, 0),
				irr::core::vector3df(minXPlace, minYPlace, 0), irr::video::SColor(255, 255, 255, 255));
		driver->draw3DLine(irr::core::vector3df(minXPlace, minYPlace, 0),
				irr::core::vector3df(minXPlace, maxYPlace, 0), irr::video::SColor(255, 255, 255, 255));

		// draw the graph
		float stepX = (maxXPlace - minXPlace) /
			(maxXValue - minXValue);
		float stepY = (maxYPlace - minYPlace) /
			(maxYValue - minYValue);

		for (int channel = 0; channel < numberChannels; channel++)
		{
			if (channels[channel].values.size() == 0)
			{
				continue;
			}

			//driver->setMaterial(channels[channel].mat);
			std::vector<irr::video::S3DVertex> vertices;
			std::vector<irr::u32> indices;

			std::list<float>::iterator itY = channels[channel].values.begin();
			float tmpX, tmpY;
			tmpX = (0 - minXValue) * stepX + minXPlace;
			tmpY = (*itY - minYValue) * stepY + minYPlace;
			itY++;
			for (unsigned int i = 0; itY != channels[channel].values.end() && i < capacity + 1; itY++, i++)
			{
				/*driver->draw3DLine(irr::core::vector3df(tmpX, tmpY, 0),
						irr::core::vector3df((i - minXValue) * stepX + minXPlace,
								(*itY - minYValue) * stepY + minYPlace, 0),
								channels[channel].color);*/
				irr::video::S3DVertex vert;
				vert.Pos = irr::core::vector3df((i - minXValue) * stepX + minXPlace,
						(*itY - minYValue) * stepY + minYPlace, 0);
				vert.Color = channels[channel].color;
				indices.push_back(vertices.size());
				vertices.push_back(vert);

				/*driver->draw3DLine(
						irr::core::vector3df((i - minXValue - 0.2) * stepX + minXPlace,
								(*itY - minYValue - 0.2) * stepY + minYPlace, 0),
						irr::core::vector3df((i - minXValue + 0.2) * stepX + minXPlace,
								(*itY - minYValue + 0.2) * stepY + minYPlace, 0),
								channels[channel].color);
				driver->draw3DLine(
						irr::core::vector3df((i - minXValue + 0.2) * stepX + minXPlace,
								(*itY - minYValue - 0.2) * stepY + minYPlace, 0),
						irr::core::vector3df((i - minXValue - 0.2) * stepX + minXPlace,
								(*itY - minYValue + 0.2) * stepY + minYPlace, 0),
								channels[channel].color);*/
				tmpX = (i - minXValue) * stepX + minXPlace;
				tmpY = (*itY - minYValue) * stepY + minYPlace;
			}

			if (vertices.size() > 0)
			{
				irr::video::SMaterial mat;
				mat.Thickness = 6.0f;
				mat.EmissiveColor = channels[channel].color;
				driver->setMaterial(mat);
				driver->drawVertexPrimitiveList(&vertices[0], vertices.size(), &indices[0], vertices.size(), irr::video::EVT_STANDARD,
					irr::scene::EPT_POINTS, irr::video::EIT_32BIT);
			}
		}


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

	int inline getCapacity()
	{
		return capacity;
	}

};


#endif
