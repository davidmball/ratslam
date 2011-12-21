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

#ifndef _IRR_UTIL_H
#define _IRR_UTIL_H

#include <irrlicht/irrlicht.h>
#include <irrlicht/IEventReceiver.h>

class IrrEventReceiver : public irr::IEventReceiver
{
public:
	virtual bool OnEvent(const irr::SEvent& event)
	{
		if (event.EventType == irr::EET_KEY_INPUT_EVENT)
			KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

		if (event.EventType == irr::EET_MOUSE_INPUT_EVENT)
		{

			switch (event.MouseInput.Event)
			{
				case irr::EMIE_LMOUSE_DOUBLE_CLICK:
				case irr::EMIE_MMOUSE_DOUBLE_CLICK:
				case irr::EMIE_RMOUSE_DOUBLE_CLICK:
					Coords.X = event.MouseInput.X;
					Coords.Y = event.MouseInput.Y;
					MouseDoubleClickEvent[event.MouseInput.Event] = true;
					break;
			}
		}

		return false;
	}

	virtual bool IsKeyDown(irr::EKEY_CODE keyCode) const
	{
		return KeyIsDown[keyCode];
	}

	virtual bool IsMouseDoubleClick(irr::EMOUSE_INPUT_EVENT mouseCode, irr::core::position2d<irr::s32> &MouseCoords)
	{
		bool isEvent = MouseDoubleClickEvent[mouseCode];
		MouseDoubleClickEvent[mouseCode] = false; 
		MouseCoords = Coords; // todo: coords accurate really only for last double click but double clikcing shoudl happen more than once per frame
		return isEvent;
	}

	IrrEventReceiver()
	{
		for (unsigned int i=0; i < irr::KEY_KEY_CODES_COUNT; ++i)
			KeyIsDown[i] = false;

		for (unsigned int i=0; i < irr::EMIE_COUNT; i++)
			MouseDoubleClickEvent[i] = false;
	}

private:
	bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];
	bool MouseDoubleClickEvent[irr::EMIE_COUNT];
	irr::core::position2d<irr::s32> Coords;
};


#endif // _IRR_UTIL_H
