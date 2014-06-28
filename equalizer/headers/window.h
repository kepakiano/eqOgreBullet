/*
* Copyright (C) 2014
* Sebastian Schmitz <sschmitz@informatik.uni-siegen.de>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef WINDOW_H
#define WINDOW_H

#include "pipe.h"

#ifdef override
#  undef override
#endif
#include <OgreRenderWindow.h>
#include <OgreCamera.h>

class DynCalib;

namespace vr{

    /*
     * Window
     */

    class Window : public eq::Window
    {
    public:
        Window(eq::Pipe *parent);

        Ogre::RenderWindow* mWindow;

        void addViewport();
        void setViewport(float left, float top, float width, float height);

        void render();

    private:
        bool _ogreInitialised;
        Ogre::Viewport* _vp;

    protected:
        virtual bool configInitGL(const eq::uint128_t &init_id);
        virtual bool configExitGL();
        virtual void frameStart( const eq::uint128_t& frameID, const uint32_t frameNumber );
        virtual void frameFinish( const eq::uint128_t& frameID, const uint32_t frameNumber );
    };
}
#endif // WINDOW_H
