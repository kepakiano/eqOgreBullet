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


#include "../headers/channel.h"

#include <iostream>

#include "../headers/pipe.h"
#include "../headers/window.h"
#include "../headers/node.h"
#include "../headers/framedata.h"
#include "../headers/initdata.h"

#include "../../ogre/headers/ogreapplication.h"

#include <GL/glut.h>


Ogre::Matrix4 toOgreMatrix(eq::Matrix4f eqMat){
    Ogre::Matrix4 ogreMat;
    for(size_t x = 0; x < 4; x++)
        for(size_t y = 0; y < 4; y++)
            ogreMat[x][y] = eqMat[x][y];
    return ogreMat;
}

namespace vr {
    Channel::Channel(eq::Window *parent)
    : eq::Channel(parent)
    {
    }

    bool Channel::configInit(const eq::uint128_t &init_id)
    {
        if (!eq::Channel::configInit(init_id))
            return false;
        return true;
    }

    void Channel::frameDraw(const eq::uint128_t &frame_id)
    {
        // Setup OpenGL state (view frustum etc.)
        eq::Channel::frameDraw(frame_id);

        // Get data
        const Config *config = static_cast<Config *>(getConfig());
        const Node *node = static_cast<Node *>(getNode());
        const InitData &init_data = node->getInitData();
        const Pipe *pipe = static_cast<Pipe *>(getPipe());
        const FrameData &frame_data = pipe->getFrameData();
        Window *window = static_cast<Window *>(getWindow());        

        Ogre::Camera *cam = pipe->_ogre->getCamera();

        // Set the projection matrix
        const eq::Frustumf & frustum = getFrustum();

        cam->setCustomProjectionMatrix(true, toOgreMatrix(frustum.compute_matrix()));

        const eq::Matrix4f eqViewMatrix = getHeadTransform();
        // Adjust the view matrix according to equalizer's view matrix
        Ogre::Matrix4 ogreViewMatrix = toOgreMatrix(eqViewMatrix);
        cam->setCustomViewMatrix(true, ogreViewMatrix);
        cam->setNearClipDistance(frustum.near_plane());
        cam->setFarClipDistance(frustum.far_plane());

        // Set the viewport
        eq::PixelViewport winPvp = window->getPixelViewport();
        eq::PixelViewport pvp	 = getPixelViewport();
        window->setViewport((float)pvp.x/(float)winPvp.w,
                          1.0f - (float)(pvp.h + pvp.y)/(float)winPvp.h,
                          (float)pvp.w/(float)winPvp.w,
                          (float)pvp.h/(float)winPvp.h);

        // Render
        window->render();
    }
}
