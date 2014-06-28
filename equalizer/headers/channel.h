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



#ifndef CHANNEL_H
#define CHANNEL_H

#include <eq/eq.h>
#include "pipe.h"
#include <OgreMatrix4.h>

namespace vr{
    /*
     * Channel
     */

    class Channel : public eq::Channel
    {
    public:
        Channel(eq::Window *parent);

    protected:
        virtual bool configInit(const eq::uint128_t &init_id);
        virtual void frameDraw(const eq::uint128_t &frame_id);
        Ogre::Matrix4 calcViewMatrix( Ogre::Vector3 eye, Ogre::Vector3 target, Ogre::Vector3 up ) const;
    };
}
#endif // CHANNEL_H
