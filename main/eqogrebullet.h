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


#ifndef EQOGREBULLET_H
#define EQOGREBULLET_H

#include "equalizer/headers/initdata.h"
#include "equalizer/headers/config.h"
#include "bullet/headers/bulletphysics.h"

class Actor;
class DiffActor;

class eqOgreBullet
{
public:
    eqOgreBullet();

    bool init(int argc, char *argv[]);
    bool run();
    bool exit();

private:
    vr::Config *config;
    vr::InitData initData;
    BulletPhysics physics;

    /** Life time in seconds */
    float lifeTime;

    /**
     * @brief actorMap The game state: A map of actors each containing a graphical
     *  (OgreActor) and a physical (BulletActors) representation
     */
    std::map< std::string, std::shared_ptr< Actor > > actorMap;

    /** OgreActors added since the last frame. Must be cleared at the end of every frame */
    std::vector<std::shared_ptr<OgreActor>> newOgreActors;

    /**
     * @brief initEq Initiliases the Equalizer framework
     */
    bool initEq(int argc, char *argv[]);

    /**
     * @brief initActors Creates some objects and puts them into the actor map
     */
    void initActors();

    /**
     * @brief sync Transfers the changes bullet made earlier in this frame to the
     * OgreActors by creating DiffActors.
     * @return List of actors with new positions and orientations
     */
    std::vector<std::shared_ptr<DiffActor>> sync();

    void spawnRandomly();
    std::list<std::shared_ptr<BulletActor>> getBulletActors();
};

#endif // EQOGREBULLET_H
