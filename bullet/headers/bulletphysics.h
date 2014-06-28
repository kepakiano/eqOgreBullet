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


#pragma once
#ifndef BULLETPHYSICS_H
#define BULLETPHYSICS_H

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <list>
#include <queue>

#include <btBulletDynamicsCommon.h>

#include "datastructures/headers/vec3.h"

class BulletRigidBody;
class BulletActor;

class BulletPhysics
{
public:

    BulletPhysics();
    virtual ~BulletPhysics();

    /**
     * @brief init Initialises the dynamics world, creates the ground plane and
     * creates objects for every BulletActor in 'actors'
     */
    bool init(std::list<std::shared_ptr<BulletActor>> actors);

    /**
     * @brief update Do one simulation step
     * @param elapsedTime The time the last frame took in seconds
     */
    void update(float elapsedTime);

    /**
     * @brief After an update, the rigidBodies with their positions and orientations are of out sync
     * with their representation in the game state (eqOgreBullet::actorMap). Thus, we copy the changes
     * made by the dynamics world over to the BulletActors.
     */
    void sync(std::list<std::shared_ptr<BulletActor>> actors);


private:

    btDefaultCollisionConfiguration* _collisionConfiguration;
    btCollisionDispatcher* _dispatcher;
    btBroadphaseInterface* _broadphase;
    btSequentialImpulseConstraintSolver* _solver;
    btDiscreteDynamicsWorld* _dynamicsWorld;

    /** Keeps track of the BulletActors for the synchronisation */
    std::map < std::string, std::shared_ptr < BulletRigidBody > > _rigidBodies;

    btVector3 _gravity;

    /** Keep tracks of the shapes, we release memory at exit. */
    btAlignedObjectArray<btCollisionShape*> *_collisionShapes;

    /**
     * @brief addActor Adds the given actor to the dynamics world and creates a BulletRigidBody for it
     */
    void addActor(std::shared_ptr<BulletActor> actor);

};

#endif // BULLETPHYSICS_H
