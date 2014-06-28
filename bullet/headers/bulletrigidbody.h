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
#ifndef BULLETRIGIDBODY_H
#define BULLETRIGIDBODY_H


#include <string>
#include <btBulletDynamicsCommon.h>

/**
 * @brief The BulletRigidBody class This is a wrapper class for btRigidBody.
 * With this class we can make sure, our meshes and their
 * physical representation are connected.
 */
class BulletRigidBody
{
public:
    BulletRigidBody(const btRigidBody::btRigidBodyConstructionInfo & constructionInfo, std::string name, bool isGraphical);

    virtual ~BulletRigidBody();

    btRigidBody* rigidBody() const {return _rigidBody;}
    std::string name() const {return _name;}

private:
    std::string _name;
    btRigidBody* _rigidBody;
};

#endif // BULLETRIGIDBODY_H
