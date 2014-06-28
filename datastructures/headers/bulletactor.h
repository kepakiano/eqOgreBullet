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
#ifndef BULLETACTOR_H
#define BULLETACTOR_H

#include <string>
#include <vector>

#include "datastructures/headers/vec3.h"
#include "datastructures/headers/vec4.h"


enum CollisionShape{
    btBox = 0,
    btSphere = 1,
    btCapsule = 2,
    btCylinder = 3
};

/**
 * @brief The BulletActor class Physical representation of an actor.
 * Contains everything bullet needs to know about the object
 */
class BulletActor
{
private:
    BulletActor(const std::string & name,
                const Vec3 & pos,
                const float & mass,
                const Vec4 & rot,
                const float & restitution,
                const float & friction
                )
        :  _name(name)
        , _position(pos)
        , _mass(mass)
        , _orientation(rot)
        , _restitution(restitution)
        , _friction(friction)
    {

    }

public:

    friend class Actor;

    /**
     * @brief BulletActor - Constructor for primitive Collision shapes (Sphere / Box etc.)
     * @param name
     * @param shape
     * @param parameters - parameters for the shape. If shape is sphere then parameters[0] defines the radius. If shape is box parameters[0,1,2] defines width along x,y,z
     * @param pos
     * @param mass - 0 mass means the object is static and not moveable = infinite mass
     * @param rot
     * @param restitution
     * @param friction
     */
    BulletActor(const std::string & name,
                CollisionShape shape,
                const std::vector<float> & parameters,
                const Vec3 & pos,
                const float & mass,
                const Vec4 & rot,
                const float & restitution,
                const float & friction
                )
        : BulletActor(name, pos, mass, rot, restitution, friction)
    {
        _shape = shape;
        _parameters = parameters;
    }

    std::string         name()                  const {return _name;}
    CollisionShape      shape()                 const {return _shape;}
    std::vector<float>  parameters()            const {return _parameters;}
    Vec3                position()              const {return _position;}
    float               mass()                  const {return _mass;}
    Vec4                orientation()           const {return _orientation;}
    float               restitution()           const {return _restitution;}
    float               friction()              const {return _friction;}


    void setRotation(Vec4 rot)                  { _orientation = rot; }
    void setPosition(Vec3 pos)                  { _position = pos; }
    void setMass(float mass)                    { _mass = mass; }
    void setRestitution(float restitution)      { _restitution = restitution; }
    void setFriction(float friction)            { _friction = friction; }
    void setName(std::string name) { _name = name;}

private:
    BulletActor(){} // No empty BulletActors allowed
    std::string _name;

    CollisionShape _shape;
    std::vector<float> _parameters;

    Vec3 _position;
    float _mass;
    Vec4 _orientation;
    float _restitution;
    float _friction;
};

#endif // BULLETACTOR_H
