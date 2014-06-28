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



#ifndef DIFFACTOR_H
#define DIFFACTOR_H

#include <string>

#include "vec3.h"
#include "vec4.h"

/**
 * @brief The DiffActor class contains the name of a node in Ogre and its new position
 * and orientation.
 */

class DiffActor
{
public:
    DiffActor(std::string name, const Vec3 & pos, const Vec4 & rot)
        : _name(name), _position(pos), _orientation(rot)
    {
    }

    std::string nodeName() const {return _name;}
    Vec3 position() const {return _position;}
    Vec4 orientation() const {return _orientation;}

    void position(const Vec3 & position){_position = position;}
    void orientation(const Vec4 & orientation){_orientation = orientation;}

protected:
    DiffActor(){}

    std::string _name;
    Vec3 _position;
    Vec4 _orientation;

};

#endif // DIFFACTOR_H
