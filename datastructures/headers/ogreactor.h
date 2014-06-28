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


#ifndef OGREACTOR_H
#define OGREACTOR_H

#include <string>
#include "vec3.h"
#include "vec4.h"

/**
 * @brief The OgreActor class. It contains a lot of information OGRE needs
 * to create an object attached to a scene node. It is not complete, since
 * it does not contain information on how to create light sources for examples.
 */

class OgreActor
{
public:
    friend class Actor;
    OgreActor(std::string nodeName,
              Vec3 position,
              Vec4 orientation,
              std::string meshName,
              std::string entName,
              std::string materialName,
              bool castShadows,
              Vec3 scale)
        : _nodeName(nodeName)
        , _position(position)
        , _orientation(orientation)
        , _meshName(meshName)
        , _entName(entName)
        , _materialName(materialName)
        , _castShadows(castShadows)
        , _scale(scale)
    {

    }

    virtual ~OgreActor(){}

    std::string nodeName() const {return _nodeName;}
    Vec3 position() const {return _position;}
    Vec4 orientation() const {return _orientation;}
    std::string meshName() const {return _meshName;}
    std::string entName() const {return _entName;}
    std::string materialName() const {return _materialName;}
    bool castShadows() const {return _castShadows;}
    Vec3 scale() const {return _scale;}

    void setNodeName(std::string nodeName)          { _nodeName = nodeName;}
    void setOrientation(Vec4 ori)                   {_orientation = ori;}
    void setPosition(Vec3 pos)                      {_position = pos;}
    void setMeshName(std::string meshName)          { _meshName = meshName;}
    void setEntName(std::string entName)            { _entName = entName;}
    void setMaterialName(std::string materialName)  { _materialName = materialName;}
    void setCastShadows(bool castShadows)           { _castShadows = castShadows;}
    void setScale(Vec3 scale)                       { _scale = scale;}

protected:
    OgreActor(){} //  No empty OgreActors allowed

    std::string _nodeName;

    Vec3 _position;
    Vec4 _orientation;

    std::string _meshName;
    std::string _entName;
    std::string _materialName;

    bool _castShadows;

    Vec3 _scale;
};

#endif // OGREACTOR_H
