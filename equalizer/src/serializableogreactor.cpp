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


#include "../headers/serializableogreactor.h"

SerializableOgreActor::SerializableOgreActor()
{
}

SerializableOgreActor::SerializableOgreActor(std::shared_ptr<OgreActor> actor)
    : OgreActor(actor->nodeName(), actor->position(),
      actor->orientation(), actor->meshName(), actor->entName(),
                actor->materialName(), actor->castShadows(), actor->scale())
{

}

SerializableOgreActor::SerializableOgreActor(std::string nodeName,
                     Vec3 position,
                     Vec4 orientation,
                     std::string meshName,
                     std::string entName,
                     std::string materialName,
                     bool castShadows, Vec3 scale)
    : OgreActor( nodeName, position, orientation, meshName, entName,
                 materialName, castShadows, scale )
{
}

SerializableOgreActor::SerializableOgreActor(const SerializableOgreActor& rhs)
    : OgreActor( rhs.nodeName(), rhs.position(), rhs.orientation(),
                 rhs.meshName(), rhs.entName(), rhs.materialName(),
                 rhs.castShadows(), rhs.scale( ))
{

}

SerializableOgreActor::~SerializableOgreActor()
{

}

void SerializableOgreActor::writeToStream (co::DataOStream& os) const
{
    os << _nodeName;
    os << _position.x() << _position.y() << _position.z();
    os << _orientation.x() << _orientation.y() << _orientation.z() << _orientation.w();
    os << _meshName;
    os << _entName;
    os << _materialName;
    os << _castShadows;
    os << _scale.x() << _scale.y() << _scale.z();
}

void SerializableOgreActor::loadFromStream (co::DataIStream& is)
{
    float p_x, p_y, p_z, o_x, o_y, o_z, o_w, s_x, s_y, s_z;

    is >> _nodeName;
    is >> p_x >> p_y >> p_z;
    is >> o_x >> o_y >> o_z >> o_w;
    is >> _meshName;
    is >> _entName;
    is >> _materialName;
    is >> _castShadows;
    is >> s_x >> s_y >> s_z;

    _position.x(p_x); _position.y(p_y); _position.z(p_z);
    _orientation.x(o_x); _orientation.y(o_y); _orientation.z(o_z); _orientation.w(o_w);
    _scale.x(s_x); _scale.y(s_y); _scale.z(s_z);
}
