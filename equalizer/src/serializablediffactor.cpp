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



#include "../headers/serializablediffactor.h"

SerializableDiffActor::SerializableDiffActor()
{
}

SerializableDiffActor::SerializableDiffActor(std::shared_ptr<DiffActor> actor)
    : DiffActor(actor->nodeName(), actor->position(), actor->orientation())
{

}

SerializableDiffActor::~SerializableDiffActor()
{
}


void  SerializableDiffActor::writeToStream (co::DataOStream& os) const
{
    os << _name;
    os << _position.x() << _position.y() << _position.z();
    os << _orientation.x() << _orientation.y() << _orientation.z() << _orientation.w();
}

void SerializableDiffActor::loadFromStream(co::DataIStream& is)
{
    float p_x, p_y, p_z, o_x, o_y, o_z, o_w;

    is >> _name;
    is >> p_x >> p_y >> p_z;
    is >> o_x >> o_y >> o_z >> o_w;

    _position.x(p_x); _position.y(p_y); _position.z(p_z);
    _orientation.x(o_x); _orientation.y(o_y); _orientation.z(o_z); _orientation.w(o_w);
}
