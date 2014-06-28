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



#include "../headers/initdata.h"

#include "../headers/serializablediffactor.h"
#include "../headers/eqserialization.h"

namespace vr
{

InitData::InitData()
{
    data.seed = 1;
    data.tracking = false;
    _ogreActors.clear();
}

void InitData::setFrameDataID(const eq::uint128_t &id)
{
    data.frame_data_id = id;
}

const eq::uint128_t &InitData::getFrameDataID() const
{
    return data.frame_data_id;
}

void InitData::getInstanceData(co::DataOStream &os)
{
    os << data << _ogreActors;
}

void InitData::applyInstanceData(co::DataIStream &is)
{
    is >> data >> _ogreActors;
}

co::Object::ChangeType InitData::getChangeType() const
{
    return STATIC;
}
}
