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


#include "../headers/framedata.h"

#include "../../datastructures/headers/ogreactor.h"
#include "../../datastructures/headers/diffactor.h"

#include "../headers/eqserialization.h"

namespace vr
{

FrameData::FrameData()
{
    data.camera_pos[0] = 0.0f;
    data.camera_pos[1] = 1.78f;
    data.camera_pos[2] = 0.0f;

    data.camera_look_at[0] = 0.0f;
    data.camera_look_at[1] = 1.78f;
    data.camera_look_at[2] = -10.0f;

    data.camera_up[0] = 0.0f;
    data.camera_up[1] = 1.0f;
    data.camera_up[2] = 0.0f;

    data.timestamp = 0;
}

void FrameData::getInstanceData(co::DataOStream &os)
{
    os << data << _newOgreActors << _diffActors;
}

void FrameData::applyInstanceData(co::DataIStream &is)
{
    is >> data >> _newOgreActors >> _diffActors;
}

co::Object::ChangeType FrameData::getChangeType() const
{
    return INSTANCE;
}

void FrameData::setGameState (std::vector<std::shared_ptr<OgreActor>> newOgreActors, std::vector<std::shared_ptr<DiffActor> > diffActors)
{
    _newOgreActors.clear();
    for(std::shared_ptr<OgreActor> actor : newOgreActors) {
        _newOgreActors.push_back(SerializableOgreActor(actor));
    }
    _diffActors.clear();
    for(std::shared_ptr<DiffActor> actor : diffActors){
        _diffActors.push_back(SerializableDiffActor(actor));
    }
}
}
