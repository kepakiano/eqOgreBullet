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


#ifndef FRAMEDATA_H
#define FRAMEDATA_H

#include <eq/eq.h>
#include <eq/client/system.h>
#include <vector>
#include <memory>

#include "serializableogreactor.h"
#include "serializablediffactor.h"

class OgreActor;

namespace vr{
    /*
     * FrameData
     *
     * Data that may change with each new frame.
     */
    class FrameData : public co::Object
    {
    public:
        struct Data {
            float camera_pos[3];
            float camera_look_at[3];
            float camera_up[3];
            int64_t timestamp;
        } data;


        FrameData();

        std::vector<SerializableOgreActor> _newOgreActors;
        std::vector<SerializableDiffActor> _diffActors;

        virtual void setGameState (std::vector<std::shared_ptr<OgreActor>> newOgreActors,
                                   std::vector<std::shared_ptr<DiffActor>> diffActors);


    protected:
        virtual void getInstanceData(co::DataOStream &os);
        virtual void applyInstanceData(co::DataIStream &is);
        virtual ChangeType getChangeType() const;
    };
}
#endif // FRAMEDATA_H
