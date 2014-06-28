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



#ifndef INITDATA_H
#define INITDATA_H

#include "serializableogreactor.h"

#include <eq/eq.h>

namespace vr{
    /*
     * InitData
     *
     * Data that is set only once and remains constant for all frames.
     */

    class InitData : public co::Object
    {
        public:
            struct {
                eq::uint128_t frame_data_id;
                unsigned int seed;
                bool tracking;
            } data;

            InitData();

            std::vector<SerializableOgreActor> _ogreActors;

            void setFrameDataID(const eq::uint128_t &id);
            const eq::uint128_t &getFrameDataID() const;

        protected:
            virtual void getInstanceData(co::DataOStream &os);
            virtual void applyInstanceData(co::DataIStream &is);
            virtual ChangeType getChangeType() const;
    };
}
#endif // INITDATA_H
