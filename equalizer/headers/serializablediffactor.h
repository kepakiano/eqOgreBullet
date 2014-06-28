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



#ifndef SERIALIZABLEDIFFACTOR_H
#define SERIALIZABLEDIFFACTOR_H

#include <memory>
#include <eq/eq.h>
#include "../../datastructures/headers/diffactor.h"
/**
 * @brief The SerializableDiffActor class is separated from its base class DiffActor,
 * because nobody but the part of our program concerned with equalizer has to know about it.
 * It is only there so we can serialize vectors of DiffActors in our frame data.
 *
 */

class SerializableDiffActor : public DiffActor
{
public:
    SerializableDiffActor();
    SerializableDiffActor(std::shared_ptr<DiffActor> actor);
    virtual ~SerializableDiffActor();

    SerializableDiffActor(std::string nodeName,
              Vec3 position,
              Vec4 orientation
              );


    virtual void writeToStream(co::DataOStream& os) const;
    virtual void loadFromStream (co::DataIStream& is);
};

#endif // SERIALIZABLEDIFFACTOR_H
