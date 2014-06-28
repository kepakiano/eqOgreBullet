/*
 * Copyright (C) 2013-2014
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


#ifndef EQSERIALIZATION_H
#define EQSERIALIZATION_H

#include "serializablediffactor.h"
#include "serializableogreactor.h"
#include <eq/eq.h>


/*
 * Partial template specializations for our custom classes.
 */
namespace co
{

template<> inline
DataOStream& DataOStream::operator << ( const SerializableOgreActor& value )
{
    value.writeToStream(*this);
    return *this;
}
template<> inline
DataIStream& DataIStream::operator >> ( SerializableOgreActor& value )
{
    value.loadFromStream(*this);
    return *this;
}

template<> inline
DataOStream& DataOStream::operator << ( const SerializableDiffActor& value )
{
    value.writeToStream(*this);
    return *this;
}
template<> inline
DataIStream& DataIStream::operator >> ( SerializableDiffActor& value )
{
    value.loadFromStream(*this);
    return *this;
}

}
#endif // EQSERIALIZATION_H
