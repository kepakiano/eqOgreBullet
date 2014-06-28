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

#pragma once
#ifndef ACTOR_H
#define ACTOR_H

#include <memory>
#include <string>

#include "ogreactor.h"
#include "bulletactor.h"

/**
 * @brief The Actor class Physical and graphical representation of an actor (an object in the game).
 * Every actor has a unique name, which can be used to indentify it throughout the game.
 */

class Actor
{
public:

    Actor(std::string name, std::shared_ptr<BulletActor> & bulletActor, std::shared_ptr<OgreActor> & ogreActor)
        : _name(name)
        , _bulletActor(bulletActor)
        , _ogreActor(ogreActor)
    {
        assert((name.compare(_bulletActor->_name) == 0) && "Actor and BulletActor must have the same name.");
        assert((name.compare(_ogreActor->_nodeName) == 0) && "Actor and OgreActor must have the same name.");
    }

    virtual ~Actor()
    {
        _bulletActor.reset();
        _ogreActor.reset();
    }

    std::shared_ptr<BulletActor> bulletActor() const {return _bulletActor;}
    std::shared_ptr<OgreActor> ogreActor() const {return _ogreActor;}
    std::string name() const {return _name;}

protected:
    /**
     * @brief _name Unique name. Another actor with this name must not exist, because Ogre demands unique scene node names.
     */
    std::string _name;

private:
    Actor(); // Creating an actor without BulletActor and OgreActor is forbidden

    /**
     * @brief _bulletActor Physical representation of this actor
     */
    std::shared_ptr<BulletActor> _bulletActor;

    /**
     * @brief _ogreActor Graphical representation of this actor
     */
    std::shared_ptr<OgreActor> _ogreActor;
};

#endif // ACTOR_H
