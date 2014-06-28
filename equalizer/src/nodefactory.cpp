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


#include "../headers/nodefactory.h"

#include "../headers/config.h"
#include "../headers/node.h"
#include "../headers/pipe.h"
#include "../headers/window.h"
#include "../headers/channel.h"


namespace vr{
    eq::Config *NodeFactory::createConfig(eq::ServerPtr parent)
    {
        return new Config(parent);
    }

    eq::Node *NodeFactory::createNode(eq::Config *parent)
    {
        return new Node(parent);
    }

    eq::Pipe *NodeFactory::createPipe(eq::Node *parent)
    {
        return new Pipe(parent);
    }

    eq::Window *NodeFactory::createWindow(eq::Pipe *parent)
    {
        return new Window(parent);
    }

    eq::Channel *NodeFactory::createChannel(eq::Window *parent)
    {
        return new Channel(parent);
    }

}
