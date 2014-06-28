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



#include "../headers/node.h"

#include <GL/glut.h>

namespace vr{
    Node::Node(eq::Config *parent)
    : eq::Node(parent)
    {
        // Initialize glut (only once per proces)
        if (!getenv("DISPLAY")) {
            setenv("DISPLAY", ":0.0", 0);
        }
        int fake_argc = 1;
        char fake_argv0[] = "fake";
        char *fake_argv[] = { fake_argv0, NULL };
        glutInit(&fake_argc, fake_argv);
    }

    const InitData &Node::getInitData() const
    {
        return init_data;
    }

    bool Node::configInit(const eq::uint128_t &init_id)
    {
        if (!eq::Node::configInit(init_id))
            return false;
        // Map our InitData instance to the master instance
        eq::Config *config = getConfig();
        return config->mapObject(&init_data, init_id);
    }

    bool Node::configExit()
    {
        // Unmap our InitData instance
        eq::Config *config = getConfig();
        config->unmapObject(&init_data);
        return eq::Node::configExit();
    }
}
