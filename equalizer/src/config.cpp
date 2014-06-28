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


#include "../headers/config.h"
#include <iostream>

namespace vr{
    Config::Config(eq::ServerPtr parent)
        : eq::Config(parent)
        , spawn(false)
        , paused(false)
    {
    }

    bool Config::init(const InitData &init_data_template)
    {
        // Initialize InitData master instance
        init_data.data = init_data_template.data;

        init_data._ogreActors = init_data_template._ogreActors;

        // Register master instances
        registerObject(&frame_data);
        init_data.setFrameDataID(frame_data.getID());
        registerObject(&init_data);

        std::cout << "Keys:" << std::endl;
        std::cout << "\tp:\tToggle pause" << std::endl;
        std::cout << "\ts:\tSpawn an object in the air randomly" << std::endl;

        if (!eq::Config::init(init_data.getID()))
            return false;
        return true;
    }

    bool Config::exit()
    {
        const bool ret = eq::Config::exit();
        // Deregister master instances
        deregisterObject(&init_data);
        deregisterObject(&frame_data);

        return ret;
    }

    uint32_t Config::startFrame()
    {
        // Get time, in milliseconds. Can be used for animation purposes.
        frame_data.data.timestamp = getTime();

        // Set viewer head position and orientation
        eq::Matrix4f head_matrix = eq::Matrix4f::IDENTITY;

        // DO HEAD TRACKING HERE

        // If you want to do fancy stuff like head tracking, this is
        // the place you would want to do it. Just adjust head_matrix
        // accordingly

        head_matrix.set_translation(
                frame_data.data.camera_pos[0],
                frame_data.data.camera_pos[1],
                frame_data.data.camera_pos[2]);
        eq::Observer *first_observer = getObservers().at(0);
        first_observer->setHeadMatrix(head_matrix);

        // Commit new version of updated frame data
        const eq::uint128_t version = frame_data.commit();

        // Start this frame with the committed frame data
        return eq::Config::startFrame(version);
    }

    bool Config::handleEvent(const eq::ConfigEvent *event)
    {
        if (eq::Config::handleEvent(event))
            return true;

        if (event->data.type == eq::Event::KEY_PRESS) {
            switch (event->data.keyPress.key) {
            case 'p': paused = !paused;
             break;
            case 's': spawn = true;
            default:
                break;
            }
        }
        return true;
    }
}
