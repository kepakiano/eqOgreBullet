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


#ifndef CONFIG_H
#define CONFIG_H

#include <eq/eq.h>

#include "../headers/initdata.h"
#include "../headers/framedata.h"


class MoveConnector;

namespace vr{
    /*
     * Config
     *
     * Controls all instances and transfers positions/orientations of scene nodes to the FrameData.
     */

    class Config : public eq::Config
    {
    private:
        InitData init_data;                 // Master InitData instance
        FrameData frame_data;               // Master FrameData instance

    public:
        Config(eq::ServerPtr parent);
        bool init(const InitData &init_data_template);
        virtual bool exit();
        virtual uint32_t startFrame();

        bool spawn;
        bool paused;

        /**
         * @brief setGameState - Call this method to change the state of the scene graph
         * @param newOgreActors - Actors (scene nodes) you wish to add to the scene
         * @param diffActors - Updated positions and orientations of already created scene nodes
         */
        virtual void setGameState (std::vector<std::shared_ptr<OgreActor>> newOgreActors,
                                   std::vector<std::shared_ptr<DiffActor>> diffActors = std::vector<std::shared_ptr<DiffActor>>())
        {
            frame_data.setGameState(newOgreActors, diffActors);
        }

        bool handleEvent(const eq::ConfigEvent *event);


    };
}

#endif // CONFIG_H
