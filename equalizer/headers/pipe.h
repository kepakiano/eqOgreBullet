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



#ifndef PIPE_H
#define PIPE_H

#include "framedata.h"
#include "config.h"

#include <memory>


class OgreApplication;

namespace vr{

    /**
     * @brief The Pipe class. Every pipe handles its own copy of an OgreApplication.
     * The scene graph thus exists once per node and you can changed its state by calling
     * Config::setGameState(). Note that there can only be one pipe per node! Otherwise
     * Ogre::Root complains that it is not a singleton.
     */
    class Pipe : public eq::Pipe
    {
    private:
        FrameData frame_data;
        bool _ogreInitialised;

    public:
        Pipe(eq::Node *parent);
        ~Pipe();
        const FrameData &getFrameData() const;

        std::shared_ptr<OgreApplication> _ogre;

        void finalizeOgre();

    protected:
        virtual bool configInit(const eq::uint128_t &init_id);
        virtual bool configExit();
        virtual void frameStart(const eq::uint128_t &frame_id, const uint32_t frame_number);
    };

}
#endif // PIPE_H
