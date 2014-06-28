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


#include "../headers/pipe.h"

#include "../headers/node.h"
#include "../../ogre/headers/ogreapplication.h"
#include "../../datastructures/headers/diffactor.h"

namespace vr{

    Pipe::Pipe(eq::Node *parent)
        : eq::Pipe(parent), _ogreInitialised(false)
    {
    }

    Pipe::~Pipe()
    {
        _ogre.reset();
    }

    const FrameData &Pipe::getFrameData() const
    {
        return frame_data;
    }

    bool Pipe::configInit(const eq::uint128_t &init_id)
    {
        _ogre = std::make_shared<OgreApplication>();

        if (!eq::Pipe::configInit(init_id))
            return false;
        if (!_ogre->initStart())
            return false;
        // Map our FrameData instance to the master instance
        const Node *node = static_cast<Node *>(getNode());
        const InitData &init_data = node->getInitData();
        const eq::uint128_t frame_data_id = init_data.getFrameDataID();
        eq::Config *config = getConfig();

        return config->mapObject(&frame_data, frame_data_id);
    }

    bool Pipe::configExit()
    {
        // Unmap our FrameData instance
        eq::Config *config = getConfig();
        config->unmapObject(&frame_data);
        return eq::Pipe::configExit();
    }

    void Pipe::frameStart(const eq::uint128_t &frame_id, const uint32_t frame_number)
    {
        if (!_ogreInitialised){
            finalizeOgre();
        }
        // Update our FrameData instance from the master instance before drawing
        frame_data.sync(frame_id);

        const Config *config = static_cast<Config*>(getConfig());

        // Get the newly created OgreActors and the DiffActors from the
        // FrameData and send them to our framework. Note that the position
        // of a scene node will not change, if you don't provide an actor for it
        std::list<std::shared_ptr<OgreActor>> newOgreActors;
        std::list<std::shared_ptr<DiffActor>> diffActors;

        for(size_t i = 0; i < frame_data._newOgreActors.size(); i++){
            SerializableOgreActor actor = frame_data._newOgreActors[i];
            newOgreActors.push_back(std::make_shared<OgreActor>(
                                        actor.nodeName(),
                                        actor.position(),
                                        actor.orientation(),
                                        actor.meshName(),
                                        actor.entName(),
                                        actor.materialName(),
                                        actor.castShadows(),
                                        actor.scale()
                                        )
                                    );
        }

        for(size_t i = 0; i < frame_data._diffActors.size(); i++){
            SerializableDiffActor actor = frame_data._diffActors[i];
            diffActors.push_back(std::make_shared<DiffActor>(
                                     actor.nodeName(),
                                     actor.position(),
                                     actor.orientation()
                                     )
                                 );
        }


        _ogre->updateNodes(newOgreActors, diffActors);

        startFrame(frame_number);
    }

    void Pipe::finalizeOgre()
    {
        const Config *config = static_cast<Config*>(getConfig());
        const Node *node = static_cast<Node *>(getNode());
        const InitData &init_data = node->getInitData();

        // Get the OgreActors we put into the InitData and send them to our OGRE framework
        std::list<std::shared_ptr<OgreActor>> ogreActors;
        for(size_t i = 0; i < init_data._ogreActors.size(); i++){
            SerializableOgreActor actor = init_data._ogreActors[i];
            ogreActors.push_back(std::make_shared<OgreActor>(
                                        actor.nodeName(),
                                        actor.position(),
                                        actor.orientation(),
                                        actor.meshName(),
                                        actor.entName(),
                                        actor.materialName(),
                                        actor.castShadows(),
                                        actor.scale()
                                        )
                                    );
        }


        _ogre->initFinish(ogreActors);

        _ogreInitialised = true;

    }
}
