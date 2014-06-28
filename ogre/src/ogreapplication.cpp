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


#include "../headers/ogreapplication.h"

#include <OgreConfigFile.h>
#include <OgreRenderSystem.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreCamera.h>
#include <OgreLogManager.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreSceneNode.h>

#include "../../datastructures/headers/ogreactor.h"
#include "../../datastructures/headers/diffactor.h"
#include "../../datastructures/headers/vec3.h"
#include "../../datastructures/headers/vec4.h"

#include <OgreParticleSystem.h>

Ogre::Quaternion toOgreQuaternion(const Vec4 & vec)
{
    return Ogre::Quaternion(vec.w(), vec.x(), vec.y(), vec.z());
}

Ogre::Vector3 toOgreVector3(const Vec3 & vec)
{
    return Ogre::Vector3(vec.x(), vec.y(), vec.z());
}

OgreApplication::OgreApplication()
    : mPluginsCfg(Ogre::StringUtil::BLANK)
    , mResourcesCfg(Ogre::StringUtil::BLANK)
    , mRoot(0)
    , mWindow(nullptr)
{
}
OgreApplication::~OgreApplication()
{
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
    delete mRoot;
    delete mCamera;
    delete mSceneMgr;
}

bool OgreApplication::initStart(void)
{

    mPluginsCfg = "../configs/plugins.cfg";
    mResourcesCfg = "../configs/resources.cfg";

    Ogre::LogManager * lm = new Ogre::LogManager();
    lm->createLog("OgreLogfile.log", true, false, false);

    // construct Ogre::Root
    mRoot = new Ogre::Root(mPluginsCfg, "", "");

    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                archName, typeName, secName);
        }
    }

    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../assets/meshes", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../assets/images", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../assets/materials", "FileSystem");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation("../assets/particles", "FileSystem");

    // Do not add this to the application
    Ogre::RenderSystem *rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
    mRoot->setRenderSystem(rs);
    rs->setConfigOption("Full Screen", "No");
    rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit colour");
    rs->setStencilCheckEnabled(true);

    mRoot->initialise(false);

    running = true;
    return true;
}
bool OgreApplication::initFinish(std::list<std::shared_ptr<OgreActor>> actors)
{

    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

    // Set default mipmap level (note: some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    // initialise all resource groups
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    createSceneManager();
    createCamera();
    createScene(actors);
}

void OgreApplication::createSceneManager()
{
    // Create the SceneManager, in this case a generic one
    mSceneMgr = mRoot->createSceneManager("DefaultSceneManager");
}

void OgreApplication::createCamera()
{
    // Create the camera
    mCamera = mSceneMgr->createCamera("PlayerCam");
    mCamera->setPosition(Ogre::Vector3(0, 1.78, 0));

    // Look back along -Z
    mCamera->lookAt(Ogre::Vector3(0, 1.78, -1.0));
    mCamera->setNearClipDistance(5);
}

void OgreApplication::createScene(std::list<std::shared_ptr<OgreActor> > actors)
{
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0, 0, 0));
    mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);

    for(std::shared_ptr<OgreActor> actor : actors){
        addActor(actor);
    }

    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);

    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane, 1500, 1500, 20, 20, true, 1, 500, 500, Ogre::Vector3::UNIT_Z);

    Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
    mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(entGround);

    entGround->setMaterialName("Examples/Rockwall");
    entGround->setCastShadows(false);

    Ogre::Light* directionalLight = mSceneMgr->createLight("directionalLight");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue(0.75, 0.75, 0.75));
    directionalLight->setSpecularColour(Ogre::ColourValue(1.0, 1.0, 1.0));
    directionalLight->setDirection(Ogre::Vector3( 0, -1, -1 ));

    Ogre::Vector3 sunPosition(15, 15, -90);
    Ogre::Light* pointLight = mSceneMgr->createLight("pointLight");
    pointLight->setType(Ogre::Light::LT_POINT);
    pointLight->setPosition(sunPosition);
    pointLight->setDiffuseColour(1.0, 1.0, 1.0);
    pointLight->setSpecularColour(1.0, 1.0, 1.0);

    Ogre::SceneNode *sunNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("SUN_NODE");
    sunNode->setPosition(sunPosition);
    sunNode->attachObject(mSceneMgr->createParticleSystem("SUN_PARTICLE", "Space/Sun"));
}

void OgreApplication::updateNodes(std::list<std::shared_ptr<OgreActor> > newActors, std::list<std::shared_ptr<DiffActor> > diffActor)
{
    for(std::shared_ptr<OgreActor> actor : newActors){
        addActor(actor);
    }

    for (std::shared_ptr<DiffActor> actor : diffActor)
    {
        Ogre::SceneNode *node;
        try {
          node = getSceneManager()->getSceneNode(actor->nodeName());
          node->setPosition(toOgreVector3(actor->position()));
          node->setOrientation(toOgreQuaternion(actor->orientation()));
        }
        catch (Ogre::ItemIdentityException e){
            std::cerr << "OgreApplication::updateNodes, warning: Node " << actor->nodeName() << " was not found. DiffActor was ignored." << std::endl;
        }
    }
}

void OgreApplication::addActor(std::shared_ptr<OgreActor> actor)
{

    try {
        Ogre::Entity* ent;
        ent = mSceneMgr->createEntity(actor->entName(), actor->meshName());
        ent->setCastShadows(actor->castShadows());
        ent->setMaterialName(actor->materialName());

        Ogre::SceneNode *node;
        node = mSceneMgr->getRootSceneNode()->createChildSceneNode(actor->nodeName());
        node->setPosition(toOgreVector3(actor->position()));
        node->setOrientation(toOgreQuaternion(actor->orientation()));
        node->scale(actor->scale().x(),actor->scale().y(), actor->scale().z());
        node->attachObject(ent);
    }
    catch (Ogre::ItemIdentityException e){
        std::cerr << "OgreApplication::addActor, warning: Node " << actor->nodeName() << " already exists. Actor was not added." << std::endl;
    }
}


bool OgreApplication::isRunning()
{
    return running;
}

void OgreApplication::exit()
{

}

bool OgreApplication::windowClosing(Ogre::RenderWindow* rw){
    if(mWindow && rw == mWindow)
        running = false;
    return true;
}
