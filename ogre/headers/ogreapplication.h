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



#ifndef OGREAPPLICATION_H
#define OGREAPPLICATION_H

#include <eq/eq.h>

#ifdef override
#  undef override
#endif
#include <OgreRoot.h>
#include <OgreString.h>
#include <OgreRenderWindow.h>
#include <OgreWindowEventUtilities.h>

#include <list>
class OgreActor;
class DiffActor;

class OgreApplication : Ogre::WindowEventListener
{
public:
    OgreApplication();
    ~OgreApplication();

    bool initStart(void);
    bool initFinish(std::list<std::shared_ptr<OgreActor>> actors);
    bool isRunning();
    void exit();

    Ogre::Root * getRoot() const {return mRoot;}
    Ogre::Camera * getCamera() const {return mCamera;}
    Ogre::SceneManager * getSceneManager() const {return mSceneMgr;}

    void updateNodes(std::list<std::shared_ptr<OgreActor>> actors, std::list<std::shared_ptr<DiffActor>> diffActor);

    virtual bool windowClosing(Ogre::RenderWindow * rw);
    void setRenderWindow(Ogre::RenderWindow *w){mWindow = w;}
private:
    bool running;

    Ogre::Root *mRoot;
    Ogre::Camera* mCamera;
    Ogre::String mPluginsCfg;
    Ogre::String mResourcesCfg;
    Ogre::SceneManager* mSceneMgr;
    Ogre::RenderWindow *mWindow;

    void createSceneManager();
    void createCamera();
    void createScene(std::list<std::shared_ptr<OgreActor>> actors);

    void addActor(std::shared_ptr<OgreActor> actor);
};

#endif // OGREAPPLICATION_H
