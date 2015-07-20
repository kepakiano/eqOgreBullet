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


#include "eqogrebullet.h"

#include <sys/time.h>

#include "equalizer/headers/nodefactory.h"
#include "datastructures/headers/bulletactor.h"
#include "datastructures/headers/actor.h"
#include "equalizer/headers/serializableogreactor.h"

eqOgreBullet::eqOgreBullet()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double init = tv.tv_sec + tv.tv_usec * 0.000001;
    srandom(init);
}

bool eqOgreBullet::initEq(int argc, char *argv[])
{
    vr::NodeFactory nodeFactory;
    if (!eq::init(argc, argv, &nodeFactory))
    {
        std::cerr << "Equalizer initialization failed!" << std::endl;
        return 1;
    }

    /* Initialize InitData */
    initData.data.seed = static_cast<unsigned int>(time(NULL));

    for(auto a : actorMap)
        initData._ogreActors.push_back(SerializableOgreActor(a.second->ogreActor()));

    /* Get a configuration */
    bool success = true;
    config = static_cast<vr::Config *>(eq::getConfig(argc, argv));
    if (config)
    {
        /* Initialize configuration */
        if (!config->init(initData))
        {
            std::cerr << "EqConfig initialization failed" << std::endl;
            success = false;
        }
    }
    else
    {
        std::cerr << "Cannot get configuration" << std::endl;
        success = false;
    }
    return success;
}

void eqOgreBullet::initActors()
{
    // Create some spheres falling from the sky
    for(int i = 0; i < 21; i++)
    {
        const Vec3 pos((i - 5)/3.0f, 2+i*2, -3);
        std::stringstream ssName;
        ssName << "sphere_" << i;
        const std::string name = ssName.str();
        std::vector<float> parameters;
        parameters.push_back(0.1);

        std::shared_ptr<OgreActor> ogreActor = std::make_shared<OgreActor>(
                    name,                        // Name of the Ogre::SceneNode
                    pos,                         
                    Vec4(0,0,0,1),               // Orientation, will be transformed into a (x,y,z,w) quaternion
                    "sphere.mesh",               // Name of the mesh, see assets folder
                    name,                        // Name of the Ogre::Entity
                    "Green",                     // Material name
                    true,                        // cast shadows
                    Vec3(parameters[0]) * 0.01   // Scale, sphere.mesh has a radius of 50 units
                );
        std::shared_ptr<BulletActor> bulletActor = std::make_shared<BulletActor>(
                    name,
                    btSphere,                   // Create a sphere
                    parameters,                 // One parameter needed: the radius
                    pos,
                    5,                          // Mass: 5kg
                    Vec4(0,0,0,1),              // Orientation
                    0.5f,                       // Restitution
                    0.5f                        // Friction
                );
        std::shared_ptr<Actor> actor = std::make_shared<Actor>(name, bulletActor, ogreActor);
        actorMap[name] = actor;
    }
    // Create some boxes on the ground
    for(int i = 0; i < 21; i++)
    {
        const Vec3 pos((i - 5)/3.0f, 0.1, -3);
        std::stringstream ssName;
        ssName << "box_" << i;
        const std::string name = ssName.str();
        std::vector<float> parameters;
        parameters.push_back(0.1);
        parameters.push_back(0.2);
        parameters.push_back(0.3);

        std::shared_ptr<OgreActor> ogreActor = std::make_shared<OgreActor>(
                    name,                           // Name of the Ogre::SceneNode
                    pos,                            
                    Vec4(0,0,0,1),                  // Orientation, will be transformed into a (x,y,z,w) quaternion
                    "cube.mesh",                    // Name of the mesh, see assets folder
                    name,                           // Name of the Ogre::Entity
                    "Green",                        // Material name
                    true,                           // cast shadows
                    Vec3(parameters[0], parameters[1], parameters[2]) * 0.02   // Scale, cube.mesh has an edge length of 100 units
                );
        std::shared_ptr<BulletActor> bulletActor = std::make_shared<BulletActor>(
                    name,
                    btBox,                      // Create a box
                    parameters,                 // Three parameter needed: box length, width and height
                    pos,
                    1,                          // Mass: 1kg
                    Vec4(0,0,0,1),              // Orientation
                    0.5f,                       // Restitution
                    0.5f                        // Friction
                );
        std::shared_ptr<Actor> actor = std::make_shared<Actor>(name, bulletActor, ogreActor);
        actorMap[name] = actor;
    }

}

std::vector<std::shared_ptr<DiffActor>> eqOgreBullet::sync()
{
    std::vector<std::shared_ptr<DiffActor>> diffs;
    for(std::pair<std::string, std::shared_ptr<Actor> > p : actorMap)
    {
        std::shared_ptr<BulletActor> bulletActor = p.second->bulletActor();
        std::shared_ptr<OgreActor> ogreActor = p.second->ogreActor();
        std::string actorName = p.first;

        // Check if something changed, if so, we create a DiffActor for this actor.
        // This way, we don't have to send the complete scene graph in every frame, just the changes
        if (ogreActor->position() != bulletActor->position() ||
                ogreActor->orientation() != bulletActor->orientation())
        {

            std::shared_ptr<DiffActor> diff = std::make_shared<DiffActor>(
                        actorName,
                        bulletActor->position(),
                        bulletActor->orientation()
                        );

            diffs.push_back(diff);
        }

        ogreActor->setPosition(bulletActor->position());
        ogreActor->setOrientation(bulletActor->orientation());
    }

    return diffs;
}

void eqOgreBullet::spawnRandomly()
{
    static int serialNumber = 0;
    const Vec3 pos(random() % 10 - 5, random() % 5 + 10, random()%2 - 4);
    bool boxOrSphere = random() % 2 == 0;
    std::stringstream ssName;
    std::string meshName;
    std::vector<float> parameters;
    int mass = (random() % 5 + 1)*5;
    float restitution = (random() % 11) / 10.0f;
    std::vector<std::string> colors = {"Blue", "Red", "Yellow", "Green", "Magenta"};
    std::string materialName = colors[random() % colors.size()];
    CollisionShape shape;
    Vec3 scale;
    if(boxOrSphere){
        ssName << "random_" << "box_" << serialNumber++;
        parameters.push_back((random() % 3 + 1) / 10.f);
        parameters.push_back((random() % 3 + 1) / 10.f);
        parameters.push_back((random() % 3 + 1) / 10.f);
        scale = Vec3(parameters[0], parameters[1], parameters[2]) * 0.02;
        meshName = "cube.mesh";
        shape = btBox;
    }
    else{
        ssName << "random_" << "sphere_" << serialNumber++;
        parameters.push_back((random() % 3 + 1) / 10.0f);
        meshName = "sphere.mesh";
        scale = Vec3(parameters[0]) * 0.01;
        shape = btSphere;
    }
    const std::string name = ssName.str();

    std::shared_ptr<OgreActor> ogreActor = std::make_shared<OgreActor>(name,pos,Vec4(0,0,0,1),meshName,name,materialName,true,scale);
    std::shared_ptr<BulletActor> bulletActor = std::make_shared<BulletActor>(name,shape,parameters,pos,mass,Vec4(0,0,0,1),restitution,0.5f);
    std::shared_ptr<Actor> actor = std::make_shared<Actor>(name, bulletActor, ogreActor);
    actorMap[name] = actor;
    newOgreActors.push_back(ogreActor);
}

std::list<std::shared_ptr<BulletActor>> eqOgreBullet::getBulletActors()
{
    std::list<std::shared_ptr<BulletActor>> bulletActors;
    for(auto a : actorMap)
        bulletActors.push_back(a.second->bulletActor());
    return bulletActors;
}

bool eqOgreBullet::init(int argc, char *argv[])
{
    initActors();

    physics.init(getBulletActors());

    return initEq(argc, argv);
}

struct timespec start, finish;

bool eqOgreBullet::run()
{
    float elapsedTime = 0.0f;
    while (config->isRunning())
    {
        clock_gettime(CLOCK_MONOTONIC, &start);

        if(config->spawn){
            spawnRandomly();
            config->spawn = false;
        }

        if(!config->paused)
            physics.update(elapsedTime);

        // Bullet calculated new positions -> transfer them to our game state
        physics.sync(getBulletActors());

        // Physical and graphical views are out of sync -> Sync them and create DiffActors for the ones that changed
        std::vector<std::shared_ptr<DiffActor>> diffActors = sync();

        // Send new OgreActors and the changes to Ogre
        config->setGameState(newOgreActors, diffActors);

        config->startFrame();
        config->finishFrame();

        // Measure time for the next simulation step
        clock_gettime(CLOCK_MONOTONIC, &finish);
        elapsedTime = (finish.tv_sec - start.tv_sec);
        elapsedTime += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;

        lifeTime += elapsedTime;
        newOgreActors.clear();
    }
}

bool eqOgreBullet::exit()
{
    /* Exit configuration */
    config->exit();
    eq::exit();
}


int main(int argc, char *argv[])
{
    eqOgreBullet app;
    bool initSuccess = app.init(argc, argv);
    if(initSuccess)
        app.run();
   app.exit();

}
