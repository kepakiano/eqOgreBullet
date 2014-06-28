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


#include "../headers/bulletphysics.h"

#include <iostream>
#include <sstream>

#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include "../headers/bulletrigidbody.h"
#include "datastructures/headers/bulletactor.h"
#include "datastructures/headers/vec3.h"
#include "datastructures/headers/vec4.h"


btVector3 toBtVector3(const Vec3 & vec)
{
    return btVector3(vec.x(), vec.y(), vec.z());
}

btQuaternion toBtQuaternion(const Vec4 & vec)
{
    return btQuaternion(vec.x(),vec.y(),vec.z(), vec.w());
}

Vec3 toVec3(const btVector3 & vec)
{
    return Vec3(vec.x(), vec.y(), vec.z());
}

Vec4 toVec4(const btQuaternion & quat)
{
    return Vec4(quat.getX(),quat.getY(), quat.getZ(), quat.getW());
}

BulletPhysics::BulletPhysics()
{
}


BulletPhysics::~BulletPhysics()
{
    // delete Constraints before deleting Rigidbodies --> or else the rigidbodies won't be deleted correctly
    while(_dynamicsWorld->getNumConstraints()){
        btTypedConstraint * constraint = _dynamicsWorld->getConstraint(0);
        if(constraint != nullptr)
            _dynamicsWorld->removeConstraint(constraint);
    }

    //remove the rigidbodies from the dynamics world and delete them
    for (int i=_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = _dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);

        std::string name;
        for(auto p : _rigidBodies){
            if(p.second->rigidBody() == body){
                name = p.first;
                break;
            }
        }
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        _dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }


    //delete collision shapes
    for (int j=0;j<_collisionShapes->size();j++)
    {
        btCollisionShape* shape = _collisionShapes->at(j);
        (*_collisionShapes)[j] = 0;
        delete shape;
    }

    _rigidBodies.clear();
    _collisionShapes->clear();

    delete _dynamicsWorld;
    delete _solver;
    delete _dispatcher;
    delete _collisionConfiguration;
    delete _broadphase;

    delete _collisionShapes;
}

bool BulletPhysics::init(std::list< std::shared_ptr< BulletActor > > actors)
{

    // For RigidBodies
    _collisionConfiguration = new btDefaultCollisionConfiguration();

    _dispatcher = new	btCollisionDispatcher(_collisionConfiguration);
    _broadphase = new btDbvtBroadphase();

    // For RigidBodies
    _solver = new btSequentialImpulseConstraintSolver;

    _gravity = btVector3(0,-9.81f,0);

    _dispatcher
            ->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,
                                          BOX_SHAPE_PROXYTYPE,
                                          _collisionConfiguration
                                          ->getCollisionAlgorithmCreateFunc(
                                              CONVEX_HULL_SHAPE_PROXYTYPE,
                                              CONVEX_HULL_SHAPE_PROXYTYPE
                                              )
                                          );


    // For RigidBodies
    _dynamicsWorld =
            new btDiscreteDynamicsWorld(_dispatcher
                                         , _broadphase
                                         , _solver
                                         , _collisionConfiguration);


    _dynamicsWorld->setGravity(_gravity);
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

    _collisionShapes = new btAlignedObjectArray<btCollisionShape*>();
    _collisionShapes->push_back(groundShape);

    btDefaultMotionState* groundMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
    std::string groundName("ground");
    std::shared_ptr<BulletRigidBody> groundRigidBody = std::make_shared<BulletRigidBody>(groundRigidBodyCI, groundName, false);
    _dynamicsWorld->addRigidBody(groundRigidBody->rigidBody());
    _rigidBodies[groundName] = groundRigidBody;


    groundRigidBody->rigidBody()->setRestitution(0.5);
    groundRigidBody->rigidBody()->setFriction(0.5);

    for(std::shared_ptr<BulletActor> actor : actors){
        addActor(actor);
    }

    return true;
}


void BulletPhysics::addActor(std::shared_ptr<BulletActor> actor)
{
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(toBtVector3(actor->position()));
    transform.setRotation(toBtQuaternion(actor->orientation()));
    btDefaultMotionState* motionState = new btDefaultMotionState(transform);

    std::vector<float> parameters = actor->parameters();

    btCollisionShape* shape;

    switch (actor->shape())
    {
    case btSphere:
        assert(parameters.size() == 1); // Spheres only need a radius
        shape = new btSphereShape(parameters[0]);
        break;
    case btCapsule:
        assert(parameters.size() == 2); // Capsules need two radii
        shape = new btCapsuleShape(parameters[0],parameters[1]);
        break;
    case btBox:
        assert(parameters.size() == 3); // Boxes need three edge lengths
        shape = new btBoxShape(btVector3(parameters[0], parameters[1], parameters[2]));
        break;
    case btCylinder:
        assert(parameters.size() == 3); // Cylinders need three edge lengths (radius is derived from them)
        shape = new btCylinderShape(btVector3(parameters[0],parameters[1],parameters[2]));
        break;

    default:
        shape = new btSphereShape(1.0f);
        break;
    }

    _collisionShapes->push_back(shape);
    btScalar mass = actor->mass();
    btVector3 inertia(0,0,0);
    shape->calculateLocalInertia(mass,inertia);

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,motionState,shape,inertia);

    std::shared_ptr<BulletRigidBody> rigidBody =
            std::make_shared<BulletRigidBody>( rigidBodyCI, actor->name(), true);

    _dynamicsWorld->addRigidBody(rigidBody->rigidBody());
    _rigidBodies[rigidBody->name()] = rigidBody;
    rigidBody->rigidBody()->setRestitution(actor->restitution());
    rigidBody->rigidBody()->setFriction(actor->friction());
}


void BulletPhysics::update(float elapsedTime)
{
    _dynamicsWorld->stepSimulation(elapsedTime, 10, btScalar(1.)/btScalar(240.));
}


void BulletPhysics::sync(std::list<std::shared_ptr<BulletActor>> actors)
{
    for(std::shared_ptr<BulletActor> actor : actors)
    {
        std::string name = actor->name();

        // If the actor does not exist, we need to add it to the world
        if (_rigidBodies.count(name) == 0)
        {
            addActor(actor);
        }
        btRigidBody * rigidBody = _rigidBodies[name]->rigidBody();
        if (rigidBody && rigidBody->getMotionState())
        {
            btTransform trans;
            rigidBody->getMotionState()->getWorldTransform(trans);

            btVector3 position = trans.getOrigin();
            btQuaternion quaternion = trans.getRotation();

            actor->setPosition(toVec3(position));
            actor->setRotation(toVec4(quaternion));
            actor->setRestitution(rigidBody->getRestitution());
            actor->setFriction(rigidBody->getFriction());
        }
    }
}
