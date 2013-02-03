/*
 * WorldObject.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "World.h"

#include <boost/foreach.hpp>

#include "WorldObject.h"

// NAMESPACES ////////////////////////////////


namespace dicephys {

const btScalar World::GRAVITY = -9.81;

World::World(const btScalar stepsize):
myStepsize(stepsize)
{
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
  myCollisionConfiguration.reset(new btDefaultCollisionConfiguration());

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  myCollisionDispatcher.reset(new	btCollisionDispatcher(myCollisionConfiguration.get()));

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
  myBroadphaseInterface.reset(new btDbvtBroadphase());

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  myConstraintSolver.reset(new btSequentialImpulseConstraintSolver);

  myDynamicsWorld.reset(
    new btDiscreteDynamicsWorld(
      myCollisionDispatcher.get(),
      myBroadphaseInterface.get(),
      myConstraintSolver.get(),
      myCollisionConfiguration.get()
    )
  );

	myDynamicsWorld->setGravity(btVector3(0,GRAVITY,0));
}

World::~World()
{
  BOOST_FOREACH(WorldObject & object, myObjects)
  {
    object.worldRemoving(*this);
  }
}

btDynamicsWorld & World::getDynamicsWorld()
{
  return *myDynamicsWorld;
}

void World::remove(iterator pos)
{
  if(pos == myObjects.end())
    return;

  pos->worldRemoving(*this);
  myObjects.erase(pos);
}

void World::step()
{
  myDynamicsWorld->stepSimulation(myStepsize, 1, myStepsize);
}

}
