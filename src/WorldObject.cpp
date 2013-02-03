/*
 * WorldObject.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "WorldObject.h"

#include <iomanip>
#include <iostream>

#include "World.h"

// NAMESPACES ////////////////////////////////


namespace dicephys {

WorldObject::WorldObject(btCollisionShape * const shape, const btScalar mass):
myRBConstructionInfo(mass, new btDefaultMotionState(), shape)
{
  myRBConstructionInfo.m_motionState = new btDefaultMotionState();
}

WorldObject::~WorldObject()
{
  cleanUpRigidBody();
}

void WorldObject::setInitialPos(const btVector3 &pos)
{
  btTransform trans;
  myRBConstructionInfo.m_motionState->getWorldTransform(trans);
  trans.setOrigin(pos);
  myRBConstructionInfo.m_motionState->setWorldTransform(trans);
}

void WorldObject::setInitialVel(const btVector3 & vel)
{
  myInitialVel = vel;
}

void WorldObject::setInitialAngularVel(const btVector3 & angVel)
{
  myInitialAngVel = angVel;
}

void WorldObject::setFriction(const btScalar friction)
{
  myRBConstructionInfo.m_friction = friction;
}


void WorldObject::setRestitution(const btScalar restitution)
{
  myRBConstructionInfo.m_restitution = restitution;
}

void WorldObject::worldInserted(World &world)
{
  myWorld = &world;

  createRigidBody();
  inserting();
  myWorld->getDynamicsWorld().addRigidBody(myRigidBody.get());
}

WorldObject::OptionalVec3 WorldObject::getPosition() const
{
  OptionalVec3 pos;
  if(myRigidBody.get())
  {
    btTransform trans;
    myRigidBody->getMotionState()->getWorldTransform(trans);
    pos.reset(trans.getOrigin());
  }
  
  return pos;
}

WorldObject::OptionalVec3 WorldObject::getFacing(const btVector3 & relativeTo) const
{
  OptionalVec3 facing;
  if(myRigidBody.get())
  {
    btTransform trans;
    myRigidBody->getMotionState()->getWorldTransform(trans);
    facing.reset(trans.getBasis() * relativeTo);
  }
  
  return facing;
}

WorldObject::OptionalVec3 WorldObject::getLinearVelocity() const
{
  OptionalVec3 vel;
  if(myRigidBody.get())
    vel.reset(myRigidBody->getLinearVelocity());
  
  return vel;
}

WorldObject::OptionalVec3 WorldObject::getAngularVelocity() const
{
  OptionalVec3 angVel;
  if(myRigidBody.get())
    angVel.reset(myRigidBody->getAngularVelocity());
  
  return angVel;
}

::std::ostream & WorldObject::printPosition(::std::ostream & os) const
{
  const OptionalVec3 pos = getPosition();
  if(pos)
  {
    ::std::cout << ::std::setprecision(12) <<
      pos->getX() << " " << pos->getY() << " " << pos->getZ();
  }
  return os;
}

::std::ostream & WorldObject::printFacingVector(::std::ostream & os) const
{
  const OptionalVec3 pos = getPosition();
  const OptionalVec3 facing = getFacing();
  if(pos && facing)
  {
    ::std::cout << ::std::setprecision(12) <<
      pos->getX() << " " << pos->getY() << " " << pos->getZ() << " " <<
      facing->getX() << " " << facing->getY() << " " << facing->getZ();
  }
  return os;
}

void WorldObject::worldRemoving(World & world)
{
  cleanUpRigidBody();
}

void WorldObject::createRigidBody()
{
  cleanUpRigidBody();

  myRBConstructionInfo.m_localInertia.setZero();
  if(myRBConstructionInfo.m_mass != 0.)
  {
    myRBConstructionInfo.m_collisionShape->calculateLocalInertia(
      myRBConstructionInfo.m_mass,
      myRBConstructionInfo.m_localInertia
    );
  }

  myRigidBody.reset(new btRigidBody(myRBConstructionInfo));

  myRigidBody->setLinearVelocity(myInitialVel);
  myRigidBody->setAngularVelocity(myInitialAngVel);
}

void WorldObject::cleanUpRigidBody()
{
  if(myRigidBody.get())
  {
    if(myWorld)
      myWorld->getDynamicsWorld().removeCollisionObject(myRigidBody.get());

    myRigidBody.reset();
  }
}

}
