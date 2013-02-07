/*
 * WorldObject.h
 *
 *
 *  Created on: Feb 2, 2013
 *      Author: Martin Uhrin
 */

#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

// INCLUDES /////////////////////////////////////////////
#include "DicePhys.h"

#include <ostream>

#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>

#include <btBulletDynamicsCommon.h>

// FORWARD DECLARATIONS ////////////////////////////////////


namespace dicephys {

class World;

class WorldObject
{
public:
  typedef ::boost::optional<btVector3> OptionalVec3;

  WorldObject(btCollisionShape * const shape, const btScalar mass);
  ~WorldObject();

  void setInitialPos(const btVector3 & pos);
  void setInitialVel(const btVector3 & vel);
  void setInitialAngularVel(const btVector3 & angVel);
  void setFriction(const btScalar friction);
  void setRestitution(const btScalar restitution);

  void worldInserted(World & world);
  void worldRemoving(World & world);

  OptionalVec3 getPosition() const;
  OptionalVec3 getFacing(const btVector3 & relativeTo = Y_VECTOR) const;
  OptionalVec3 getLinearVelocity() const;
  OptionalVec3 getAngularVelocity() const;

  ::std::ostream & printPosition(::std::ostream & os) const;
  ::std::ostream & printFacingVector(::std::ostream & os) const;

protected:
  typedef ::boost::scoped_ptr<btCollisionShape> ShapePtr;
  typedef ::boost::scoped_ptr<btRigidBody> RigidBodyPtr;

  virtual void inserting() {}
  virtual void removing() {}

  World * myWorld;

  RigidBodyPtr myRigidBody;

private:

  void createRigidBody();
  void cleanUpRigidBody();

  btRigidBody::btRigidBodyConstructionInfo myRBConstructionInfo;
  btVector3 myInitialVel;
  btVector3 myInitialAngVel;
};

}

#endif /* WORLD_OBJECT_H */
