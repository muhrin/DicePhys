/*
 * Dice.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "Dice.h"

#include <cmath>

// NAMESPACES ////////////////////////////////


namespace dicephys {

const btScalar ONE_HALF(0.5);

Dice::Dice(const btScalar width, const btScalar height, const btScalar mass):
WorldObject(new btBoxShape(
  btVector3(ONE_HALF * width, ONE_HALF * height, ONE_HALF)), mass
),
myIs2D(true)
{}

Dice::Dice(const btScalar width, const btScalar height, const btScalar depth, const btScalar mass):
WorldObject(new btBoxShape(btVector3(ONE_HALF * width, ONE_HALF * height, ONE_HALF * depth)), mass),
myIs2D(false)
{}

void Dice::inserting()
{
  if(myIs2D && myRigidBody.get())
  {
    // Constrain to X-Y plan
    myRigidBody->setLinearFactor(btVector3(1., 1., 0.));
    myRigidBody->setAngularFactor(btVector3(0., 0., 1.));
  }
}

bool Dice::is2D() const
{
  return myIs2D;
}

Dice::Orientation::Value Dice::getUpFace() const
{
  if(!myRigidBody.get())
    return Orientation::UNKNOWN;

  btTransform trans;
  myRigidBody->getMotionState()->getWorldTransform(trans);
  const btMatrix3x3 rot = trans.getBasis();

  if(isFacingDirection(Y_VECTOR, X_VECTOR, rot))
    return Orientation::WIDTH;
  else if(isFacingDirection(Y_VECTOR, Y_VECTOR, rot))
    return Orientation::HEIGHT;
  else if(isFacingDirection(Y_VECTOR, Z_VECTOR, rot))
    return Orientation::DEPTH;

  return Orientation::UNKNOWN;
}

bool Dice::isFacingDirection(const btVector3 & facing_, const btVector3 & direction, const btMatrix3x3 & rot) const
{
  btVector3 currentlyFacing(rot * direction);
  currentlyFacing.normalize();

  const btVector3 facing = facing_.normalized();
  const btScalar dot = ::std::abs(facing.dot(currentlyFacing));
  if(stableEq(dot, 1.0))
    return true;

  return false;
}

}

::std::ostream & operator <<(::std::ostream & os, const dicephys::Dice::Orientation::Value & orient)
{
  if(orient == dicephys::Dice::Orientation::WIDTH)
    os << "WIDTH";
  else if(orient == dicephys::Dice::Orientation::HEIGHT)
    os << "HEIGHT";
  else if(orient == dicephys::Dice::Orientation::DEPTH)
    os << "DEPTH";
  else
    os << "UNKNOWN";
  return os;
}


