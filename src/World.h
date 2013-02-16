/*
 * World.h
 *
 *
 *  Created on: Feb 2, 2013
 *      Author: Martin Uhrin
 */

#ifndef WORLD_H
#define WORLD_H

// INCLUDES /////////////////////////////////////////////

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>

#include "DicePhys.h"

namespace dicephys {

// FORWARD DECLARATIONS ////////////////////////////////////
class WorldObject;

class World
{
  typedef ::std::auto_ptr<WorldObject> WorldObjectPtr;
  typedef ::boost::ptr_vector<WorldObject> Objects;

  static const btScalar GRAVITY;
public:

  typedef Objects::iterator iterator;

  World(const btScalar stepsize = static_cast<btScalar>(1.0/120.0));
  ~World();

  btDynamicsWorld & getDynamicsWorld();

  template <class ObjectType>
  ObjectType & insert(::std::auto_ptr<ObjectType> worldObject);
  void remove(iterator pos);
  template <class ObjectType>
  bool remove(ObjectType & obj);

  void step();

private:

  ::boost::scoped_ptr<btCollisionConfiguration> myCollisionConfiguration;
  ::boost::scoped_ptr<btCollisionDispatcher> myCollisionDispatcher;
  ::boost::scoped_ptr<btBroadphaseInterface> myBroadphaseInterface;
  ::boost::scoped_ptr<btConstraintSolver> myConstraintSolver;
  ::boost::scoped_ptr<btDynamicsWorld> myDynamicsWorld;

  Objects myObjects;

  const btScalar myStepsize;
};

template <class ObjectType>
ObjectType & World::insert(::std::auto_ptr<ObjectType> worldObject)
{
  ObjectType * const obj = worldObject.get();
  iterator it = myObjects.insert(myObjects.end(), worldObject);
  obj->worldInserted(*this);
  return *obj;
}

template <class ObjectType>
bool World::remove(ObjectType & obj)
{
  iterator it = myObjects.begin();
  for(iterator end = myObjects.end(); it != end; ++it)
  {
    if(&(*it) == &obj)
      break;
  }
  if(it == myObjects.end())
    return false;

  remove(it);
  return true;
}

}

#endif /* WORLD_H */
