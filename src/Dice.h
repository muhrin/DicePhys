/*
 * Dice.h
 *
 *
 *  Created on: Feb 2, 2013
 *      Author: Martin Uhrin
 */

#ifndef DICE_H
#define DICE_H

// INCLUDES /////////////////////////////////////////////
#include "WorldObject.h"

#include <ostream>

// FORWARD DECLARATIONS ////////////////////////////////////


namespace dicephys {

class Dice : public WorldObject
{
public:

  struct Orientation { enum Value { UNKNOWN, WIDTH, HEIGHT, DEPTH }; };

  // Create 2D dice
  Dice(
    const btScalar width,
    const btScalar height,
    const btScalar mass
  );
  // Create 3D dice
  Dice(
    const btScalar width,
    const btScalar height,
    const btScalar depth,
    const btScalar mass
  );

  bool is2D() const;
  Orientation::Value getUpFace() const;

private:
  virtual void inserting();

  bool isFacingDirection(const btVector3 & facing, const btVector3 & direction, const btMatrix3x3 & trans) const;

  const bool myIs2D;
};

}

inline ::std::ostream & operator <<(::std::ostream & os, const dicephys::Dice::Orientation::Value & orient)
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

#endif /* DICE_H */
