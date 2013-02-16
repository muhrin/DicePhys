/*
 * DicePhys.h
 *
 *  Created on: Feb 3, 2013
 *      Author: Martin Uhrin
 */

#ifndef DICEPHYS_H
#define DICEPHYS_H

// INCLUDES //////////////////////////////////

#include <boost/version.hpp>

#define BT_USE_DOUBLE_PRECISION
// Bullet physics library
#include <btBulletDynamicsCommon.h>


// DEFINES ///////////////////
#ifndef NULL
#  define NULL 0
#endif

#if (BOOST_VERSION / 100000) <= 1 && ((BOOST_VERSION / 100) % 1000) <= 41
#  define DICEPHYS_BOOST_PO_OLD
#endif

// CONSTANTS ///////////////////
extern const btVector3 X_VECTOR;
extern const btVector3 Y_VECTOR;
extern const btVector3 Z_VECTOR;

// FUNCTIONS //////////////////
bool stableEq(const double x1, const double x2, const double tol = 1e-5);

#endif /* DICEPHYS_H */
