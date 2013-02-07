/*
 * DicePhys.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: Martin Uhrin
 */

// INCLUDES //////////////////////////////////
#include "DicePhys.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <time.h>

#include <boost/program_options.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>

// Local includes
#include <World.h>
#include <WorldObject.h>
#include <Dice.h>

// TYPEDEFS ////////////////////////////
typedef ::std::auto_ptr<dicephys::WorldObject> WorldObjectPtr;

// CONSTANTS ////////////////////////////
const int RESULT_SUCCESS = 0;

const unsigned int MAX_STEPS = 100000;
const btScalar MIN_VEL(0.0001);

const int MAX_SUB_STEPS = 2;
const btScalar FIXED_TIME_STEP = 1./120.;
const btScalar TIME_STEP = FIXED_TIME_STEP;

const btVector3 X_VECTOR(static_cast<btScalar>(1.0), static_cast<btScalar>(0.0), static_cast<btScalar>(0.0));
const btVector3 Y_VECTOR(static_cast<btScalar>(0.0), static_cast<btScalar>(1.0), static_cast<btScalar>(0.0));
const btVector3 Z_VECTOR(static_cast<btScalar>(0.0), static_cast<btScalar>(0.0), static_cast<btScalar>(1.0));

// ENUMS ////////////////////////////////

// CLASSES ///////////////////////////////
template <typename T>
struct ScopedPtr
{
  typedef ::boost::scoped_ptr<T> Type;
};

struct InputOptions
{
  btScalar velocity;
  btScalar angularVelocity;
  bool deterministic;
  btScalar width;
  btScalar height;
  btScalar depth;
  btScalar initialY;
  btScalar mass;
  btScalar diceFriction;
  btScalar diceRestitution;
  btScalar groundFriction;
  btScalar groundRestitution;
  unsigned int numRolls;
};


// FUNCTION PROTOTYPES ////////////////////
void runSimulation(const InputOptions & in);
btScalar myRandom();
int processInputOptions(InputOptions & in, const int argc, char * argv[]);

WorldObjectPtr createGround(const InputOptions & in);
::std::auto_ptr<dicephys::Dice> createDice(const InputOptions & in);
void printResults(const unsigned int (& results)[4]);

int main(const int argc, char * argv[])
{
  InputOptions in;
  int result = processInputOptions(in, argc, argv);
  if(result != RESULT_SUCCESS)
    return result;

  if(!in.deterministic)
    srand(static_cast<unsigned int>(time(NULL)));

  // Mamke sure the random number generator has been called at least once
  myRandom();

  runSimulation(in);

  return 0;
}

void runSimulation(const InputOptions & in)
{
  dicephys::World world;
  world.insert(createGround(in));

  unsigned int results[4];
  memset(results, 0, sizeof(unsigned int) * 4);

  btVector3 pos;
  btVector3 vel;
  btVector3 angVel;
  for(unsigned int i = 0; i < in.numRolls; ++i)
  {

    // Insert objects into the world
    const dicephys::Dice & dice = world.insert(createDice(in));

    if(in.numRolls == 1)
    {
      // Print initial position
      ::std::cout << 0 << " ";
      dice.printFacingVector(::std::cout) << ::std::endl;
    }

    bool finished = false;
    for(unsigned int i = 1; i < MAX_STEPS && !finished; i++)
    {
      world.step();

      if(in.numRolls == 1)
      {
        ::std::cout << i << " ";
        dice.printFacingVector(::std::cout) << ::std::endl;
      }

      vel = *dice.getLinearVelocity();
      angVel = *dice.getAngularVelocity();
      if(vel.length() < MIN_VEL && angVel.length() < MIN_VEL)
        finished = true;
    }

    ++results[dice.getUpFace()];

    world.remove(dice);
  }

  printResults(results);
}

btScalar myRandom()
{
  return static_cast<btScalar>(::std::rand()) / static_cast<btScalar>(RAND_MAX);
}

int processInputOptions(InputOptions & in, const int argc, char * argv[])
{
  namespace po = ::boost::program_options;

  const ::std::string exeName(argv[0]);

  try
  {
    po::options_description desc("Usage: " + exeName + " [options] files...\n" +
      "\nOptions");
    desc.add_options()
      ("help", "Show help message")
      ("velocity,v", po::value<btScalar>(&in.velocity)->default_value(0.0), "velocity (0 = random)")
      ("angular-velocity,a", po::value<btScalar>(&in.angularVelocity)->default_value(0.0), "angular velocity (0 = random)")
      ("deterministic,D", po::value<bool>(&in.deterministic)->default_value(false)->zero_tokens(), "deterministic mode (no randomness)")
      ("mass,m", po::value<btScalar>(&in.mass)->default_value(1.0), "dice mass")
      ("width,w", po::value<btScalar>(&in.width)->default_value(1.0), "dice width")
      ("height,h", po::value<btScalar>(&in.height)->default_value(1.0), "dice height")
      ("depth,d", po::value<btScalar>(&in.depth)->default_value(1.0), "dice depth")
      ("initial-y,y", po::value<btScalar>(&in.initialY)->default_value(2.0), "initial y-coordinate (maximum dimension of dice will be added)")
      ("dice-friction,f", po::value<btScalar>(&in.diceFriction)->default_value(0.5), "dice friction")
      ("dice-restitution,r", po::value<btScalar>(&in.diceRestitution)->default_value(0.0), "dice restitution")
      ("ground-friction,F", po::value<btScalar>(&in.groundFriction)->default_value(0.5), "ground friction")
      ("ground-restitution,R", po::value<btScalar>(&in.groundRestitution)->default_value(0.0), "ground restitution")
      ("num-rolls,n", po::value<unsigned int>(&in.numRolls)->default_value(1), "number of rolls")
    ;

    po::positional_options_description p;
    p.add("input", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 1;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cerr << e.what() << ::std::endl;
    return 1;
  }

  return RESULT_SUCCESS;
}

bool stableEq(const double x1, const double x2, const double tol)
{
  if(x1 < (x2 + tol) && x1 > x2 - tol)
    return true;
  else
    return false;
}


WorldObjectPtr createGround(const InputOptions & in)
{
  // create the ground
  WorldObjectPtr ground(new dicephys::WorldObject(
    new btBoxShape(btVector3(btScalar(5000.),btScalar(50.),btScalar(5000.))), 0.0)
  );
  
  // Set physical properties
  ground->setFriction(in.groundFriction);
  ground->setRestitution(in.groundRestitution);

  ground->setInitialPos(btVector3(0.0, -50.0, 0.0));

  return ground;
}

::std::auto_ptr<dicephys::Dice> createDice(const InputOptions & in)
{
  ::std::auto_ptr<dicephys::Dice> dice;
  if(in.depth == 0.0) // 2D
    dice.reset(new dicephys::Dice(in.width, in.height, in.mass)); 
  else // 3D
    dice.reset(new dicephys::Dice(in.width, in.height, in.depth, in.mass));

  const btScalar initialY = ::std::max(::std::max(in.height, in.width), in.depth) + in.initialY;
  dice->setInitialPos(btVector3(2, initialY, 0));

  // Set the physical properties
  dice->setFriction(in.diceFriction);
  dice->setRestitution(in.diceRestitution);

  // set up the linear and angular velocities
  btVector3 vel(0.0, 0.0, 0.0);
  vel.setX(in.velocity == 0.0 ? myRandom() : in.velocity);
  dice->setInitialVel(vel);

  if(dice->is2D())
    vel.setValue(0.0, 0.0, myRandom());
  else
    vel.setValue(myRandom(), myRandom(), myRandom());
  
  vel.normalize();
  vel *= in.angularVelocity == 0.0 ? myRandom() : in.angularVelocity;
  dice->setInitialAngularVel(vel);

  return dice;
}


void printResults(const unsigned int (& results)[4])
{
  using namespace dicephys;

  ::std::cout << dicephys::Dice::Orientation::UNKNOWN << "\t" << results[Dice::Orientation::UNKNOWN] << ::std::endl;
  ::std::cout << Dice::Orientation::HEIGHT << "\t" << results[Dice::Orientation::HEIGHT] << ::std::endl;
  ::std::cout << Dice::Orientation::WIDTH << "\t" << results[Dice::Orientation::WIDTH] << ::std::endl;
  ::std::cout << Dice::Orientation::DEPTH << "\t" << results[Dice::Orientation::DEPTH] << ::std::endl;
}
