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

#include <btBulletDynamicsCommon.h>


// CONSTANTS ////////////////////////////
const btScalar GRAVITY = -9.81;

const unsigned int MAX_STEPS = 100000;
const btScalar MIN_VEL = 0.0001;

const int MAX_SUB_STEPS = 2;
const btScalar FIXED_TIME_STEP = 1./120.;
const btScalar TIME_STEP = FIXED_TIME_STEP;

// ENUMS ////////////////////////////////
struct Orientation { enum Value { UNKNOWN, WIDTH, HEIGHT, DEPTH }; };

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
};


// FUNCTION PROTOTYPES ////////////////////
void runSimulation(const InputOptions & in);
btScalar random();
int processInputOptions(InputOptions & in, const int argc, char * argv[]);
Orientation::Value getOrientation(const btRigidBody & dice, const btVector3 & dimensions);
bool stableEq(const double x1, const double x2, const double tol = 1e-5);

int main(const int argc, char * argv[])
{
  InputOptions in;
  processInputOptions(in, argc, argv);

  if(!in.deterministic)
    srand(static_cast<unsigned int>(time(NULL)));
  random();

  runSimulation(in);

  return 0;
}

void runSimulation(const InputOptions & in)
{
  typedef ::boost::ptr_vector<btCollisionShape> CollisionShapes;

	// Initialization_start

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
  ScopedPtr<btDefaultCollisionConfiguration>::Type collisionConfiguration(new btDefaultCollisionConfiguration());

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  ScopedPtr<btCollisionDispatcher>::Type dispatcher(new	btCollisionDispatcher(collisionConfiguration.get()));

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
  ScopedPtr<btBroadphaseInterface>::Type overlappingPairCache(new btDbvtBroadphase());

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  ScopedPtr<btSequentialImpulseConstraintSolver>::Type solver(new btSequentialImpulseConstraintSolver);

  ScopedPtr<btDiscreteDynamicsWorld>::Type dynamicsWorld(
    new btDiscreteDynamicsWorld(
      dispatcher.get(),
      overlappingPairCache.get(),
      solver.get(),
      collisionConfiguration.get()
    )
  );

	dynamicsWorld->setGravity(btVector3(0,GRAVITY,0));

	// Initialization_end

	///create a few basic rigid bodies

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	CollisionShapes collisionShapes;

  {
    // create the ground
    btCollisionShape & groundShape = *collisionShapes.insert(
      collisionShapes.end(),
      new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)))
    );
    btScalar mass(0.);
    btVector3 localInertia(0.,0.,0.);

	  btTransform groundTransform;
	  groundTransform.setIdentity();
	  groundTransform.setOrigin(btVector3(0,-50,0));

    btRigidBody::btRigidBodyConstructionInfo rbInfo(
      mass,
      new btDefaultMotionState(groundTransform),
      &groundShape,
      localInertia
    );
    dynamicsWorld->addRigidBody(new btRigidBody(rbInfo));
  }


  btRigidBody * dice;
  btVector3 diceDimensions(btScalar(in.width), btScalar(in.height), btScalar(in.depth));
	{
		//create the dice
    btCollisionShape & diceShape = *collisionShapes.insert(
      collisionShapes.end(),
      new btBoxShape(diceDimensions)
    );

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.);

		btVector3 localInertia(0,0,0);
    diceShape.calculateLocalInertia(mass,localInertia);

    const btScalar initialY = ::std::max(::std::max(in.height, in.width), in.depth) + in.initialY;
    startTransform.setOrigin(btVector3(2, initialY, 0));
		
    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState * myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(
      mass,
      myMotionState,
      &diceShape,
      localInertia
    );
    dice = new btRigidBody(rbInfo);

    // set up the linear and angular velocities
    btVector3 vel(0.0, 0.0, 0.0);
    vel.setX(in.velocity == 0.0 ? random() : in.velocity);
    dice->setLinearVelocity(vel);

    vel.setValue(random(), random(), random());
    vel.normalize();
    vel *= in.angularVelocity == 0.0 ? random() : in.angularVelocity;
    dice->setAngularVelocity(vel);

		dynamicsWorld->addRigidBody(dice);
	}

  btTransform trans;
  btVector3 vel;
  btVector3 angVel;
  bool finished = false;
  for(unsigned int i = 0; i < MAX_STEPS && !finished; i++)
  {
    dynamicsWorld->stepSimulation(TIME_STEP, MAX_SUB_STEPS, FIXED_TIME_STEP);

		//print positions of the dice
		dice->getMotionState()->getWorldTransform(trans);

    ::std::cout << i << " " << ::std::setprecision(12) <<
      static_cast<double>(trans.getOrigin().getX()) << " " <<
      static_cast<double>(trans.getOrigin().getY()) << " " <<
      static_cast<double>(trans.getOrigin().getZ()) << ::std::endl;

    vel = dice->getLinearVelocity();
    angVel = dice->getAngularVelocity();
    if(vel.length() < MIN_VEL && angVel.length() < MIN_VEL)
      finished = true;
  }

  ::std::cout << getOrientation(*dice, diceDimensions);

	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	while(dynamicsWorld->getNumCollisionObjects() != 0)
	{
    const int back = dynamicsWorld->getNumCollisionObjects() - 1;
		btCollisionObject * const obj = dynamicsWorld->getCollisionObjectArray()[back];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
			delete body->getMotionState();

		dynamicsWorld->removeCollisionObject(obj);
    delete obj;
	}

	///-----cleanup_end-----
}

btScalar random()
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
      ("width,w", po::value<btScalar>(&in.width)->default_value(1.0), "dice width")
      ("height,h", po::value<btScalar>(&in.height)->default_value(1.0), "dice height")
      ("depth,d", po::value<btScalar>(&in.depth)->default_value(1.0), "dice depth")
      ("initial-y,y", po::value<btScalar>(&in.initialY)->default_value(2.0), "initial y-coordinate (maximum dimension of dice will be added)")
    ;

    po::positional_options_description p;
    p.add("input", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    // Deal with help first, otherwise missing required parameters will cause exception
    if(vm.count("help"))
    {
      ::std::cout << desc << ::std::endl;
      return 0;
    }

    po::notify(vm);
  }
  catch(std::exception& e)
  {
    ::std::cerr << e.what() << ::std::endl;
    return 1;
  }

  return 0;
}

Orientation::Value getOrientation(const btRigidBody & dice, const btVector3 & dimensions)
{
  const btVector3 pos = dice.getWorldTransform().getOrigin();
  if(stableEq(pos.getY(), dimensions.getX(), 1e-2))
    return Orientation::WIDTH;
  else if(stableEq(pos.getY(), dimensions.getY(), 1e-2))
    return Orientation::HEIGHT;
  else if(stableEq(pos.getY(), dimensions.getZ(), 1e-2))
    return Orientation::DEPTH;
  else
    return Orientation::UNKNOWN;
}

bool stableEq(const double x1, const double x2, const double tol)
{
  if(x1 < (x2 + tol) && x1 > x2 - tol)
    return true;
  else
    return false;
}

::std::ostream & operator <<(const Orientation::Value & orient, ::std::ostream & os)
{
  if(orient == Orientation::WIDTH)
    os << "WIDTH";
  else if(orient == Orientation::HEIGHT)
    os << "HEIGHT";
  else if(orient == Orientation::DEPTH)
    os << "DEPTH";
  else
    os << "UNKNOWN";
  return os;
}
