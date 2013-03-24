/*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"
#include <iostream>
#include <time.h>


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{
	enum
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

public:
	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
		
		// Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));
		m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}

	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}
};


static RagdollDemo* ragdollDemo;

bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
	int *ID1, *ID2;
	btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
	btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
	int groundID = 9;
	ID1 = static_cast<int*>(o1->getUserPointer());
	ID2 = static_cast<int*>(o2->getUserPointer());
	
	ragdollDemo->touches[*ID1] = 1;
	ragdollDemo->touches[*ID2] = 1;
	ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
	ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;

	//printf("ID1 = %d, ID2 = %d\n", *ID1, *ID2);
	//printf("ID1 = %d, ID2 = %d\n", *ID1, *ID2);
	return false;
}

void RagdollDemo::initPhysics()
{
	srand(time(NULL));
	//set each value in weights to a random float value in the range of -1 to 1 (ASSIGNMENT 9)
	for(int i=0; i<8; i++){
		for(int j=0; j<4; j++){
			weights[j][i] = RandFloat(-1,1);
			
		}
	}

	minFPS = 1000000.f/60.f;
	// Setup the basic world
	gContactProcessedCallback = myContactProcessedCallback;

	ragdollDemo = this;

	IDs[9] = 9;
	for(int i = 0; i<9;i++){
		IDs[i] = i;
	}

	ragdollDemo->touches = new int[10];
	for(int i = 0; i<=10;i++){
		touches[i] = 0;
		//printf("Touch%d = %d\n", i, touch[i]);
	}

	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(5.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		m_dynamicsWorld->addCollisionObject(fixedGround);
		fixedGround->setUserPointer(&IDs[9]);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}
	counter = 0;

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	//spawnRagdoll(startOffset);
	startOffset.setValue(-1,0.5,0);
	//spawnRagdoll(startOffset);

	CreateRobot();
	//Robot robot = Robot(this);


	clientResetScene();		
}

void RagdollDemo::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, startOffset);
	m_ragdolls.push_back(ragDoll);
}	

void RagdollDemo::clientMoveAndDisplay()
{
	//Set all touch sensors to 0
	for(int i = 0; i <10;i++){
		touches[i] = 0;
	}
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	//float minFPS = 100000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;
	
		if (m_dynamicsWorld)
		{
			if(!pause){
				m_dynamicsWorld->stepSimulation(ms / minFPS);
			}else{
				if(oneStep){
					m_dynamicsWorld->stepSimulation((ms / minFPS)+1);
					oneStep = false;
				}
			}
			//optional but useful: debug drawing
			m_dynamicsWorld->debugDrawWorld();
		}


		//ASSIGNMENT 9 Stuff below
		if (!pause || (pause && oneStep)) {
			m_dynamicsWorld->stepSimulation(ms / 1000000.f );
			
			
			if(counter==50){

				for(int i=0; i<8; i++){

					double motorCommand = 0.0;

					for(int j=0; j<4; j++){

						//Did j+5 because my 4 feet touch sensors start at the 5th index. May change depending on your configuration
						motorCommand = motorCommand + touches[(j+5)]*weights[j][i];
						//std::cout << touches[(j+5)] << std::endl;
					}
					
					//Keep motorcommand between -1 and 1
					motorCommand = tanh(motorCommand);

					//Expand it to be between -45 and 45
					motorCommand = motorCommand*M_PI_4;

					//motorCommand = 0;
					std::cout << motorCommand << std::endl;

					ActuateJoint(i, motorCommand, 0, ms / minFPS);
					//m_dynamicsWorld->stepSimulation(ms / 1000000.f );
				}

				//printf("%d%d%d%d\n",touches[5],touches[6],touches[7],touches[8]);
				counter = 0;
			}
		oneStep = !oneStep;
		counter++;
			
			
	}

		

		
	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		btVector3 startOffset(0,2,0);
		spawnRagdoll(startOffset);
		break;
		}
	case 'p':
		{
			if(!pause){
				pause = true;
			}else{
				pause = false;
			}
			break;
		}
	case 'k':
		{
			oneStep = !oneStep;
			break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}


void	RagdollDemo::exitPhysics()
{

	int i;

	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}

/*
	ASSIGNMENTS
*/


void RagdollDemo::CreateBox(int index, double x, double y, double z, double length, double width, double height){
	geom[index] = new btBoxShape(btVector3(btScalar(length),btScalar(width),btScalar(height))); 
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z))); 
	body[index] = localCreateRigidBody(btScalar(1.0),offset,geom[index]); 

	body[index]->setUserPointer(&IDs[index]);
}

void RagdollDemo::CreateCylinder(int index,double x, double y, double z,double radius, double length, double eulerX, double eulerY, double eulerZ){
	
	//geom[index] = new btCylinderShape(btVector3(btScalar(radius),btScalar(length),btScalar(0)));

	geom[index] = new btCapsuleShape(btScalar(radius),btScalar(length));
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z)));

	btTransform transform;
	transform.setOrigin(btVector3(btScalar(0),btScalar(1),btScalar(0)));
	transform.getBasis().setEulerZYX(eulerX,eulerY,eulerZ); 

	body[index] = localCreateRigidBody(btScalar(1.0),offset*transform,geom[index]);
	body[index]->setUserPointer(&IDs[index]);
}

void RagdollDemo::CreateHinge(int jointIndex, int bodyAIndex, int bodyBIndex, const btVector3& axisInA, const btVector3& axisInB,
		const btVector3& pivotInA, const btVector3& pivotInB){
	
	joints[jointIndex] = new btHingeConstraint(*body[bodyAIndex], *body[bodyBIndex], pivotInA, pivotInB, axisInA, axisInB);
	//btHingeConstraint* tempHinge = new btHingeConstraint(*body[bodyAIndex], *body[bodyBIndex], pivotInA, pivotInB, axisInA, axisInB);
	m_dynamicsWorld->addConstraint(joints[jointIndex], true);
}

btVector3 RagdollDemo::PointWorldToLocal(int index, btVector3& p)
{
	btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
	return local1*p;
}

btVector3 RagdollDemo::AxisWorldToLocal(int index, btVector3& a) {
	btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
	btVector3 zero(0,0,0);
	local1.setOrigin(zero);
	return local1*a;
}

void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle, double jointOffset, double timeStep){

	switch(jointIndex){
		case 0:{
			jointOffset=-M_PI_2;
			   break;
			   }
		case 1:{
			jointOffset=M_PI_2;
			break;
			   }
		case 2:{
			jointOffset=-M_PI_2;
			break;
			   }
		case 3:{
			jointOffset=M_PI_2;
			break;
			   }
		case 4:{
			jointOffset=M_PI_2;
			break;
			   }
		case 5:{
			jointOffset=-M_PI_2;
			break;
			   }
		case 6:{
			jointOffset=M_PI_2;
			break;
			   }
		case 7:{
			jointOffset=-M_PI_2;
			break;
			   }
	}
	double maxForce = 5;
	
	/*
	double current_angle = joints[jointIndex]->getHingeAngle() - jointOffset;//returns current angle in radians
	double diff = desiredAngle - current_angle;

	std::cout << diff << std::endl;

	joints[jointIndex]->enableAngularMotor(true, 1*diff, maxForce);
	*/
	//Commented out below is essentially 'actuate joint 1'
	joints[jointIndex]->enableMotor(TRUE);
	joints[jointIndex]->setMaxMotorImpulse(maxForce);
	joints[jointIndex]->setMotorTarget(desiredAngle + jointOffset, 1);
	

}

float RagdollDemo::RandFloat(float min, float max){
	float r = (float)rand() / (float)RAND_MAX;
	return (double)min + r*(max-min);
}

void RagdollDemo::CreateRobot(){
	
	//Create the body @ Index 0
	CreateBox(0, 0,1.8, 0, 1, 0.2, 1);

	//Right legs
	CreateCylinder(1, -1.8, 0.8, 0, .2, 1.5, 0, 0, -M_PI_2);//Upper-Right
	CreateHinge(0, 0, 1, AxisWorldToLocal(0, btVector3(0,0,-1)), AxisWorldToLocal(1, btVector3(0,0,-1)), 
		PointWorldToLocal(0, btVector3(-1, 1.8, 0)), PointWorldToLocal(1,btVector3(-1,1.8,0)));
	CreateCylinder(5, -2.6, 0, 0, .2, 1.5, 0, 0, 0);//Lower-Right
	CreateHinge(1, 1, 5, AxisWorldToLocal(1, btVector3(0,0,-1)), AxisWorldToLocal(5, btVector3(0,0,-1)), 
		PointWorldToLocal(1, btVector3(-2.6, 1.8, 0)), PointWorldToLocal(5,btVector3(-2.6,1.8,0)));
	joints[0]->setLimit(-M_PI_2 - M_PI_4, -M_PI_2 + M_PI_4);
	joints[1]->setLimit(M_PI_2 - M_PI_4, M_PI_2 + M_PI_4);

	//Left Legs
	CreateCylinder(3, 1.8, 0.8, 0, .2, 1.5, 0, 0, M_PI_2); //Upper-Left
	CreateHinge(2, 0, 3, AxisWorldToLocal(0, btVector3(0,0,1)), AxisWorldToLocal(3, btVector3(0,0,1)), 
		PointWorldToLocal(0, btVector3(1, 1.8, 0)), PointWorldToLocal(3,btVector3(1,1.8,0)));
	CreateCylinder(6, 2.6, 0, 0, .2, 1.5, 0, 0, 0);//Lower-Left
	CreateHinge(3, 3, 6, AxisWorldToLocal(3, btVector3(0,0,1)), AxisWorldToLocal(6, btVector3(0,0,1)), 
		PointWorldToLocal(3, btVector3(2.6, 1.8, 0)), PointWorldToLocal(6,btVector3(2.6,1.8,0)));
	joints[2]->setLimit(-M_PI_2 - M_PI_4, -M_PI_2 + M_PI_4);
	joints[3]->setLimit(M_PI_2 - M_PI_4, M_PI_2 + M_PI_4);

	//Front Legs
	CreateCylinder(5, 0, 0.8, -1.8, .2, 1.5, -M_PI_2, 0, 0);//Upper-Front
	CreateHinge(4, 0, 5, AxisWorldToLocal(0, btVector3(1,0,0)), AxisWorldToLocal(5, btVector3(1,0,0)), 
		PointWorldToLocal(0, btVector3(0, 1.8, -1)), PointWorldToLocal(5,btVector3(0,1.8,-1)));
	CreateCylinder(7, 0, 0, -2.6, .2, 1.5, 0, 0, 0);//Lower-Front
	CreateHinge(5, 5, 7, AxisWorldToLocal(5, btVector3(1,0,0)), AxisWorldToLocal(7, btVector3(1,0,0)), 
		PointWorldToLocal(5, btVector3(0, 1.8, -2.6)), PointWorldToLocal(7,btVector3(0,1.8,-2.6)));
	joints[4]->setLimit(M_PI_2 - M_PI_4, M_PI_2 + M_PI_4);
	joints[5]->setLimit(-M_PI_2 - M_PI_4, -M_PI_2 + M_PI_4);

	//Back legs
	CreateCylinder(7, 0, 0.8, 1.8, .2, 1.5, M_PI_2, 0, 0); //Upper-Back
	CreateHinge(6, 0, 7, AxisWorldToLocal(0, btVector3(-1,0,0)), AxisWorldToLocal(7, btVector3(-1,0,0)), 
		PointWorldToLocal(0, btVector3(0, 1.8, 1)), PointWorldToLocal(7,btVector3(0,1.8,1)));
	CreateCylinder(8, 0, 0, 2.6, .2, 1.5, 0, 0, 0);//Lower-Back
	CreateHinge(7, 7, 8, AxisWorldToLocal(7, btVector3(-1,0,0)), AxisWorldToLocal(8, btVector3(-1,0,0)), 
		PointWorldToLocal(7, btVector3(0, 1.8, 2.6)), PointWorldToLocal(8,btVector3(0,1.8,2.6)));
	joints[6]->setLimit(M_PI_2 - M_PI_4, M_PI_2 + M_PI_4);
	joints[7]->setLimit(-M_PI_2 - M_PI_4, -M_PI_2 + M_PI_4);
}