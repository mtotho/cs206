/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
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

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H
#include "GLDebugDrawer.h"
#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
//#include "Robot.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class btHingeConstraint;

class RagdollDemo : public GlutDemoApplication
{

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:
	virtual void renderme() {
		extern GLDebugDrawer gDebugDrawer;
		// Call the parent method.
		GlutDemoApplication::renderme();
		// Make a circle with a 0.9 radius at (0,0,0)
		// with RGB color (1,0,0).
		//DebugDrawer.drawSphere(btVector3(0.,0.,0.),
		//0.9, btVector3(1., 0., 0.));

		for(int i = 0; i<10; i++){
			if(touches[i] == 1){
				//sleep(100);
				gDebugDrawer.drawSphere(touchPoints[i], 0.4, btVector3(1., 0., 0.));

			}
		}
	}
	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
	//Start Assignments
	btRigidBody* body[9];
	btCollisionShape* geom[9];
	btHingeConstraint* joints[8];
	bool pause;
	bool oneStep;
	int IDs[10];
	int *touches;
	double weights[4][8];
	btVector3 touchPoints[10];
	int counter;
	float minFPS;
	
	//bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1);

	void CreateBox(int index, double x, double y, double z, double length, double width, double height);
	void CreateCylinder(int index,double x, double y, double z,double radius, double length, double eulerX, double eulerY, double eulerZ);
	void CreateHinge(int jointIndex,
		int bodyAIndex,
		int bodyBIndex,
		const btVector3& axisInA,
		const btVector3& axisInB,
		const btVector3& pivotInA,
		const btVector3& pivotInB);
	btVector3 PointWorldToLocal(int bodyIndex, btVector3& point);
	btVector3 AxisWorldToLocal(int index, btVector3& a);
	void ActuateJoint(int jointIndex, double desiredAngle, double jointOffset, double timeStep);
	//void setUserPointer(void *data);
	float RandFloat(float min, float max);
	void CreateRobot();
	
	
};


#endif
