/*#pragma once

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "RagdollDemo.h"
#include "RagdollDemo.cpp"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class Robot
{
public:
	Robot(void);
	Robot(RagdollDemo& dem);
	~Robot(void);

	RagdollDemo& demo;
	btRigidBody* body[9];
	btCollisionShape* geom[9];

	void CreateBox(int index, double x, double y, double z, double length, double width, double height);
};

*/