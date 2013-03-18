/*#include "Robot.h"
//#include "R.cpp"


Robot::Robot(void)
{
}

Robot::Robot(RagdollDemo& dem)
{
	demo=dem;

	CreateBox(0, 0, 1, 0, 1, 0.2, 1);
}


Robot::~Robot(void)
{
}

void Robot::CreateBox(int index, double x, double y, double z, double length, double width, double height){
	geom[index] = new btBoxShape(btVector3(btScalar(length),btScalar(width),btScalar(height))); 
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z))); 
	body[index] = demo.localCreateRigidBody(btScalar(1.0),offset,geom[index]); 
}

*/