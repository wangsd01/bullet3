/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "Tensegrity.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Importers/ImportMJCFDemo/ImportMJCFSetup.h"

struct Tensegrity : public CommonRigidBodyBase
{
	Tensegrity(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~Tensegrity() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void Tensegrity::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies, i.e. create ground. which contains to setup ground collision shape, groundTransform and mass.
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	{
		//create a few dynamic rigidbodies, i.e. boxes. Here reuse the collision shape
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* cylShape = new btCylinderShape(btVector3(0.25, 0.5, 0.25)); // radius, height, not used.
		m_collisionShapes.push_back(cylShape);
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(btScalar(0), btScalar(1), btScalar(0)));
		btScalar mass(1.f);

		btRigidBody* body = createRigidBody(mass, startTransform, cylShape);
		body->setRollingFriction(0.0);
		body->setSpinningFriction(0.0);
		body->setFriction(0.0);

		//////////////////////////
		//compute the local 'fromto' transform
		// btVector3 f = col->m_geometry.m_capsuleFrom;
		// 				btVector3 t = col->m_geometry.m_capsuleTo;

		// 				//compute the local 'fromto' transform
		// 				btVector3 localPosition = btScalar(0.5) * (t + f);
		// 				btQuaternion localOrn;
		// 				localOrn = btQuaternion::getIdentity();

		// 				btVector3 diff = t - f;
		// 				btScalar lenSqr = diff.length2();
		// 				btScalar height = 0.f;

		// 				if (lenSqr > SIMD_EPSILON)
		// 				{
		// 					height = btSqrt(lenSqr);
		// 					btVector3 ax = diff / height;

		// 					btVector3 zAxis(0, 0, 1);
		// 					localOrn = shortestArcQuat(zAxis, ax);
		// 				}
		// 				btCylinderShapeZ* cyl = new btCylinderShapeZ(btVector3(col->m_geometry.m_capsuleRadius, col->m_geometry.m_capsuleRadius, btScalar(0.5) * height));

		// 				btCompoundShape* compound = new btCompoundShape();
		// 				btTransform localTransform(localOrn, localPosition);
		// 				compound->addChildShape(localTransform, cyl);
		// 				childShape = compound;
		////////////////////////////
		// body->setRollingFriction(0.03);
		// body->setSpinningFriction(0.03);
		// body->setFriction(1);
		// body->setAnisotropicFriction(cylShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);



		// bool isDynamic = (mass != 0.f);
		// btVector3 localInertia(0, 0, 0);
		// if (isDynamic)
		// 	cylShape->calculateLocalInertia(mass, localInertia);
		// btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, 0, cylShape, localInertia);
		// rbInfo.m_startWorldTransform = startTransform;
		// btRigidBody* body = new btRigidBody(rbInfo);
		// body->setRollingFriction(0.03);
		// body->setSpinningFriction(0.03);
		// body->setFriction(1);
		// body->setAnisotropicFriction(cylShape->getAnisotropicRollingFrictionDirection(), btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);

		// m_dynamicsWorld->addRigidBody(body);

		// btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));

		// //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		// m_collisionShapes.push_back(colShape);

		// /// Create Dynamic Objects
		// btTransform startTransform;
		// startTransform.setIdentity();

		// btScalar mass(1.f);

		// //rigidbody is dynamic if and only if mass is non zero, otherwise static
		// bool isDynamic = (mass != 0.f);

		// btVector3 localInertia(0, 0, 0);
		// if (isDynamic)
		// 	colShape->calculateLocalInertia(mass, localInertia);

		// for (int k = 0; k < ARRAY_SIZE_Y; k++)
		// {
		// 	for (int i = 0; i < ARRAY_SIZE_X; i++)
		// 	{
		// 		for (int j = 0; j < ARRAY_SIZE_Z; j++)
		// 		{
		// 			startTransform.setOrigin(btVector3(
		// 				btScalar(0.2 * i),
		// 				btScalar(2 + .2 * k),
		// 				btScalar(0.2 * j)));

		// 			createRigidBody(mass, startTransform, colShape);
		// 		}
		// 	}
		// }
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void Tensegrity::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

class CommonExampleInterface* TensegrityCreateFunc(CommonExampleOptions& options)
{
	// return new ImportMJCFSetup(options.m_guiHelper, options.m_option, options.m_fileName);
	return new Tensegrity(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(TensegrityCreateFunc)
