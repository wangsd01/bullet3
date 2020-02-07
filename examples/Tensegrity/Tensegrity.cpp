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

#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

const int maxProxies = 32766;

void tensegrityPickingPreTickCallback(btDynamicsWorld* world, btScalar timeStep);

struct Tensegrity : public CommonRigidBodyBase
{
	Tensegrity(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~Tensegrity() {}
	virtual void initPhysics();
	virtual void renderScene();
	btRigidBody* createCylinderByFromTo(btVector3 cylinderFrom, btVector3 cylinderTo, btScalar cylinderRadius);
	void evaluateEquations();
	void createEmptySoftRigidDynamicsWorld();
	btSoftBodyWorldInfo m_softBodyWorldInfo;

	bool m_autocam;
	bool m_cutting;
	bool m_raycast;
	btScalar m_animtime;
	btClock m_clock;
	int m_lastmousepos[2];
	btVector3 m_impact;
	btSoftBody::sRayCast m_results;
	btSoftBody::Node* m_node;
	btVector3 m_goal;
	bool m_drag;

	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld()
	{
		///just make it a btSoftRigidDynamicsWorld please
		///or we will add type checking
		return (btSoftRigidDynamicsWorld*)m_dynamicsWorld;
	}

	GUIHelperInterface* getGUIHelper()
	{
		return m_guiHelper;
	}
};

btRigidBody* Tensegrity::createCylinderByFromTo(btVector3 cylinderFrom, btVector3 cylinderTo, btScalar cylinderRadius)
{
//////////////////////////
	btScalar mass(1.f);
	//compute the local 'fromto' transform
	btVector3 f = cylinderFrom;
	btVector3 t = cylinderTo;

	btVector3 localPosition = btScalar(0.5) * (t + f);
	btQuaternion localOrn;
	localOrn = btQuaternion::getIdentity();

	btVector3 diff = t - f;
	btScalar lenSqr = diff.length2();
	btScalar height = 0.f;

	if (lenSqr > SIMD_EPSILON)
	{
		height = btSqrt(lenSqr);
		btVector3 ax = diff / height;

		btVector3 zAxis(0, 0, 1);
		localOrn = shortestArcQuat(zAxis, ax);
	}
	btCylinderShapeZ* cylShape = new btCylinderShapeZ(btVector3(cylinderRadius, cylinderRadius, btScalar(0.5) * height));
	m_collisionShapes.push_back(cylShape);

	btTransform localTransform(localOrn, localPosition);
	btRigidBody* body = createRigidBody(mass, localTransform, cylShape);
	body->setRollingFriction(0.0);
	body->setSpinningFriction(0.0);
	body->setFriction(0.0);
	return body;
	// btCompoundShape* compound = new btCompoundShape();
	// btTransform localTransform(localOrn, localPosition);
	// compound->addChildShape(localTransform, cyl);
	// return compound;
////////////////////////////
}

void Tensegrity::createEmptySoftRigidDynamicsWorld()
{
	m_dispatcher = 0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm

	///Register softbody versus rigidbody collision algorithm

	////////////////////////////

	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	m_broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btSoftBodySolver* softBodySolver = 0;
#ifdef USE_AMD_OPENCL

	static bool once = true;
	if (once)
	{
		once = false;
		initCL(0, 0);
	}

	if (g_openCLSIMDSolver)
		delete g_openCLSIMDSolver;
	if (g_softBodyOutput)
		delete g_softBodyOutput;

	if (1)
	{
		g_openCLSIMDSolver = new btOpenCLSoftBodySolverSIMDAware(g_cqCommandQue, g_cxMainContext);
		//	g_openCLSIMDSolver = new btOpenCLSoftBodySolver( g_cqCommandQue, g_cxMainContext);
		g_openCLSIMDSolver->setCLFunctions(new CachingCLFunctions(g_cqCommandQue, g_cxMainContext));
	}

	softBodySolver = g_openCLSIMDSolver;
	g_softBodyOutput = new btSoftBodySolverOutputCLtoCPU;
#endif  //USE_AMD_OPENCL

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);
	m_dynamicsWorld = world;
	m_dynamicsWorld->setInternalTickCallback(tensegrityPickingPreTickCallback, this, true);

	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);
	// m_guiHelper->createPhysicsDebugDrawer(world);
	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	//	clientResetScene();
}

void Tensegrity::evaluateEquations()
{
	btScalar bodyUniqueId(0);
	btAlignedObjectArray<btRigidBody*>& rbs = m_dynamicsWorld->getNonStaticRigidBodies();
	btRigidBody* rb = rbs[0];
	btTransform tr = rb->getWorldTransform();
	btVector3 position = tr.getOrigin();
	btQuaternion quat = tr.getRotation();
	btVector3 lin_vel = rb->getLinearVelocity();
	btVector3 ang_vel = rb->getAngularVelocity();

	btVector3 force = rb->getTotalForce();
	btVector3 torque = rb->getTotalTorque();


	btAlignedObjectArray<btSoftBody*>& sbs = getSoftDynamicsWorld()->getSoftBodyArray();
	btSoftBody* sb = sbs[0];

	btSoftBody::Anchor& a = sb->m_anchors[0];
	const btVector3 wa = a.m_body->getWorldTransform() * a.m_local;
	
}

void Tensegrity::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	// createEmptyDynamicsWorld();
	createEmptySoftRigidDynamicsWorld();
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

		btCollisionShape* cylShape = new btCylinderShape(btVector3(0.5, 0.5, 0.25)); // radius, half_height, not used.
		btCollisionShape* sphere1 = new btSphereShape(0.1);
		btCompoundShape* cyl0 = new btCompoundShape();
			
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0.5, 0.5, 0));
		cyl0->addChildShape(btTransform::getIdentity(), cylShape);
		cyl0->addChildShape(startTransform, sphere1);

		btScalar mass = 6.28;
		btVector3 localInertia;
		cyl0->calculateLocalInertia(mass, localInertia);
		btRigidBody::btRigidBodyConstructionInfo ci(mass, 0, cyl0, localInertia);
		ci.m_startWorldTransform.setOrigin(btVector3(0, 5, 0));
		btRigidBody* body = new btRigidBody(ci);  //1,0,cyl0,localInertia);
		m_dynamicsWorld->addRigidBody(body);

		// m_collisionShapes.push_back(cylShape);
		// btTransform startTransform;
		// startTransform.setIdentity();
		// startTransform.setOrigin(btVector3(btScalar(0), btScalar(0.5), btScalar(0))); // center of mass.
		// btScalar mass(1.f);
		// btRigidBody* body = createRigidBody(mass, startTransform, cylShape);
		// body->setRollingFriction(0.0);
		// body->setSpinningFriction(0.0);
		// body->setFriction(0.0);

		// btRigidBody* cylFromTo = createCylinderByFromTo(btVector3(2, 5, 1), btVector3(2, 6, 1), btScalar(0.5));


		//create a rope
		btSoftBody* psb = btSoftBodyHelpers::CreateRope(m_softBodyWorldInfo, btVector3(0, 8, -1),
												btVector3(0.5, 5.5, 0),
												0,
												1); // 1 means only anchor first end. 1+2 means anchor both ends.
		psb->m_cfg.piterations = 4;
		psb->m_materials[0]->m_kLST = 0.1 + 0.9;
		psb->setTotalMass(1.);
		getSoftDynamicsWorld()->addSoftBody(psb);

		psb->appendAnchor(psb->m_nodes.size() - 1, body);

		
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
	btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

	for (int i = 0; i < softWorld->getSoftBodyArray().size(); i++)
	{
		btSoftBody* psb = (btSoftBody*)softWorld->getSoftBodyArray()[i];
		//if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
		{
			btSoftBodyHelpers::DrawFrame(psb, softWorld->getDebugDrawer());
			btSoftBodyHelpers::Draw(psb, softWorld->getDebugDrawer(), softWorld->getDrawFlags());
		}
	}
}

class CommonExampleInterface* TensegrityCreateFunc(CommonExampleOptions& options)
{
	// return new ImportMJCFSetup(options.m_guiHelper, options.m_option, options.m_fileName);
	return new Tensegrity(options.m_guiHelper);
}


////////////////////////////////////
///for mouse picking
void tensegrityPickingPreTickCallback(btDynamicsWorld* world, btScalar timeStep)
{
	// Tensegrity* tensegrity = (Tensegrity*)world->getWorldUserInfo();

	// if (tensegrity->m_drag)
	// {
	// 	const int x = tensegrity->m_lastmousepos[0];
	// 	const int y = tensegrity->m_lastmousepos[1];
	// 	float rf[3];
	// 	tensegrity->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraPosition(rf);
	// 	float target[3];
	// 	tensegrity->getGUIHelper()->getRenderInterface()->getActiveCamera()->getCameraTargetPosition(target);
	// 	btVector3 cameraTargetPosition(target[0], target[1], target[2]);

	// 	const btVector3 cameraPosition(rf[0], rf[1], rf[2]);
	// 	const btVector3 rayFrom = cameraPosition;

	// 	const btVector3 rayTo = tensegrity->getRayTo(x, y);
	// 	const btVector3 rayDir = (rayTo - rayFrom).normalized();
	// 	const btVector3 N = (cameraTargetPosition - cameraPosition).normalized();
	// 	const btScalar O = btDot(tensegrity->m_impact, N);
	// 	const btScalar den = btDot(N, rayDir);
	// 	if ((den * den) > 0)
	// 	{
	// 		const btScalar num = O - btDot(N, rayFrom);
	// 		const btScalar hit = num / den;
	// 		if ((hit > 0) && (hit < 1500))
	// 		{
	// 			tensegrity->m_goal = rayFrom + rayDir * hit;
	// 		}
	// 	}
	// 	btVector3 delta = tensegrity->m_goal - tensegrity->m_node->m_x;
	// 	static const btScalar maxdrag = 10;
	// 	if (delta.length2() > (maxdrag * maxdrag))
	// 	{
	// 		delta = delta.normalized() * maxdrag;
	// 	}
	// 	tensegrity->m_node->m_v += delta / timeStep;
	// }
}

B3_STANDALONE_EXAMPLE(TensegrityCreateFunc)
