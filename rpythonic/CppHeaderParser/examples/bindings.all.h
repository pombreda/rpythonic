//// ../src/btBulletDynamicsCommon.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BULLET_DYNAMICS_COMMON_H
#define BULLET_DYNAMICS_COMMON_H

///Common headerfile includes for Bullet Dynamics, including Collision Detection
#include "btBulletCollisionCommon.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btContinuousDynamicsWorld.h"

#include "BulletDynamics/Dynamics/btSimpleDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/ConstraintSolver/btUniversalConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHinge2Constraint.h"

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"


///Vehicle simulation, with wheel contact simulated by raycasts
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"






#endif //BULLET_DYNAMICS_COMMON_H

//// ../src/btBulletCollisionCommon.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BULLET_COLLISION_COMMON_H
#define BULLET_COLLISION_COMMON_H

///Common headerfile includes for Bullet Collision Detection

///Bullet's btCollisionWorld and btCollisionObject definitions
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

///Collision Shapes
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btEmptyShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"

///Narrowphase Collision Detector
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"

//btSphereBoxCollisionAlgorithm is broken, use gjk for now
//#include "BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

///Dispatching and generation of collision pairs (broadphase)
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

///Math library & Utils
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btSerializer.h"


#endif //BULLET_COLLISION_COMMON_H

//// ../src/BulletCollision/CollisionDispatch/btCollisionWorld.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://bulletphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


/**
 * @mainpage Bullet Documentation
 *
 * @section intro_sec Introduction
 * Bullet Collision Detection & Physics SDK
 *
 * Bullet is a Collision Detection and Rigid Body Dynamics Library. The Library is Open Source and free for commercial use, under the ZLib license ( http://opensource.org/licenses/zlib-license.php ).
 *
 * The main documentation is Bullet_User_Manual.pdf, included in the source code distribution.
 * There is the Physics Forum for feedback and general Collision Detection and Physics discussions.
 * Please visit http://www.bulletphysics.com
 *
 * @section install_sec Installation
 *
 * @subsection step1 Step 1: Download
 * You can download the Bullet Physics Library from the Google Code repository: http://code.google.com/p/bullet/downloads/list
 *
 * @subsection step2 Step 2: Building
 * Bullet main build system for all platforms is cmake, you can download http://www.cmake.org
 * cmake can autogenerate projectfiles for Microsoft Visual Studio, Apple Xcode, KDevelop and Unix Makefiles.
 * The easiest is to run the CMake cmake-gui graphical user interface and choose the options and generate projectfiles.
 * You can also use cmake in the command-line. Here are some examples for various platforms:
 * cmake . -G "Visual Studio 9 2008"
 * cmake . -G Xcode
 * cmake . -G "Unix Makefiles"
 * Although cmake is recommended, you can also use autotools for UNIX: ./autogen.sh ./configure to create a Makefile and then run make.
 * 
 * @subsection step3 Step 3: Testing demos
 * Try to run and experiment with BasicDemo executable as a starting point.
 * Bullet can be used in several ways, as Full Rigid Body simulation, as Collision Detector Library or Low Level / Snippets like the GJK Closest Point calculation.
 * The Dependencies can be seen in this documentation under Directories
 * 
 * @subsection step4 Step 4: Integrating in your application, full Rigid Body and Soft Body simulation
 * Check out BasicDemo how to create a btDynamicsWorld, btRigidBody and btCollisionShape, Stepping the simulation and synchronizing your graphics object transform.
 * Check out SoftDemo how to use soft body dynamics, using btSoftRigidDynamicsWorld.
 * @subsection step5 Step 5 : Integrate the Collision Detection Library (without Dynamics and other Extras)
 * Bullet Collision Detection can also be used without the Dynamics/Extras.
 * Check out btCollisionWorld and btCollisionObject, and the CollisionInterfaceDemo.
 * @subsection step6 Step 6 : Use Snippets like the GJK Closest Point calculation.
 * Bullet has been designed in a modular way keeping dependencies to a minimum. The ConvexHullDistance demo demonstrates direct use of btGjkPairDetector.
 *
 * @section copyright Copyright
 * For up-to-data information and copyright and contributors list check out the Bullet_User_Manual.pdf
 * 
 */
 
 

#ifndef BT_COLLISION_WORLD_H
#define BT_COLLISION_WORLD_H

class btStackAlloc;
class btCollisionShape;
class btConvexShape;
class btBroadphaseInterface;
class btSerializer;

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "btCollisionObject.h"
#include "btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "LinearMath/btAlignedObjectArray.h"

///CollisionWorld is interface and container for the collision detection
class btCollisionWorld
{

 
protected:

 btAlignedObjectArray<btCollisionObject*> m_collisionObjects;
 
 btDispatcher* m_dispatcher1;

 btDispatcherInfo m_dispatchInfo;

 btStackAlloc* m_stackAlloc;

 btBroadphaseInterface* m_broadphasePairCache;

 btIDebugDraw* m_debugDrawer;

 ///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
 ///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
 bool m_forceUpdateAllAabbs;

 void serializeCollisionObjects(btSerializer* serializer);

public:

 //this constructor doesn't own the dispatcher and paircache/broadphase
 btCollisionWorld(btDispatcher* dispatcher,btBroadphaseInterface* broadphasePairCache, btCollisionConfiguration* collisionConfiguration);

 virtual ~btCollisionWorld();

 void setBroadphase(btBroadphaseInterface* pairCache)
 {
  m_broadphasePairCache = pairCache;
 }

 const btBroadphaseInterface* getBroadphase() const
 {
  return m_broadphasePairCache;
 }

 btBroadphaseInterface* getBroadphase()
 {
  return m_broadphasePairCache;
 }

 btOverlappingPairCache* getPairCache()
 {
  return m_broadphasePairCache->getOverlappingPairCache();
 }


 btDispatcher* getDispatcher()
 {
  return m_dispatcher1;
 }

 const btDispatcher* getDispatcher() const
 {
  return m_dispatcher1;
 }

 void updateSingleAabb(btCollisionObject* colObj);

 virtual void updateAabbs();
 
 virtual void setDebugDrawer(btIDebugDraw* debugDrawer)
 {
   m_debugDrawer = debugDrawer;
 }

 virtual btIDebugDraw* getDebugDrawer()
 {
  return m_debugDrawer;
 }

 virtual void debugDrawWorld();

 virtual void debugDrawObject(const btTransform& worldTransform, const btCollisionShape* shape, const btVector3& color);


 ///LocalShapeInfo gives extra information for complex shapes
 ///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
 struct LocalShapeInfo
 {
  int m_shapePart;
  int m_triangleIndex;
  
  //const btCollisionShape* m_shapeTemp;
  //const btTransform* m_shapeLocalTransform;
 };

 struct LocalRayResult
 {
  LocalRayResult(btCollisionObject* collisionObject, 
   LocalShapeInfo* localShapeInfo,
   const btVector3&  hitNormalLocal,
   btScalar hitFraction)
  :m_collisionObject(collisionObject),
  m_localShapeInfo(localShapeInfo),
  m_hitNormalLocal(hitNormalLocal),
  m_hitFraction(hitFraction)
  {
  }

  btCollisionObject*  m_collisionObject;
  LocalShapeInfo*   m_localShapeInfo;
  btVector3    m_hitNormalLocal;
  btScalar    m_hitFraction;

 };

 ///RayResultCallback is used to report new raycast results
 struct RayResultCallback
 {
  btScalar m_closestHitFraction;
  btCollisionObject*  m_collisionObject;
  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
      //@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback
      unsigned int m_flags;

  virtual ~RayResultCallback()
  {
  }
  bool hasHit() const
  {
   return (m_collisionObject != 0);
  }

  RayResultCallback()
   :m_closestHitFraction(btScalar(1.)),
   m_collisionObject(0),
   m_collisionFilterGroup(btBroadphaseProxy::DefaultFilter),
   m_collisionFilterMask(btBroadphaseProxy::AllFilter),
         //@BP Mod
         m_flags(0)
  {
  }

  virtual bool needsCollision(btBroadphaseProxy* proxy0) const
  {
   bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
   collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   return collides;
  }


  virtual btScalar addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace) = 0;
 };

 struct ClosestRayResultCallback : public RayResultCallback
 {
  ClosestRayResultCallback(const btVector3& rayFromWorld,const btVector3& rayToWorld)
  :m_rayFromWorld(rayFromWorld),
  m_rayToWorld(rayToWorld)
  {
  }

  btVector3 m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
  btVector3 m_rayToWorld;

  btVector3 m_hitNormalWorld;
  btVector3 m_hitPointWorld;
   
  virtual btScalar addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace)
  {
   //caller already does the filter on the m_closestHitFraction
   btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
   
   m_closestHitFraction = rayResult.m_hitFraction;
   m_collisionObject = rayResult.m_collisionObject;
   if (normalInWorldSpace)
   {
    m_hitNormalWorld = rayResult.m_hitNormalLocal;
   } else
   {
    ///need to transform normal into worldspace
    m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
   }
   m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
   return rayResult.m_hitFraction;
  }
 };

 struct AllHitsRayResultCallback : public RayResultCallback
 {
  AllHitsRayResultCallback(const btVector3& rayFromWorld,const btVector3& rayToWorld)
  :m_rayFromWorld(rayFromWorld),
  m_rayToWorld(rayToWorld)
  {
  }

  btAlignedObjectArray<btCollisionObject*>  m_collisionObjects;

  btVector3 m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
  btVector3 m_rayToWorld;

  btAlignedObjectArray<btVector3> m_hitNormalWorld;
  btAlignedObjectArray<btVector3> m_hitPointWorld;
  btAlignedObjectArray<btScalar> m_hitFractions;
   
  virtual btScalar addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace)
  {
   m_collisionObject = rayResult.m_collisionObject;
   m_collisionObjects.push_back(rayResult.m_collisionObject);
   btVector3 hitNormalWorld;
   if (normalInWorldSpace)
   {
    hitNormalWorld = rayResult.m_hitNormalLocal;
   } else
   {
    ///need to transform normal into worldspace
    hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
   }
   m_hitNormalWorld.push_back(hitNormalWorld);
   btVector3 hitPointWorld;
   hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
   m_hitPointWorld.push_back(hitPointWorld);
   m_hitFractions.push_back(rayResult.m_hitFraction);
   return m_closestHitFraction;
  }
 };


 struct LocalConvexResult
 {
  LocalConvexResult(btCollisionObject* hitCollisionObject, 
   LocalShapeInfo* localShapeInfo,
   const btVector3&  hitNormalLocal,
   const btVector3&  hitPointLocal,
   btScalar hitFraction
   )
  :m_hitCollisionObject(hitCollisionObject),
  m_localShapeInfo(localShapeInfo),
  m_hitNormalLocal(hitNormalLocal),
  m_hitPointLocal(hitPointLocal),
  m_hitFraction(hitFraction)
  {
  }

  btCollisionObject*  m_hitCollisionObject;
  LocalShapeInfo*   m_localShapeInfo;
  btVector3    m_hitNormalLocal;
  btVector3    m_hitPointLocal;
  btScalar    m_hitFraction;
 };

 ///RayResultCallback is used to report new raycast results
 struct ConvexResultCallback
 {
  btScalar m_closestHitFraction;
  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
  
  ConvexResultCallback()
   :m_closestHitFraction(btScalar(1.)),
   m_collisionFilterGroup(btBroadphaseProxy::DefaultFilter),
   m_collisionFilterMask(btBroadphaseProxy::AllFilter)
  {
  }

  virtual ~ConvexResultCallback()
  {
  }
  
  bool hasHit() const
  {
   return (m_closestHitFraction < btScalar(1.));
  }

  

  virtual bool needsCollision(btBroadphaseProxy* proxy0) const
  {
   bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
   collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   return collides;
  }

  virtual btScalar addSingleResult(LocalConvexResult& convexResult,bool normalInWorldSpace) = 0;
 };

 struct ClosestConvexResultCallback : public ConvexResultCallback
 {
  ClosestConvexResultCallback(const btVector3& convexFromWorld,const btVector3& convexToWorld)
  :m_convexFromWorld(convexFromWorld),
  m_convexToWorld(convexToWorld),
  m_hitCollisionObject(0)
  {
  }

  btVector3 m_convexFromWorld;//used to calculate hitPointWorld from hitFraction
  btVector3 m_convexToWorld;

  btVector3 m_hitNormalWorld;
  btVector3 m_hitPointWorld;
  btCollisionObject* m_hitCollisionObject;
  
  virtual btScalar addSingleResult(LocalConvexResult& convexResult,bool normalInWorldSpace)
  {
//caller already does the filter on the m_closestHitFraction
   btAssert(convexResult.m_hitFraction <= m_closestHitFraction);
      
   m_closestHitFraction = convexResult.m_hitFraction;
   m_hitCollisionObject = convexResult.m_hitCollisionObject;
   if (normalInWorldSpace)
   {
    m_hitNormalWorld = convexResult.m_hitNormalLocal;
   } else
   {
    ///need to transform normal into worldspace
    m_hitNormalWorld = m_hitCollisionObject->getWorldTransform().getBasis()*convexResult.m_hitNormalLocal;
   }
   m_hitPointWorld = convexResult.m_hitPointLocal;
   return convexResult.m_hitFraction;
  }
 };

 ///ContactResultCallback is used to report contact points
 struct ContactResultCallback
 {
  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
  
  ContactResultCallback()
   :m_collisionFilterGroup(btBroadphaseProxy::DefaultFilter),
   m_collisionFilterMask(btBroadphaseProxy::AllFilter)
  {
  }

  virtual ~ContactResultCallback()
  {
  }
  
  virtual bool needsCollision(btBroadphaseProxy* proxy0) const
  {
   bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
   collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   return collides;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1) = 0;
 };



 int getNumCollisionObjects() const
 {
  return int(m_collisionObjects.size());
 }

 /// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
 /// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
 virtual void rayTest(const btVector3& rayFromWorld, const btVector3& rayToWorld, RayResultCallback& resultCallback) const; 

 /// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
 /// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
 void    convexSweepTest (const btConvexShape* castShape, const btTransform& from, const btTransform& to, ConvexResultCallback& resultCallback,  btScalar allowedCcdPenetration = btScalar(0.)) const;

 ///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
 ///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
 void contactTest(btCollisionObject* colObj, ContactResultCallback& resultCallback);

 ///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
 ///it reports one or more contact points (including the one with deepest penetration)
 void contactPairTest(btCollisionObject* colObjA, btCollisionObject* colObjB, ContactResultCallback& resultCallback);


 /// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
 /// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
 /// This allows more customization.
 static void rayTestSingle(const btTransform& rayFromTrans,const btTransform& rayToTrans,
       btCollisionObject* collisionObject,
       const btCollisionShape* collisionShape,
       const btTransform& colObjWorldTransform,
       RayResultCallback& resultCallback);

 /// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
 static void objectQuerySingle(const btConvexShape* castShape, const btTransform& rayFromTrans,const btTransform& rayToTrans,
       btCollisionObject* collisionObject,
       const btCollisionShape* collisionShape,
       const btTransform& colObjWorldTransform,
       ConvexResultCallback& resultCallback, btScalar allowedPenetration);

 virtual void addCollisionObject(btCollisionObject* collisionObject,short int collisionFilterGroup=btBroadphaseProxy::DefaultFilter,short int collisionFilterMask=btBroadphaseProxy::AllFilter);

 btCollisionObjectArray& getCollisionObjectArray()
 {
  return m_collisionObjects;
 }

 const btCollisionObjectArray& getCollisionObjectArray() const
 {
  return m_collisionObjects;
 }


 virtual void removeCollisionObject(btCollisionObject* collisionObject);

 virtual void performDiscreteCollisionDetection();

 btDispatcherInfo& getDispatchInfo()
 {
  return m_dispatchInfo;
 }

 const btDispatcherInfo& getDispatchInfo() const
 {
  return m_dispatchInfo;
 }
 
 bool getForceUpdateAllAabbs() const
 {
  return m_forceUpdateAllAabbs;
 }
 void setForceUpdateAllAabbs( bool forceUpdateAllAabbs)
 {
  m_forceUpdateAllAabbs = forceUpdateAllAabbs;
 }

 ///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (Bullet/Demos/SerializeDemo)
 virtual void serialize(btSerializer* serializer);

};


#endif //BT_COLLISION_WORLD_H
//// ../src/LinearMath/btVector3.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_VECTOR3_H
#define BT_VECTOR3_H


#include "btScalar.h"
#include "btMinMax.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btVector3Data btVector3DoubleData
#define btVector3DataName "btVector3DoubleData"
#else
#define btVector3Data btVector3FloatData
#define btVector3DataName "btVector3FloatData"
#endif //BT_USE_DOUBLE_PRECISION




/**@brief btVector3 can be used to represent 3D points and vectors.
 * It has an un-used w component to suit 16-byte alignment when btVector3 is stored in containers. This extra component can be used by derived classes (Quaternion?) or by user
 * Ideally, this class should be replaced by a platform optimized SIMD version that keeps the data in registers
 */
class btVector3
{
public:

#if 0
  btScalar m_floats[4];
public:
  const vec_float4& get128() const
 {
  return *((const vec_float4*)&m_floats[0]);
 }
public:
#else //__CELLOS_LV2__ __SPU__

#endif //__CELLOS_LV2__ __SPU__

 public:

  /**@brief No initialization constructor */
  btVector3() {}

 
 
  /**@brief Constructor from scalars 
   * @param x X value
   * @param y Y value 
   * @param z Z value 
   */
  btVector3(const btScalar& x, const btScalar& y, const btScalar& z)
 {
  m_floats[0] = x;
  m_floats[1] = y;
  m_floats[2] = z;
  m_floats[3] = btScalar(0.);
 }

 
/**@brief Add a vector to this one 
 * @param The vector to add to this one */
  btVector3& operator+=(const btVector3& v)
 {

  m_floats[0] += v.m_floats[0]; m_floats[1] += v.m_floats[1];m_floats[2] += v.m_floats[2];
  return *this;
 }


  /**@brief Subtract a vector from this one
   * @param The vector to subtract */
  btVector3& operator-=(const btVector3& v) 
 {
  m_floats[0] -= v.m_floats[0]; m_floats[1] -= v.m_floats[1];m_floats[2] -= v.m_floats[2];
  return *this;
 }
  /**@brief Scale the vector
   * @param s Scale factor */
  btVector3& operator*=(const btScalar& s)
 {
  m_floats[0] *= s; m_floats[1] *= s;m_floats[2] *= s;
  return *this;
 }

  /**@brief Inversely scale the vector 
   * @param s Scale factor to divide by */
  btVector3& operator/=(const btScalar& s) 
 {
  btFullAssert(s != btScalar(0.0));
  return *this *= btScalar(1.0) / s;
 }

  /**@brief Return the dot product
   * @param v The other vector in the dot product */
  btScalar dot(const btVector3& v) const
 {
  return m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1] +m_floats[2] * v.m_floats[2];
 }

  /**@brief Return the length of the vector squared */
  btScalar length2() const
 {
  return dot(*this);
 }

  /**@brief Return the length of the vector */
  btScalar length() const
 {
  return btSqrt(length2());
 }

  /**@brief Return the distance squared between the ends of this and another vector
   * This is symantically treating the vector like a point */
  btScalar distance2(const btVector3& v) const;

  /**@brief Return the distance between the ends of this and another vector
   * This is symantically treating the vector like a point */
  btScalar distance(const btVector3& v) const;

  btVector3& safeNormalize() 
 {
  btVector3 absVec = this->absolute();
  int maxIndex = absVec.maxAxis();
  if (absVec[maxIndex]>0)
  {
   *this /= absVec[maxIndex];
   return *this /= length();
  }
  setValue(1,0,0);
  return *this;
 }

  /**@brief Normalize this vector 
   * x^2 + y^2 + z^2 = 1 */
  btVector3& normalize() 
 {
  return *this /= length();
 }

  /**@brief Return a normalized version of this vector */
  btVector3 normalized() const;

  /**@brief Return a rotated version of this vector
   * @param wAxis The axis to rotate about 
   * @param angle The angle to rotate by */
  btVector3 rotate( const btVector3& wAxis, const btScalar angle ) const;

  /**@brief Return the angle between this and another vector
   * @param v The other vector */
  btScalar angle(const btVector3& v) const 
 {
  btScalar s = btSqrt(length2() * v.length2());
  btFullAssert(s != btScalar(0.0));
  return btAcos(dot(v) / s);
 }
  /**@brief Return a vector will the absolute values of each element */
  btVector3 absolute() const 
 {
  return btVector3(
   btFabs(m_floats[0]), 
   btFabs(m_floats[1]), 
   btFabs(m_floats[2]));
 }
  /**@brief Return the cross product between this and another vector 
   * @param v The other vector */
  btVector3 cross(const btVector3& v) const
 {
  return btVector3(
   m_floats[1] * v.m_floats[2] -m_floats[2] * v.m_floats[1],
   m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2],
   m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]);
 }

  btScalar triple(const btVector3& v1, const btVector3& v2) const
 {
  return m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) + 
   m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) + 
   m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
 }

  /**@brief Return the axis with the smallest value 
   * Note return values are 0,1,2 for x, y, or z */
  int minAxis() const
 {
  return m_floats[0] < m_floats[1] ? (m_floats[0] <m_floats[2] ? 0 : 2) : (m_floats[1] <m_floats[2] ? 1 : 2);
 }

  /**@brief Return the axis with the largest value 
   * Note return values are 0,1,2 for x, y, or z */
  int maxAxis() const 
 {
  return m_floats[0] < m_floats[1] ? (m_floats[1] <m_floats[2] ? 2 : 1) : (m_floats[0] <m_floats[2] ? 2 : 0);
 }

  int furthestAxis() const
 {
  return absolute().minAxis();
 }

  int closestAxis() const 
 {
  return absolute().maxAxis();
 }

  void setInterpolate3(const btVector3& v0, const btVector3& v1, btScalar rt)
 {
  btScalar s = btScalar(1.0) - rt;
  m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
  m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
  m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
  //don't do the unused w component
  //  m_co[3] = s * v0[3] + rt * v1[3];
 }

  /**@brief Return the linear interpolation between this and another vector 
   * @param v The other vector 
   * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
  btVector3 lerp(const btVector3& v, const btScalar& t) const 
 {
  return btVector3(m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
   m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
   m_floats[2] + (v.m_floats[2] -m_floats[2]) * t);
 }

  /**@brief Elementwise multiply this vector by the other 
   * @param v The other vector */
  btVector3& operator*=(const btVector3& v)
 {
  m_floats[0] *= v.m_floats[0]; m_floats[1] *= v.m_floats[1];m_floats[2] *= v.m_floats[2];
  return *this;
 }

  /**@brief Return the x value */
   const btScalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
   const btScalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
   const btScalar& getZ() const { return m_floats[2]; }
  /**@brief Set the x value */
   void setX(btScalar x) { m_floats[0] = x;};
  /**@brief Set the y value */
   void setY(btScalar y) { m_floats[1] = y;};
  /**@brief Set the z value */
   void setZ(btScalar z) {m_floats[2] = z;};
  /**@brief Set the w value */
   void setW(btScalar w) { m_floats[3] = w;};
  /**@brief Return the x value */
   const btScalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
   const btScalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
   const btScalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
   const btScalar& w() const { return m_floats[3]; }

 // btScalar&       operator[](int i)       { return (&m_floats[0])[i]; }      
 // const btScalar& operator[](int i) const { return (&m_floats[0])[i]; }
 ///operator btScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
  operator       btScalar *()       { return &m_floats[0]; }
  operator const btScalar *() const { return &m_floats[0]; }

  bool operator==(const btVector3& other) const
 {
  return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
 }

  bool operator!=(const btVector3& other) const
 {
  return !(*this == other);
 }

  /**@brief Set each element to the max of the current values and the values of another btVector3
   * @param other The other btVector3 to compare with 
   */
   void setMax(const btVector3& other)
  {
   btSetMax(m_floats[0], other.m_floats[0]);
   btSetMax(m_floats[1], other.m_floats[1]);
   btSetMax(m_floats[2], other.m_floats[2]);
   btSetMax(m_floats[3], other.w());
  }
  /**@brief Set each element to the min of the current values and the values of another btVector3
   * @param other The other btVector3 to compare with 
   */
   void setMin(const btVector3& other)
  {
   btSetMin(m_floats[0], other.m_floats[0]);
   btSetMin(m_floats[1], other.m_floats[1]);
   btSetMin(m_floats[2], other.m_floats[2]);
   btSetMin(m_floats[3], other.w());
  }

   void  setValue(const btScalar& x, const btScalar& y, const btScalar& z)
  {
   m_floats[0]=x;
   m_floats[1]=y;
   m_floats[2]=z;
   m_floats[3] = btScalar(0.);
  }

  void getSkewSymmetricMatrix(btVector3* v0,btVector3* v1,btVector3* v2) const
  {
   v0->setValue(0.  ,-z()  ,y());
   v1->setValue(z() ,0.   ,-x());
   v2->setValue(-y() ,x() ,0.);
  }

  void setZero()
  {
   setValue(btScalar(0.),btScalar(0.),btScalar(0.));
  }

   bool isZero() const 
  {
   return m_floats[0] == btScalar(0) && m_floats[1] == btScalar(0) && m_floats[2] == btScalar(0);
  }

   bool fuzzyZero() const 
  {
   return length2() < SIMD_EPSILON;
  }

   void serialize(struct btVector3Data& dataOut) const;

   void deSerialize(const struct btVector3Data& dataIn);

   void serializeFloat(struct btVector3FloatData& dataOut) const;

   void deSerializeFloat(const struct btVector3FloatData& dataIn);

   void serializeDouble(struct btVector3DoubleData& dataOut) const;

   void deSerializeDouble(const struct btVector3DoubleData& dataIn);

};

/**@brief Return the sum of two vectors (Point symantics)*/
 btVector3 
operator+(const btVector3& v1, const btVector3& v2) 
{
 return btVector3(v1.m_floats[0] + v2.m_floats[0], v1.m_floats[1] + v2.m_floats[1], v1.m_floats[2] + v2.m_floats[2]);
}

/**@brief Return the elementwise product of two vectors */
 btVector3 
operator*(const btVector3& v1, const btVector3& v2) 
{
 return btVector3(v1.m_floats[0] * v2.m_floats[0], v1.m_floats[1] * v2.m_floats[1], v1.m_floats[2] * v2.m_floats[2]);
}

/**@brief Return the difference between two vectors */
 btVector3 
operator-(const btVector3& v1, const btVector3& v2)
{
 return btVector3(v1.m_floats[0] - v2.m_floats[0], v1.m_floats[1] - v2.m_floats[1], v1.m_floats[2] - v2.m_floats[2]);
}
/**@brief Return the negative of the vector */
 btVector3 
operator-(const btVector3& v)
{
 return btVector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
}

/**@brief Return the vector scaled by s */
 btVector3 
operator*(const btVector3& v, const btScalar& s)
{
 return btVector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
}

/**@brief Return the vector scaled by s */
 btVector3 
operator*(const btScalar& s, const btVector3& v)
{ 
 return v * s; 
}

/**@brief Return the vector inversely scaled by s */
 btVector3
operator/(const btVector3& v, const btScalar& s)
{
 btFullAssert(s != btScalar(0.0));
 return v * (btScalar(1.0) / s);
}

/**@brief Return the vector inversely scaled by s */
 btVector3
operator/(const btVector3& v1, const btVector3& v2)
{
 return btVector3(v1.m_floats[0] / v2.m_floats[0],v1.m_floats[1] / v2.m_floats[1],v1.m_floats[2] / v2.m_floats[2]);
}

/**@brief Return the dot product between two vectors */
 btScalar 
btDot(const btVector3& v1, const btVector3& v2) 
{ 
 return v1.dot(v2); 
}


/**@brief Return the distance squared between two vectors */
 btScalar
btDistance2(const btVector3& v1, const btVector3& v2) 
{ 
 return v1.distance2(v2); 
}


/**@brief Return the distance between two vectors */
 btScalar
btDistance(const btVector3& v1, const btVector3& v2) 
{ 
 return v1.distance(v2); 
}

/**@brief Return the angle between two vectors */
 btScalar
btAngle(const btVector3& v1, const btVector3& v2) 
{ 
 return v1.angle(v2); 
}

/**@brief Return the cross product of two vectors */
 btVector3 
btCross(const btVector3& v1, const btVector3& v2) 
{ 
 return v1.cross(v2); 
}

 btScalar
btTriple(const btVector3& v1, const btVector3& v2, const btVector3& v3)
{
 return v1.triple(v2, v3);
}

/**@brief Return the linear interpolation between two vectors
 * @param v1 One vector 
 * @param v2 The other vector 
 * @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
 btVector3 
lerp(const btVector3& v1, const btVector3& v2, const btScalar& t)
{
 return v1.lerp(v2, t);
}



 btScalar btVector3::distance2(const btVector3& v) const
{
 return (v - *this).length2();
}

 btScalar btVector3::distance(const btVector3& v) const
{
 return (v - *this).length();
}

 btVector3 btVector3::normalized() const
{
 return *this / length();
} 

 btVector3 btVector3::rotate( const btVector3& wAxis, const btScalar angle ) const
{
 // wAxis must be a unit lenght vector

 btVector3 o = wAxis * wAxis.dot( *this );
 btVector3 x = *this - o;
 btVector3 y;

 y = wAxis.cross( *this );

 return ( o + x * btCos( angle ) + y * btSin( angle ) );
}

class btVector4 : public btVector3
{
public:

  btVector4() {}


  btVector4(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w) 
  : btVector3(x,y,z)
 {
  m_floats[3] = w;
 }


  btVector4 absolute4() const 
 {
  return btVector4(
   btFabs(m_floats[0]), 
   btFabs(m_floats[1]), 
   btFabs(m_floats[2]),
   btFabs(m_floats[3]));
 }



 btScalar getW() const { return m_floats[3];}


   int maxAxis4() const
 {
  int maxIndex = -1;
  btScalar maxVal = btScalar(-BT_LARGE_FLOAT);
  if (m_floats[0] > maxVal)
  {
   maxIndex = 0;
   maxVal = m_floats[0];
  }
  if (m_floats[1] > maxVal)
  {
   maxIndex = 1;
   maxVal = m_floats[1];
  }
  if (m_floats[2] > maxVal)
  {
   maxIndex = 2;
   maxVal =m_floats[2];
  }
  if (m_floats[3] > maxVal)
  {
   maxIndex = 3;
   maxVal = m_floats[3];
  }
  
  
  

  return maxIndex;

 }


  int minAxis4() const
 {
  int minIndex = -1;
  btScalar minVal = btScalar(BT_LARGE_FLOAT);
  if (m_floats[0] < minVal)
  {
   minIndex = 0;
   minVal = m_floats[0];
  }
  if (m_floats[1] < minVal)
  {
   minIndex = 1;
   minVal = m_floats[1];
  }
  if (m_floats[2] < minVal)
  {
   minIndex = 2;
   minVal =m_floats[2];
  }
  if (m_floats[3] < minVal)
  {
   minIndex = 3;
   minVal = m_floats[3];
  }
  
  return minIndex;

 }


  int closestAxis4() const 
 {
  return absolute4().maxAxis4();
 }

 
 

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
  

/*  void getValue(btScalar *m) const 
  {
   m[0] = m_floats[0];
   m[1] = m_floats[1];
   m[2] =m_floats[2];
  }
*/
/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
   void setValue(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w)
  {
   m_floats[0]=x;
   m_floats[1]=y;
   m_floats[2]=z;
   m_floats[3]=w;
  }


};


///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
 void btSwapScalarEndian(const btScalar& sourceVal, btScalar& destVal)
{
 #ifdef BT_USE_DOUBLE_PRECISION
 unsigned char* dest = (unsigned char*) &destVal;
 unsigned char* src  = (unsigned char*) &sourceVal;
 dest[0] = src[7];
    dest[1] = src[6];
    dest[2] = src[5];
    dest[3] = src[4];
    dest[4] = src[3];
    dest[5] = src[2];
    dest[6] = src[1];
    dest[7] = src[0];
#else
 unsigned char* dest = (unsigned char*) &destVal;
 unsigned char* src  = (unsigned char*) &sourceVal;
 dest[0] = src[3];
    dest[1] = src[2];
    dest[2] = src[1];
    dest[3] = src[0];
#endif //BT_USE_DOUBLE_PRECISION
}
///btSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
 void btSwapVector3Endian(const btVector3& sourceVec, btVector3& destVec)
{
 for (int i=0;i<4;i++)
 {
  btSwapScalarEndian(sourceVec[i],destVec[i]);
 }

}

///btUnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
 void btUnSwapVector3Endian(btVector3& vector)
{

 btVector3 swappedVec;
 for (int i=0;i<4;i++)
 {
  btSwapScalarEndian(vector[i],swappedVec[i]);
 }
 vector = swappedVec;
}

template <class T>
 void btPlaneSpace1 (const T& n, T& p, T& q)
{
  if (btFabs(n[2]) > SIMDSQRT12) {
    // choose p in y-z plane
    btScalar a = n[1]*n[1] + n[2]*n[2];
    btScalar k = btRecipSqrt (a);
    p[0] = 0;
 p[1] = -n[2]*k;
 p[2] = n[1]*k;
    // set q = n x p
    q[0] = a*k;
 q[1] = -n[0]*p[2];
 q[2] = n[0]*p[1];
  }
  else {
    // choose p in x-y plane
    btScalar a = n[0]*n[0] + n[1]*n[1];
    btScalar k = btRecipSqrt (a);
    p[0] = -n[1]*k;
 p[1] = n[0]*k;
 p[2] = 0;
    // set q = n x p
    q[0] = -n[2]*p[1];
 q[1] = n[2]*p[0];
 q[2] = a*k;
  }
}


class btVector3FloatData
{ public: 
 float m_floats[4];
};

class btVector3DoubleData
{ public: 
 double m_floats[4];

};

 void btVector3::serializeFloat(struct btVector3FloatData& dataOut) const
{
 ///could also do a memcpy, check if it is worth it
 for (int i=0;i<4;i++)
  dataOut.m_floats[i] = float(m_floats[i]);
}

 void btVector3::deSerializeFloat(const struct btVector3FloatData& dataIn)
{
 for (int i=0;i<4;i++)
  m_floats[i] = btScalar(dataIn.m_floats[i]);
}


 void btVector3::serializeDouble(struct btVector3DoubleData& dataOut) const
{
 ///could also do a memcpy, check if it is worth it
 for (int i=0;i<4;i++)
  dataOut.m_floats[i] = double(m_floats[i]);
}

 void btVector3::deSerializeDouble(const struct btVector3DoubleData& dataIn)
{
 for (int i=0;i<4;i++)
  m_floats[i] = btScalar(dataIn.m_floats[i]);
}


 void btVector3::serialize(struct btVector3Data& dataOut) const
{
 ///could also do a memcpy, check if it is worth it
 for (int i=0;i<4;i++)
  dataOut.m_floats[i] = m_floats[i];
}

 void btVector3::deSerialize(const struct btVector3Data& dataIn)
{
 for (int i=0;i<4;i++)
  m_floats[i] = dataIn.m_floats[i];
}


#endif //BT_VECTOR3_H
//// ../src/LinearMath/btScalar.h
/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_SCALAR_H
#define BT_SCALAR_H

#ifdef BT_MANAGED_CODE
//Aligned data types not supported in managed code
#pragma unmanaged
#endif


#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <float.h>

/* SVN $Revision$ on $Date$ from http://bullet.googlecode.com*/
#define BT_BULLET_VERSION 278

inline int btGetVersion()
{
 return BT_BULLET_VERSION;
}

#if defined(DEBUG) || defined (_DEBUG)
#define BT_DEBUG
#endif


#ifdef _WIN32

  #if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)

   #define  inline
   #define ATTRIBUTE_ALIGNED16(a) a
   #define ATTRIBUTE_ALIGNED64(a) a
   #define ATTRIBUTE_ALIGNED128(a) a
  #else
   //#define BT_HAS_ALIGNED_ALLOCATOR
   #pragma warning(disable : 4324) // disable padding warning
//   #pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
//   #pragma warning(disable:4996) //Turn off warnings about deprecated C routines
//   #pragma warning(disable:4786) // Disable the "debug name too long" warning

   #define  __forceinline
   #define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
   #define ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
   #define ATTRIBUTE_ALIGNED128(a) __declspec (align(128)) a
  #ifdef _XBOX
   #define BT_USE_VMX128

   #include <ppcintrinsics.h>
    #define BT_HAVE_NATIVE_FSEL
    #define btFsel(a,b,c) __fsel((a),(b),(c))
  #else

#if (defined (_WIN32) && (_MSC_VER) && _MSC_VER >= 1400) && (!defined (BT_USE_DOUBLE_PRECISION))
   #define BT_USE_SSE
   #include <emmintrin.h>
#endif

  #endif//_XBOX

  #endif //__MINGW32__

  #include <assert.h>
#ifdef BT_DEBUG
  #define btAssert assert
#else
  #define btAssert(x)
#endif
  //btFullAssert is optional, slows down a lot
  #define btFullAssert(x)

  #define btLikely(_c)  _c
  #define btUnlikely(_c) _c

#else
 
#if defined (__CELLOS_LV2__)
  #define  inline __attribute__((always_inline))
  #define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
  #define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
  #define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
  #ifndef assert
  #include <assert.h>
  #endif
#ifdef BT_DEBUG
#ifdef __SPU__
#include <spu_printf.h>
#define printf spu_printf
 #define btAssert(x) {if(!(x)){printf("Assert "__FILE__ ":%u ("#x")\n", __LINE__);spu_hcmpeq(0,0);}}
#else
 #define btAssert assert
#endif
 
#else
  #define btAssert(x)
#endif
  //btFullAssert is optional, slows down a lot
  #define btFullAssert(x)

  #define btLikely(_c)  _c
  #define btUnlikely(_c) _c

#else

#ifdef USE_LIBSPE2

  #define  __inline
  #define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
  #define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
  #define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
  #ifndef assert
  #include <assert.h>
  #endif
#ifdef BT_DEBUG
  #define btAssert assert
#else
  #define btAssert(x)
#endif
  //btFullAssert is optional, slows down a lot
  #define btFullAssert(x)


  #define btLikely(_c)   __builtin_expect((_c), 1)
  #define btUnlikely(_c) __builtin_expect((_c), 0)
  

#else
 //non-windows systems

#if (defined (__APPLE__) && defined (__i386__) && (!defined (BT_USE_DOUBLE_PRECISION)))
 #define BT_USE_SSE
 #include <emmintrin.h>

 #define  inline
///@todo: check out alignment methods for other platforms/compilers
 #define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
 #define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
 #define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
 #ifndef assert
 #include <assert.h>
 #endif

 #if defined(DEBUG) || defined (_DEBUG)
  #define btAssert assert
 #else
  #define btAssert(x)
 #endif

 //btFullAssert is optional, slows down a lot
 #define btFullAssert(x)
 #define btLikely(_c)  _c
 #define btUnlikely(_c) _c

#else

  #define  inline
  ///@todo: check out alignment methods for other platforms/compilers
  ///#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
  ///#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
  ///#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
  #define ATTRIBUTE_ALIGNED16(a) a
  #define ATTRIBUTE_ALIGNED64(a) a
  #define ATTRIBUTE_ALIGNED128(a) a
  #ifndef assert
  #include <assert.h>
  #endif

#if defined(DEBUG) || defined (_DEBUG)
  #define btAssert assert
#else
  #define btAssert(x)
#endif

  //btFullAssert is optional, slows down a lot
  #define btFullAssert(x)
  #define btLikely(_c)  _c
  #define btUnlikely(_c) _c
#endif //__APPLE__ 

#endif // LIBSPE2

#endif //__CELLOS_LV2__
#endif


///The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
#if defined(BT_USE_DOUBLE_PRECISION)
typedef double btScalar;
//this number could be bigger in double precision
#define BT_LARGE_FLOAT 1e30
#else
typedef float btScalar;
//keep BT_LARGE_FLOAT*BT_LARGE_FLOAT < FLT_MAX
#define BT_LARGE_FLOAT 1e18f
#endif



#define BT_DECLARE_ALIGNED_ALLOCATOR() \
    void* operator new(size_t sizeInBytes)   { return btAlignedAlloc(sizeInBytes,16); }   \
    void  operator delete(void* ptr)         { btAlignedFree(ptr); }   \
    void* operator new(size_t, void* ptr)   { return ptr; }   \
    void  operator delete(void*, void*)      { }   \
    void* operator new[](size_t sizeInBytes)   { return btAlignedAlloc(sizeInBytes,16); }   \
    void  operator delete[](void* ptr)         { btAlignedFree(ptr); }   \
    void* operator new[](size_t, void* ptr)   { return ptr; }   \
    void  operator delete[](void*, void*)      { }   \



#if defined(BT_USE_DOUBLE_PRECISION) || defined(BT_FORCE_DOUBLE_FUNCTIONS)
  
 btScalar btSqrt(btScalar x) { return sqrt(x); }
 btScalar btFabs(btScalar x) { return fabs(x); }
 btScalar btCos(btScalar x) { return cos(x); }
 btScalar btSin(btScalar x) { return sin(x); }
 btScalar btTan(btScalar x) { return tan(x); }
 btScalar btAcos(btScalar x) { if (x<btScalar(-1)) x=btScalar(-1); if (x>btScalar(1)) x=btScalar(1); return acos(x); }
 btScalar btAsin(btScalar x) { if (x<btScalar(-1)) x=btScalar(-1); if (x>btScalar(1)) x=btScalar(1); return asin(x); }
 btScalar btAtan(btScalar x) { return atan(x); }
 btScalar btAtan2(btScalar x, btScalar y) { return atan2(x, y); }
 btScalar btExp(btScalar x) { return exp(x); }
 btScalar btLog(btScalar x) { return log(x); }
 btScalar btPow(btScalar x,btScalar y) { return pow(x,y); }
 btScalar btFmod(btScalar x,btScalar y) { return fmod(x,y); }

#else
  
 btScalar btSqrt(btScalar y) 
{ 
#ifdef USE_APPROXIMATION
    double x, z, tempf;
    unsigned long *tfptr = ((unsigned long *)&tempf) + 1;

 tempf = y;
 *tfptr = (0xbfcdd90a - *tfptr)>>1; /* estimate of 1/sqrt(y) */
 x =  tempf;
 z =  y*btScalar(0.5);
 x = (btScalar(1.5)*x)-(x*x)*(x*z);         /* iteration formula     */
 x = (btScalar(1.5)*x)-(x*x)*(x*z);
 x = (btScalar(1.5)*x)-(x*x)*(x*z);
 x = (btScalar(1.5)*x)-(x*x)*(x*z);
 x = (btScalar(1.5)*x)-(x*x)*(x*z);
 return x*y;
#else
 return sqrtf(y); 
#endif
}
 btScalar btFabs(btScalar x) { return fabsf(x); }
 btScalar btCos(btScalar x) { return cosf(x); }
 btScalar btSin(btScalar x) { return sinf(x); }
 btScalar btTan(btScalar x) { return tanf(x); }
 btScalar btAcos(btScalar x) { 
 if (x<btScalar(-1)) 
  x=btScalar(-1); 
 if (x>btScalar(1)) 
  x=btScalar(1);
 return acosf(x); 
}
 btScalar btAsin(btScalar x) { 
 if (x<btScalar(-1)) 
  x=btScalar(-1); 
 if (x>btScalar(1)) 
  x=btScalar(1);
 return asinf(x); 
}
 btScalar btAtan(btScalar x) { return atanf(x); }
 btScalar btAtan2(btScalar x, btScalar y) { return atan2f(x, y); }
 btScalar btExp(btScalar x) { return expf(x); }
 btScalar btLog(btScalar x) { return logf(x); }
 btScalar btPow(btScalar x,btScalar y) { return powf(x,y); }
 btScalar btFmod(btScalar x,btScalar y) { return fmodf(x,y); }
 
#endif

#define SIMD_2_PI         btScalar(6.283185307179586232)
#define SIMD_PI           (SIMD_2_PI * btScalar(0.5))
#define SIMD_HALF_PI      (SIMD_2_PI * btScalar(0.25))
#define SIMD_RADS_PER_DEG (SIMD_2_PI / btScalar(360.0))
#define SIMD_DEGS_PER_RAD  (btScalar(360.0) / SIMD_2_PI)
#define SIMDSQRT12 btScalar(0.7071067811865475244008443621048490)

#define btRecipSqrt(x) ((btScalar)(btScalar(1.0)/btSqrt(btScalar(x))))  /* reciprocal square root */


#ifdef BT_USE_DOUBLE_PRECISION
#define SIMD_EPSILON      DBL_EPSILON
#define SIMD_INFINITY     DBL_MAX
#else
#define SIMD_EPSILON      FLT_EPSILON
#define SIMD_INFINITY     FLT_MAX
#endif

 btScalar btAtan2Fast(btScalar y, btScalar x) 
{
 btScalar coeff_1 = SIMD_PI / 4.0f;
 btScalar coeff_2 = 3.0f * coeff_1;
 btScalar abs_y = btFabs(y);
 btScalar angle;
 if (x >= 0.0f) {
  btScalar r = (x - abs_y) / (x + abs_y);
  angle = coeff_1 - coeff_1 * r;
 } else {
  btScalar r = (x + abs_y) / (abs_y - x);
  angle = coeff_2 - coeff_1 * r;
 }
 return (y < 0.0f) ? -angle : angle;
}

 bool      btFuzzyZero(btScalar x) { return btFabs(x) < SIMD_EPSILON; }

 bool btEqual(btScalar a, btScalar eps) {
 return (((a) <= eps) && !((a) < -eps));
}
 bool btGreaterEqual (btScalar a, btScalar eps) {
 return (!((a) <= eps));
}


 int       btIsNegative(btScalar x) {
    return x < btScalar(0.0) ? 1 : 0;
}

 btScalar btRadians(btScalar x) { return x * SIMD_RADS_PER_DEG; }
 btScalar btDegrees(btScalar x) { return x * SIMD_DEGS_PER_RAD; }

#define BT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifndef btFsel
 btScalar btFsel(btScalar a, btScalar b, btScalar c)
{
 return a >= 0 ? b : c;
}
#endif
#define btFsels(a,b,c) (btScalar)btFsel(a,b,c)


 bool btMachineIsLittleEndian()
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
    return true;
   else
    return false;
}



///btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
 unsigned btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero) 
{
    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // and zero value or'ed with its negative (which is still zero) having sign bit off 
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz;
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz)); 
}
 int btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz; 
    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
 float btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef BT_HAVE_NATIVE_FSEL
    return (float)btFsel((btScalar)condition - btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero; 
#endif
}

template<typename T>  void btSwap(T& a, T& b)
{
 T tmp = a;
 a = b;
 b = tmp;
}


//PCK: endian swapping functions
 unsigned btSwapEndian(unsigned val)
{
 return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8)  | ((val & 0x000000ff) << 24));
}

 unsigned short btSwapEndian(unsigned short val)
{
 return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

 unsigned btSwapEndian(int val)
{
 return btSwapEndian((unsigned)val);
}

 unsigned short btSwapEndian(short val)
{
 return btSwapEndian((unsigned short) val);
}

///btSwapFloat uses using char pointers to swap the endianness
////btSwapFloat/btSwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754. 
///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception. 
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you. 
///so instead of returning a float/double, we return integer/long long integer
 unsigned int  btSwapEndianFloat(float d)
{
    unsigned int a = 0;
    unsigned char *dst = (unsigned char *)&a;
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];
    return a;
}

// unswap using char pointers
 float btUnswapEndianFloat(unsigned int a) 
{
    float d = 0.0f;
    unsigned char *src = (unsigned char *)&a;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];

    return d;
}


// swap using char pointers
 void  btSwapEndianDouble(double d, unsigned char* dst)
{
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

}

// unswap using char pointers
 double btUnswapEndianDouble(const unsigned char *src) 
{
    double d = 0.0;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

 return d;
}

// returns normalized value in range [-SIMD_PI, SIMD_PI]
 btScalar btNormalizeAngle(btScalar angleInRadians) 
{
 angleInRadians = btFmod(angleInRadians, SIMD_2_PI);
 if(angleInRadians < -SIMD_PI)
 {
  return angleInRadians + SIMD_2_PI;
 }
 else if(angleInRadians > SIMD_PI)
 {
  return angleInRadians - SIMD_2_PI;
 }
 else
 {
  return angleInRadians;
 }
}

///rudimentary class to provide type info
class btTypedObject
{ public: 
 btTypedObject(int objectType)
  :m_objectType(objectType)
 {
 }
 int m_objectType;
 inline int getObjectType() const
 {
  return m_objectType;
 }
};
#endif //BT_SCALAR_H
//// ../src/LinearMath/btMinMax.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_GEN_MINMAX_H
#define BT_GEN_MINMAX_H

#include "LinearMath/btScalar.h"

template <class T>
 const T& btMin(const T& a, const T& b) 
{
  return a < b ? a : b ;
}

template <class T>
 const T& btMax(const T& a, const T& b) 
{
  return  a > b ? a : b;
}

template <class T>
 const T& btClamped(const T& a, const T& lb, const T& ub) 
{
 return a < lb ? lb : (ub < a ? ub : a); 
}

template <class T>
 void btSetMin(T& a, const T& b) 
{
    if (b < a) 
 {
  a = b;
 }
}

template <class T>
 void btSetMax(T& a, const T& b) 
{
    if (a < b) 
 {
  a = b;
 }
}

template <class T>
 void btClamp(T& a, const T& lb, const T& ub) 
{
 if (a < lb) 
 {
  a = lb; 
 }
 else if (ub < a) 
 {
  a = ub;
 }
}

#endif //BT_GEN_MINMAX_H
//// ../src/LinearMath/btTransform.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_TRANSFORM_H
#define BT_TRANSFORM_H


#include "btMatrix3x3.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btTransformData btTransformDoubleData
#else
#define btTransformData btTransformFloatData
#endif




/**@brief The btTransform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *It can be used in combination with btVector3, btQuaternion and btMatrix3x3 linear algebra classes. */
class btTransform {
 
  ///Storage for the rotation
 btMatrix3x3 m_basis;
  ///Storage for the translation
 btVector3   m_origin;

public:
 
  /**@brief No initialization constructor */
 btTransform() {}
  /**@brief Constructor from btQuaternion (optional btVector3 )
   * @param q Rotation from quaternion 
   * @param c Translation from Vector (default 0,0,0) */
 explicit  btTransform(const btQuaternion& q, 
  const btVector3& c = btVector3(btScalar(0), btScalar(0), btScalar(0))) 
  : m_basis(q),
  m_origin(c)
 {}

  /**@brief Constructor from btMatrix3x3 (optional btVector3)
   * @param b Rotation from Matrix 
   * @param c Translation from Vector default (0,0,0)*/
 explicit  btTransform(const btMatrix3x3& b, 
  const btVector3& c = btVector3(btScalar(0), btScalar(0), btScalar(0)))
  : m_basis(b),
  m_origin(c)
 {}
  /**@brief Copy constructor */
  btTransform (const btTransform& other)
  : m_basis(other.m_basis),
  m_origin(other.m_origin)
 {
 }
  /**@brief Assignment Operator */
  btTransform& operator=(const btTransform& other)
 {
  m_basis = other.m_basis;
  m_origin = other.m_origin;
  return *this;
 }


  /**@brief Set the current transform as the value of the product of two transforms
   * @param t1 Transform 1
   * @param t2 Transform 2
   * This = Transform1 * Transform2 */
   void mult(const btTransform& t1, const btTransform& t2) {
   m_basis = t1.m_basis * t2.m_basis;
   m_origin = t1(t2.m_origin);
  }

/*  void multInverseLeft(const btTransform& t1, const btTransform& t2) {
   btVector3 v = t2.m_origin - t1.m_origin;
   m_basis = btMultTransposeLeft(t1.m_basis, t2.m_basis);
   m_origin = v * t1.m_basis;
  }
  */

/**@brief Return the transform of the vector */
  btVector3 operator()(const btVector3& x) const
 {
  return btVector3(m_basis[0].dot(x) + m_origin.x(), 
   m_basis[1].dot(x) + m_origin.y(), 
   m_basis[2].dot(x) + m_origin.z());
 }

  /**@brief Return the transform of the vector */
  btVector3 operator*(const btVector3& x) const
 {
  return (*this)(x);
 }

  /**@brief Return the transform of the btQuaternion */
  btQuaternion operator*(const btQuaternion& q) const
 {
  return getRotation() * q;
 }

  /**@brief Return the basis matrix for the rotation */
  btMatrix3x3&       getBasis()          { return m_basis; }
  /**@brief Return the basis matrix for the rotation */
  const btMatrix3x3& getBasis()    const { return m_basis; }

  /**@brief Return the origin vector translation */
  btVector3&         getOrigin()         { return m_origin; }
  /**@brief Return the origin vector translation */
  const btVector3&   getOrigin()   const { return m_origin; }

  /**@brief Return a quaternion representing the rotation */
 btQuaternion getRotation() const { 
  btQuaternion q;
  m_basis.getRotation(q);
  return q;
 }
 
 
  /**@brief Set from an array 
   * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
 void setFromOpenGLMatrix(const btScalar *m)
 {
  m_basis.setFromOpenGLSubMatrix(m);
  m_origin.setValue(m[12],m[13],m[14]);
 }

  /**@brief Fill an array representation
   * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
 void getOpenGLMatrix(btScalar *m) const 
 {
  m_basis.getOpenGLSubMatrix(m);
  m[12] = m_origin.x();
  m[13] = m_origin.y();
  m[14] = m_origin.z();
  m[15] = btScalar(1.0);
 }

  /**@brief Set the translational element
   * @param origin The vector to set the translation to */
  void setOrigin(const btVector3& origin) 
 { 
  m_origin = origin;
 }

  btVector3 invXform(const btVector3& inVec) const;


  /**@brief Set the rotational element by btMatrix3x3 */
  void setBasis(const btMatrix3x3& basis)
 { 
  m_basis = basis;
 }

  /**@brief Set the rotational element by btQuaternion */
  void setRotation(const btQuaternion& q)
 {
  m_basis.setRotation(q);
 }


  /**@brief Set this transformation to the identity */
 void setIdentity()
 {
  m_basis.setIdentity();
  m_origin.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
 }

  /**@brief Multiply this Transform by another(this = this * another) 
   * @param t The other transform */
 btTransform& operator*=(const btTransform& t) 
 {
  m_origin += m_basis * t.m_origin;
  m_basis *= t.m_basis;
  return *this;
 }

  /**@brief Return the inverse of this transform */
 btTransform inverse() const
 { 
  btMatrix3x3 inv = m_basis.transpose();
  return btTransform(inv, inv * -m_origin);
 }

  /**@brief Return the inverse of this transform times the other transform
   * @param t The other transform 
   * return this.inverse() * the other */
 btTransform inverseTimes(const btTransform& t) const;  

  /**@brief Return the product of this transform and the other */
 btTransform operator*(const btTransform& t) const;

  /**@brief Return an identity transform */
 static const btTransform& getIdentity()
 {
  static const btTransform identityTransform(btMatrix3x3::getIdentity());
  return identityTransform;
 }

 void serialize(struct btTransformData& dataOut) const;

 void serializeFloat(struct btTransformFloatData& dataOut) const;

 void deSerialize(const struct btTransformData& dataIn);

 void deSerializeDouble(const struct btTransformDoubleData& dataIn);

 void deSerializeFloat(const struct btTransformFloatData& dataIn);

};


 btVector3
btTransform::invXform(const btVector3& inVec) const
{
 btVector3 v = inVec - m_origin;
 return (m_basis.transpose() * v);
}

 btTransform 
btTransform::inverseTimes(const btTransform& t) const  
{
 btVector3 v = t.getOrigin() - m_origin;
  return btTransform(m_basis.transposeTimes(t.m_basis),
   v * m_basis);
}

 btTransform 
btTransform::operator*(const btTransform& t) const
{
 return btTransform(m_basis * t.m_basis, 
  (*this)(t.m_origin));
}

/**@brief Test if two transforms have all elements equal */
 bool operator==(const btTransform& t1, const btTransform& t2)
{
   return ( t1.getBasis()  == t2.getBasis() &&
            t1.getOrigin() == t2.getOrigin() );
}


///for serialization
class btTransformFloatData
{ public: 
 btMatrix3x3FloatData m_basis;
 btVector3FloatData m_origin;
};

class btTransformDoubleData
{ public: 
 btMatrix3x3DoubleData m_basis;
 btVector3DoubleData m_origin;
};



 void btTransform::serialize(btTransformData& dataOut) const
{
 m_basis.serialize(dataOut.m_basis);
 m_origin.serialize(dataOut.m_origin);
}

 void btTransform::serializeFloat(btTransformFloatData& dataOut) const
{
 m_basis.serializeFloat(dataOut.m_basis);
 m_origin.serializeFloat(dataOut.m_origin);
}


 void btTransform::deSerialize(const btTransformData& dataIn)
{
 m_basis.deSerialize(dataIn.m_basis);
 m_origin.deSerialize(dataIn.m_origin);
}

 void btTransform::deSerializeFloat(const btTransformFloatData& dataIn)
{
 m_basis.deSerializeFloat(dataIn.m_basis);
 m_origin.deSerializeFloat(dataIn.m_origin);
}

 void btTransform::deSerializeDouble(const btTransformDoubleData& dataIn)
{
 m_basis.deSerializeDouble(dataIn.m_basis);
 m_origin.deSerializeDouble(dataIn.m_origin);
}


#endif //BT_TRANSFORM_H






//// ../src/LinearMath/btMatrix3x3.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_MATRIX3x3_H
#define BT_MATRIX3x3_H

#include "btVector3.h"
#include "btQuaternion.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btMatrix3x3Data btMatrix3x3DoubleData 
#else
#define btMatrix3x3Data btMatrix3x3FloatData
#endif //BT_USE_DOUBLE_PRECISION


/**@brief The btMatrix3x3 class implements a 3x3 rotation matrix, to perform linear algebra in combination with btQuaternion, btTransform and btVector3.
* Make sure to only include a pure orthogonal matrix without scaling. */
class btMatrix3x3 {

 ///Data storage for the matrix, each vector is a row of the matrix
 btVector3 m_el[3];

public:
 /** @brief No initializaion constructor */
 btMatrix3x3 () {}

 //  explicit btMatrix3x3(const btScalar *m) { setFromOpenGLSubMatrix(m); }

 /**@brief Constructor from Quaternion */
 explicit btMatrix3x3(const btQuaternion& q) { setRotation(q); }
 /*
 template <typename btScalar>
 Matrix3x3(const btScalar& yaw, const btScalar& pitch, const btScalar& roll)
 { 
 setEulerYPR(yaw, pitch, roll);
 }
 */
 /** @brief Constructor with row major formatting */
 btMatrix3x3(const btScalar& xx, const btScalar& xy, const btScalar& xz,
  const btScalar& yx, const btScalar& yy, const btScalar& yz,
  const btScalar& zx, const btScalar& zy, const btScalar& zz)
 { 
  setValue(xx, xy, xz, 
   yx, yy, yz, 
   zx, zy, zz);
 }
 /** @brief Copy constructor */
  btMatrix3x3 (const btMatrix3x3& other)
 {
  m_el[0] = other.m_el[0];
  m_el[1] = other.m_el[1];
  m_el[2] = other.m_el[2];
 }
 /** @brief Assignment Operator */
  btMatrix3x3& operator=(const btMatrix3x3& other)
 {
  m_el[0] = other.m_el[0];
  m_el[1] = other.m_el[1];
  m_el[2] = other.m_el[2];
  return *this;
 }

 /** @brief Get a column of the matrix as a vector 
 *  @param i Column number 0 indexed */
  btVector3 getColumn(int i) const
 {
  return btVector3(m_el[0][i],m_el[1][i],m_el[2][i]);
 }


 /** @brief Get a row of the matrix as a vector 
 *  @param i Row number 0 indexed */
  const btVector3& getRow(int i) const
 {
  btFullAssert(0 <= i && i < 3);
  return m_el[i];
 }

 /** @brief Get a mutable reference to a row of the matrix as a vector 
 *  @param i Row number 0 indexed */
  btVector3&  operator[](int i)
 { 
  btFullAssert(0 <= i && i < 3);
  return m_el[i]; 
 }

 /** @brief Get a const reference to a row of the matrix as a vector 
 *  @param i Row number 0 indexed */
  const btVector3& operator[](int i) const
 {
  btFullAssert(0 <= i && i < 3);
  return m_el[i]; 
 }

 /** @brief Multiply by the target matrix on the right
 *  @param m Rotation matrix to be applied 
 * Equivilant to this = this * m */
 btMatrix3x3& operator*=(const btMatrix3x3& m); 

 /** @brief Adds by the target matrix on the right
 *  @param m matrix to be applied 
 * Equivilant to this = this + m */
 btMatrix3x3& operator+=(const btMatrix3x3& m); 

 /** @brief Substractss by the target matrix on the right
 *  @param m matrix to be applied 
 * Equivilant to this = this - m */
 btMatrix3x3& operator-=(const btMatrix3x3& m); 

 /** @brief Set from the rotational part of a 4x4 OpenGL matrix
 *  @param m A pointer to the beginning of the array of scalars*/
 void setFromOpenGLSubMatrix(const btScalar *m)
 {
  m_el[0].setValue(m[0],m[4],m[8]);
  m_el[1].setValue(m[1],m[5],m[9]);
  m_el[2].setValue(m[2],m[6],m[10]);

 }
 /** @brief Set the values of the matrix explicitly (row major)
 *  @param xx Top left
 *  @param xy Top Middle
 *  @param xz Top Right
 *  @param yx Middle Left
 *  @param yy Middle Middle
 *  @param yz Middle Right
 *  @param zx Bottom Left
 *  @param zy Bottom Middle
 *  @param zz Bottom Right*/
 void setValue(const btScalar& xx, const btScalar& xy, const btScalar& xz, 
  const btScalar& yx, const btScalar& yy, const btScalar& yz, 
  const btScalar& zx, const btScalar& zy, const btScalar& zz)
 {
  m_el[0].setValue(xx,xy,xz);
  m_el[1].setValue(yx,yy,yz);
  m_el[2].setValue(zx,zy,zz);
 }

 /** @brief Set the matrix from a quaternion
 *  @param q The Quaternion to match */  
 void setRotation(const btQuaternion& q) 
 {
  btScalar d = q.length2();
  btFullAssert(d != btScalar(0.0));
  btScalar s = btScalar(2.0) / d;
  btScalar xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
  btScalar wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
  btScalar xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
  btScalar yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;
  setValue(btScalar(1.0) - (yy + zz), xy - wz, xz + wy,
   xy + wz, btScalar(1.0) - (xx + zz), yz - wx,
   xz - wy, yz + wx, btScalar(1.0) - (xx + yy));
 }


 /** @brief Set the matrix from euler angles using YPR around YXZ respectively
 *  @param yaw Yaw about Y axis
 *  @param pitch Pitch about X axis
 *  @param roll Roll about Z axis 
 */
 void setEulerYPR(const btScalar& yaw, const btScalar& pitch, const btScalar& roll) 
 {
  setEulerZYX(roll, pitch, yaw);
 }

 /** @brief Set the matrix from euler angles YPR around ZYX axes
 * @param eulerX Roll about X axis
 * @param eulerY Pitch around Y axis
 * @param eulerZ Yaw aboud Z axis
 * 
 * These angles are used to produce a rotation matrix. The euler
 * angles are applied in ZYX order. I.e a vector is first rotated 
 * about X then Y and then Z
 **/
 void setEulerZYX(btScalar eulerX,btScalar eulerY,btScalar eulerZ) { 
  ///@todo proposed to reverse this since it's labeled zyx but takes arguments xyz and it will match all other parts of the code
  btScalar ci ( btCos(eulerX)); 
  btScalar cj ( btCos(eulerY)); 
  btScalar ch ( btCos(eulerZ)); 
  btScalar si ( btSin(eulerX)); 
  btScalar sj ( btSin(eulerY)); 
  btScalar sh ( btSin(eulerZ)); 
  btScalar cc = ci * ch; 
  btScalar cs = ci * sh; 
  btScalar sc = si * ch; 
  btScalar ss = si * sh;

  setValue(cj * ch, sj * sc - cs, sj * cc + ss,
   cj * sh, sj * ss + cc, sj * cs - sc, 
   -sj,      cj * si,      cj * ci);
 }

 /**@brief Set the matrix to the identity */
 void setIdentity()
 { 
  setValue(btScalar(1.0), btScalar(0.0), btScalar(0.0), 
   btScalar(0.0), btScalar(1.0), btScalar(0.0), 
   btScalar(0.0), btScalar(0.0), btScalar(1.0)); 
 }

 static const btMatrix3x3& getIdentity()
 {
  static const btMatrix3x3 identityMatrix(btScalar(1.0), btScalar(0.0), btScalar(0.0), 
   btScalar(0.0), btScalar(1.0), btScalar(0.0), 
   btScalar(0.0), btScalar(0.0), btScalar(1.0));
  return identityMatrix;
 }

 /**@brief Fill the rotational part of an OpenGL matrix and clear the shear/perspective
 * @param m The array to be filled */
 void getOpenGLSubMatrix(btScalar *m) const 
 {
  m[0]  = btScalar(m_el[0].x()); 
  m[1]  = btScalar(m_el[1].x());
  m[2]  = btScalar(m_el[2].x());
  m[3]  = btScalar(0.0); 
  m[4]  = btScalar(m_el[0].y());
  m[5]  = btScalar(m_el[1].y());
  m[6]  = btScalar(m_el[2].y());
  m[7]  = btScalar(0.0); 
  m[8]  = btScalar(m_el[0].z()); 
  m[9]  = btScalar(m_el[1].z());
  m[10] = btScalar(m_el[2].z());
  m[11] = btScalar(0.0); 
 }

 /**@brief Get the matrix represented as a quaternion 
 * @param q The quaternion which will be set */
 void getRotation(btQuaternion& q) const
 {
  btScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();
  btScalar temp[4];

  if (trace > btScalar(0.0)) 
  {
   btScalar s = btSqrt(trace + btScalar(1.0));
   temp[3]=(s * btScalar(0.5));
   s = btScalar(0.5) / s;

   temp[0]=((m_el[2].y() - m_el[1].z()) * s);
   temp[1]=((m_el[0].z() - m_el[2].x()) * s);
   temp[2]=((m_el[1].x() - m_el[0].y()) * s);
  } 
  else 
  {
   int i = m_el[0].x() < m_el[1].y() ? 
    (m_el[1].y() < m_el[2].z() ? 2 : 1) :
    (m_el[0].x() < m_el[2].z() ? 2 : 0); 
   int j = (i + 1) % 3;  
   int k = (i + 2) % 3;

   btScalar s = btSqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + btScalar(1.0));
   temp[i] = s * btScalar(0.5);
   s = btScalar(0.5) / s;

   temp[3] = (m_el[k][j] - m_el[j][k]) * s;
   temp[j] = (m_el[j][i] + m_el[i][j]) * s;
   temp[k] = (m_el[k][i] + m_el[i][k]) * s;
  }
  q.setValue(temp[0],temp[1],temp[2],temp[3]);
 }

 /**@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
 * @param yaw Yaw around Y axis
 * @param pitch Pitch around X axis
 * @param roll around Z axis */ 
 void getEulerYPR(btScalar& yaw, btScalar& pitch, btScalar& roll) const
 {

  // first use the normal calculus
  yaw = btScalar(btAtan2(m_el[1].x(), m_el[0].x()));
  pitch = btScalar(btAsin(-m_el[2].x()));
  roll = btScalar(btAtan2(m_el[2].y(), m_el[2].z()));

  // on pitch = +/-HalfPI
  if (btFabs(pitch)==SIMD_HALF_PI)
  {
   if (yaw>0)
    yaw-=SIMD_PI;
   else
    yaw+=SIMD_PI;

   if (roll>0)
    roll-=SIMD_PI;
   else
    roll+=SIMD_PI;
  }
 };


 /**@brief Get the matrix represented as euler angles around ZYX
 * @param yaw Yaw around X axis
 * @param pitch Pitch around Y axis
 * @param roll around X axis 
 * @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/ 
 void getEulerZYX(btScalar& yaw, btScalar& pitch, btScalar& roll, unsigned int solution_number = 1) const
 {
  struct Euler
  {
   btScalar yaw;
   btScalar pitch;
   btScalar roll;
  };

  Euler euler_out;
  Euler euler_out2; //second solution
  //get the pointer to the raw data

  // Check that pitch is not at a singularity
  if (btFabs(m_el[2].x()) >= 1)
  {
   euler_out.yaw = 0;
   euler_out2.yaw = 0;

   // From difference of angles formula
   btScalar delta = btAtan2(m_el[0].x(),m_el[0].z());
   if (m_el[2].x() > 0)  //gimbal locked up
   {
    euler_out.pitch = SIMD_PI / btScalar(2.0);
    euler_out2.pitch = SIMD_PI / btScalar(2.0);
    euler_out.roll = euler_out.pitch + delta;
    euler_out2.roll = euler_out.pitch + delta;
   }
   else // gimbal locked down
   {
    euler_out.pitch = -SIMD_PI / btScalar(2.0);
    euler_out2.pitch = -SIMD_PI / btScalar(2.0);
    euler_out.roll = -euler_out.pitch + delta;
    euler_out2.roll = -euler_out.pitch + delta;
   }
  }
  else
  {
   euler_out.pitch = - btAsin(m_el[2].x());
   euler_out2.pitch = SIMD_PI - euler_out.pitch;

   euler_out.roll = btAtan2(m_el[2].y()/btCos(euler_out.pitch), 
    m_el[2].z()/btCos(euler_out.pitch));
   euler_out2.roll = btAtan2(m_el[2].y()/btCos(euler_out2.pitch), 
    m_el[2].z()/btCos(euler_out2.pitch));

   euler_out.yaw = btAtan2(m_el[1].x()/btCos(euler_out.pitch), 
    m_el[0].x()/btCos(euler_out.pitch));
   euler_out2.yaw = btAtan2(m_el[1].x()/btCos(euler_out2.pitch), 
    m_el[0].x()/btCos(euler_out2.pitch));
  }

  if (solution_number == 1)
  { 
   yaw = euler_out.yaw; 
   pitch = euler_out.pitch;
   roll = euler_out.roll;
  }
  else
  { 
   yaw = euler_out2.yaw; 
   pitch = euler_out2.pitch;
   roll = euler_out2.roll;
  }
 }

 /**@brief Create a scaled copy of the matrix 
 * @param s Scaling vector The elements of the vector will scale each column */

 btMatrix3x3 scaled(const btVector3& s) const
 {
  return btMatrix3x3(m_el[0].x() * s.x(), m_el[0].y() * s.y(), m_el[0].z() * s.z(),
   m_el[1].x() * s.x(), m_el[1].y() * s.y(), m_el[1].z() * s.z(),
   m_el[2].x() * s.x(), m_el[2].y() * s.y(), m_el[2].z() * s.z());
 }

 /**@brief Return the determinant of the matrix */
 btScalar            determinant() const;
 /**@brief Return the adjoint of the matrix */
 btMatrix3x3 adjoint() const;
 /**@brief Return the matrix with all values non negative */
 btMatrix3x3 absolute() const;
 /**@brief Return the transpose of the matrix */
 btMatrix3x3 transpose() const;
 /**@brief Return the inverse of the matrix */
 btMatrix3x3 inverse() const; 

 btMatrix3x3 transposeTimes(const btMatrix3x3& m) const;
 btMatrix3x3 timesTranspose(const btMatrix3x3& m) const;

  btScalar tdotx(const btVector3& v) const 
 {
  return m_el[0].x() * v.x() + m_el[1].x() * v.y() + m_el[2].x() * v.z();
 }
  btScalar tdoty(const btVector3& v) const 
 {
  return m_el[0].y() * v.x() + m_el[1].y() * v.y() + m_el[2].y() * v.z();
 }
  btScalar tdotz(const btVector3& v) const 
 {
  return m_el[0].z() * v.x() + m_el[1].z() * v.y() + m_el[2].z() * v.z();
 }


 /**@brief diagonalizes this matrix by the Jacobi method.
 * @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
 * coordinate system, i.e., old_this = rot * new_this * rot^T. 
 * @param threshold See iteration
 * @param iteration The iteration stops when all off-diagonal elements are less than the threshold multiplied 
 * by the sum of the absolute values of the diagonal, or when maxSteps have been executed. 
 * 
 * Note that this matrix is assumed to be symmetric. 
 */
 void diagonalize(btMatrix3x3& rot, btScalar threshold, int maxSteps)
 {
  rot.setIdentity();
  for (int step = maxSteps; step > 0; step--)
  {
   // find off-diagonal element [p][q] with largest magnitude
   int p = 0;
   int q = 1;
   int r = 2;
   btScalar max = btFabs(m_el[0][1]);
   btScalar v = btFabs(m_el[0][2]);
   if (v > max)
   {
    q = 2;
    r = 1;
    max = v;
   }
   v = btFabs(m_el[1][2]);
   if (v > max)
   {
    p = 1;
    q = 2;
    r = 0;
    max = v;
   }

   btScalar t = threshold * (btFabs(m_el[0][0]) + btFabs(m_el[1][1]) + btFabs(m_el[2][2]));
   if (max <= t)
   {
    if (max <= SIMD_EPSILON * t)
    {
     return;
    }
    step = 1;
   }

   // compute Jacobi rotation J which leads to a zero for element [p][q] 
   btScalar mpq = m_el[p][q];
   btScalar theta = (m_el[q][q] - m_el[p][p]) / (2 * mpq);
   btScalar theta2 = theta * theta;
   btScalar cos;
   btScalar sin;
   if (theta2 * theta2 < btScalar(10 / SIMD_EPSILON))
   {
    t = (theta >= 0) ? 1 / (theta + btSqrt(1 + theta2))
     : 1 / (theta - btSqrt(1 + theta2));
    cos = 1 / btSqrt(1 + t * t);
    sin = cos * t;
   }
   else
   {
    // approximation for large theta-value, i.e., a nearly diagonal matrix
    t = 1 / (theta * (2 + btScalar(0.5) / theta2));
    cos = 1 - btScalar(0.5) * t * t;
    sin = cos * t;
   }

   // apply rotation to matrix (this = J^T * this * J)
   m_el[p][q] = m_el[q][p] = 0;
   m_el[p][p] -= t * mpq;
   m_el[q][q] += t * mpq;
   btScalar mrp = m_el[r][p];
   btScalar mrq = m_el[r][q];
   m_el[r][p] = m_el[p][r] = cos * mrp - sin * mrq;
   m_el[r][q] = m_el[q][r] = cos * mrq + sin * mrp;

   // apply rotation to rot (rot = rot * J)
   for (int i = 0; i < 3; i++)
   {
    btVector3& row = rot[i];
    mrp = row[p];
    mrq = row[q];
    row[p] = cos * mrp - sin * mrq;
    row[q] = cos * mrq + sin * mrp;
   }
  }
 }




 /**@brief Calculate the matrix cofactor 
 * @param r1 The first row to use for calculating the cofactor
 * @param c1 The first column to use for calculating the cofactor
 * @param r1 The second row to use for calculating the cofactor
 * @param c1 The second column to use for calculating the cofactor
 * See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
 */
 btScalar cofac(int r1, int c1, int r2, int c2) const 
 {
  return m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] * m_el[r2][c1];
 }

 void serialize(struct btMatrix3x3Data& dataOut) const;

 void serializeFloat(struct btMatrix3x3FloatData& dataOut) const;

 void deSerialize(const struct btMatrix3x3Data& dataIn);

 void deSerializeFloat(const struct btMatrix3x3FloatData& dataIn);

 void deSerializeDouble(const struct btMatrix3x3DoubleData& dataIn);

};


 btMatrix3x3& 
btMatrix3x3::operator*=(const btMatrix3x3& m)
{
 setValue(m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]),
  m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]),
  m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]));
 return *this;
}

 btMatrix3x3& 
btMatrix3x3::operator+=(const btMatrix3x3& m)
{
 setValue(
  m_el[0][0]+m.m_el[0][0], 
  m_el[0][1]+m.m_el[0][1],
  m_el[0][2]+m.m_el[0][2],
  m_el[1][0]+m.m_el[1][0], 
  m_el[1][1]+m.m_el[1][1],
  m_el[1][2]+m.m_el[1][2],
  m_el[2][0]+m.m_el[2][0], 
  m_el[2][1]+m.m_el[2][1],
  m_el[2][2]+m.m_el[2][2]);
 return *this;
}

 btMatrix3x3
operator*(const btMatrix3x3& m, const btScalar & k)
{
 return btMatrix3x3(
  m[0].x()*k,m[0].y()*k,m[0].z()*k,
  m[1].x()*k,m[1].y()*k,m[1].z()*k,
  m[2].x()*k,m[2].y()*k,m[2].z()*k);
}

  btMatrix3x3 
operator+(const btMatrix3x3& m1, const btMatrix3x3& m2)
{
 return btMatrix3x3(
 m1[0][0]+m2[0][0], 
 m1[0][1]+m2[0][1],
 m1[0][2]+m2[0][2],
 m1[1][0]+m2[1][0], 
 m1[1][1]+m2[1][1],
 m1[1][2]+m2[1][2],
 m1[2][0]+m2[2][0], 
 m1[2][1]+m2[2][1],
 m1[2][2]+m2[2][2]);
}

 btMatrix3x3 
operator-(const btMatrix3x3& m1, const btMatrix3x3& m2)
{
 return btMatrix3x3(
 m1[0][0]-m2[0][0], 
 m1[0][1]-m2[0][1],
 m1[0][2]-m2[0][2],
 m1[1][0]-m2[1][0], 
 m1[1][1]-m2[1][1],
 m1[1][2]-m2[1][2],
 m1[2][0]-m2[2][0], 
 m1[2][1]-m2[2][1],
 m1[2][2]-m2[2][2]);
}


 btMatrix3x3& 
btMatrix3x3::operator-=(const btMatrix3x3& m)
{
 setValue(
 m_el[0][0]-m.m_el[0][0], 
 m_el[0][1]-m.m_el[0][1],
 m_el[0][2]-m.m_el[0][2],
 m_el[1][0]-m.m_el[1][0], 
 m_el[1][1]-m.m_el[1][1],
 m_el[1][2]-m.m_el[1][2],
 m_el[2][0]-m.m_el[2][0], 
 m_el[2][1]-m.m_el[2][1],
 m_el[2][2]-m.m_el[2][2]);
 return *this;
}


 btScalar 
btMatrix3x3::determinant() const
{ 
 return btTriple((*this)[0], (*this)[1], (*this)[2]);
}


 btMatrix3x3 
btMatrix3x3::absolute() const
{
 return btMatrix3x3(
  btFabs(m_el[0].x()), btFabs(m_el[0].y()), btFabs(m_el[0].z()),
  btFabs(m_el[1].x()), btFabs(m_el[1].y()), btFabs(m_el[1].z()),
  btFabs(m_el[2].x()), btFabs(m_el[2].y()), btFabs(m_el[2].z()));
}

 btMatrix3x3 
btMatrix3x3::transpose() const 
{
 return btMatrix3x3(m_el[0].x(), m_el[1].x(), m_el[2].x(),
  m_el[0].y(), m_el[1].y(), m_el[2].y(),
  m_el[0].z(), m_el[1].z(), m_el[2].z());
}

 btMatrix3x3 
btMatrix3x3::adjoint() const 
{
 return btMatrix3x3(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
  cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
  cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
}

 btMatrix3x3 
btMatrix3x3::inverse() const
{
 btVector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
 btScalar det = (*this)[0].dot(co);
 btFullAssert(det != btScalar(0.0));
 btScalar s = btScalar(1.0) / det;
 return btMatrix3x3(co.x() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
  co.y() * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
  co.z() * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
}

 btMatrix3x3 
btMatrix3x3::transposeTimes(const btMatrix3x3& m) const
{
 return btMatrix3x3(
  m_el[0].x() * m[0].x() + m_el[1].x() * m[1].x() + m_el[2].x() * m[2].x(),
  m_el[0].x() * m[0].y() + m_el[1].x() * m[1].y() + m_el[2].x() * m[2].y(),
  m_el[0].x() * m[0].z() + m_el[1].x() * m[1].z() + m_el[2].x() * m[2].z(),
  m_el[0].y() * m[0].x() + m_el[1].y() * m[1].x() + m_el[2].y() * m[2].x(),
  m_el[0].y() * m[0].y() + m_el[1].y() * m[1].y() + m_el[2].y() * m[2].y(),
  m_el[0].y() * m[0].z() + m_el[1].y() * m[1].z() + m_el[2].y() * m[2].z(),
  m_el[0].z() * m[0].x() + m_el[1].z() * m[1].x() + m_el[2].z() * m[2].x(),
  m_el[0].z() * m[0].y() + m_el[1].z() * m[1].y() + m_el[2].z() * m[2].y(),
  m_el[0].z() * m[0].z() + m_el[1].z() * m[1].z() + m_el[2].z() * m[2].z());
}

 btMatrix3x3 
btMatrix3x3::timesTranspose(const btMatrix3x3& m) const
{
 return btMatrix3x3(
  m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
  m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
  m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));

}

 btVector3 
operator*(const btMatrix3x3& m, const btVector3& v) 
{
 return btVector3(m[0].dot(v), m[1].dot(v), m[2].dot(v));
}


 btVector3
operator*(const btVector3& v, const btMatrix3x3& m)
{
 return btVector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
}

 btMatrix3x3 
operator*(const btMatrix3x3& m1, const btMatrix3x3& m2)
{
 return btMatrix3x3(
  m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]),
  m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]),
  m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]));
}

/*
 btMatrix3x3 btMultTransposeLeft(const btMatrix3x3& m1, const btMatrix3x3& m2) {
return btMatrix3x3(
m1[0][0] * m2[0][0] + m1[1][0] * m2[1][0] + m1[2][0] * m2[2][0],
m1[0][0] * m2[0][1] + m1[1][0] * m2[1][1] + m1[2][0] * m2[2][1],
m1[0][0] * m2[0][2] + m1[1][0] * m2[1][2] + m1[2][0] * m2[2][2],
m1[0][1] * m2[0][0] + m1[1][1] * m2[1][0] + m1[2][1] * m2[2][0],
m1[0][1] * m2[0][1] + m1[1][1] * m2[1][1] + m1[2][1] * m2[2][1],
m1[0][1] * m2[0][2] + m1[1][1] * m2[1][2] + m1[2][1] * m2[2][2],
m1[0][2] * m2[0][0] + m1[1][2] * m2[1][0] + m1[2][2] * m2[2][0],
m1[0][2] * m2[0][1] + m1[1][2] * m2[1][1] + m1[2][2] * m2[2][1],
m1[0][2] * m2[0][2] + m1[1][2] * m2[1][2] + m1[2][2] * m2[2][2]);
}
*/

/**@brief Equality operator between two matrices
* It will test all elements are equal.  */
 bool operator==(const btMatrix3x3& m1, const btMatrix3x3& m2)
{
 return ( m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0] == m2[2][0] &&
  m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1] == m2[2][1] &&
  m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2] == m2[2][2] );
}

///for serialization
class btMatrix3x3FloatData
{ public: 
 btVector3FloatData m_el[3];
};

///for serialization
class btMatrix3x3DoubleData
{ public: 
 btVector3DoubleData m_el[3];
};


 

 void btMatrix3x3::serialize(struct btMatrix3x3Data& dataOut) const
{
 for (int i=0;i<3;i++)
  m_el[i].serialize(dataOut.m_el[i]);
}

 void btMatrix3x3::serializeFloat(struct btMatrix3x3FloatData& dataOut) const
{
 for (int i=0;i<3;i++)
  m_el[i].serializeFloat(dataOut.m_el[i]);
}


 void btMatrix3x3::deSerialize(const struct btMatrix3x3Data& dataIn)
{
 for (int i=0;i<3;i++)
  m_el[i].deSerialize(dataIn.m_el[i]);
}

 void btMatrix3x3::deSerializeFloat(const struct btMatrix3x3FloatData& dataIn)
{
 for (int i=0;i<3;i++)
  m_el[i].deSerializeFloat(dataIn.m_el[i]);
}

 void btMatrix3x3::deSerializeDouble(const struct btMatrix3x3DoubleData& dataIn)
{
 for (int i=0;i<3;i++)
  m_el[i].deSerializeDouble(dataIn.m_el[i]);
}

#endif //BT_MATRIX3x3_H

//// ../src/LinearMath/btQuaternion.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_SIMD__QUATERNION_H_
#define BT_SIMD__QUATERNION_H_


#include "btVector3.h"
#include "btQuadWord.h"

/**@brief The btQuaternion implements quaternion to perform linear algebra rotations in combination with btMatrix3x3, btVector3 and btTransform. */
class btQuaternion : public btQuadWord {
public:
  /**@brief No initialization constructor */
 btQuaternion() {}

 //  template <typename btScalar>
 //  explicit Quaternion(const btScalar *v) : Tuple4<btScalar>(v) {}
  /**@brief Constructor from scalars */
 btQuaternion(const btScalar& x, const btScalar& y, const btScalar& z, const btScalar& w) 
  : btQuadWord(x, y, z, w) 
 {}
  /**@brief Axis angle Constructor
   * @param axis The axis which the rotation is around
   * @param angle The magnitude of the rotation around the angle (Radians) */
 btQuaternion(const btVector3& axis, const btScalar& angle) 
 { 
  setRotation(axis, angle); 
 }
  /**@brief Constructor from Euler angles
   * @param yaw Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
   * @param pitch Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X */
 btQuaternion(const btScalar& yaw, const btScalar& pitch, const btScalar& roll)
 { 
#ifndef BT_EULER_DEFAULT_ZYX
  setEuler(yaw, pitch, roll); 
#else
  setEulerZYX(yaw, pitch, roll); 
#endif 
 }
  /**@brief Set the rotation using axis angle notation 
   * @param axis The axis around which to rotate
   * @param angle The magnitude of the rotation in Radians */
 void setRotation(const btVector3& axis, const btScalar& angle)
 {
  btScalar d = axis.length();
  btAssert(d != btScalar(0.0));
  btScalar s = btSin(angle * btScalar(0.5)) / d;
  setValue(axis.x() * s, axis.y() * s, axis.z() * s, 
   btCos(angle * btScalar(0.5)));
 }
  /**@brief Set the quaternion using Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around Z */
 void setEuler(const btScalar& yaw, const btScalar& pitch, const btScalar& roll)
 {
  btScalar halfYaw = btScalar(yaw) * btScalar(0.5);  
  btScalar halfPitch = btScalar(pitch) * btScalar(0.5);  
  btScalar halfRoll = btScalar(roll) * btScalar(0.5);  
  btScalar cosYaw = btCos(halfYaw);
  btScalar sinYaw = btSin(halfYaw);
  btScalar cosPitch = btCos(halfPitch);
  btScalar sinPitch = btSin(halfPitch);
  btScalar cosRoll = btCos(halfRoll);
  btScalar sinRoll = btSin(halfRoll);
  setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
   cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
   sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
   cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
 }
  /**@brief Set the quaternion using euler angles 
   * @param yaw Angle around Z
   * @param pitch Angle around Y
   * @param roll Angle around X */
 void setEulerZYX(const btScalar& yaw, const btScalar& pitch, const btScalar& roll)
 {
  btScalar halfYaw = btScalar(yaw) * btScalar(0.5);  
  btScalar halfPitch = btScalar(pitch) * btScalar(0.5);  
  btScalar halfRoll = btScalar(roll) * btScalar(0.5);  
  btScalar cosYaw = btCos(halfYaw);
  btScalar sinYaw = btSin(halfYaw);
  btScalar cosPitch = btCos(halfPitch);
  btScalar sinPitch = btSin(halfPitch);
  btScalar cosRoll = btCos(halfRoll);
  btScalar sinRoll = btSin(halfRoll);
  setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                         cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                         cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                         cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
 }
  /**@brief Add two quaternions
   * @param q The quaternion to add to this one */
  btQuaternion& operator+=(const btQuaternion& q)
 {
  m_floats[0] += q.x(); m_floats[1] += q.y(); m_floats[2] += q.z(); m_floats[3] += q.m_floats[3];
  return *this;
 }

  /**@brief Subtract out a quaternion
   * @param q The quaternion to subtract from this one */
 btQuaternion& operator-=(const btQuaternion& q) 
 {
  m_floats[0] -= q.x(); m_floats[1] -= q.y(); m_floats[2] -= q.z(); m_floats[3] -= q.m_floats[3];
  return *this;
 }

  /**@brief Scale this quaternion
   * @param s The scalar to scale by */
 btQuaternion& operator*=(const btScalar& s)
 {
  m_floats[0] *= s; m_floats[1] *= s; m_floats[2] *= s; m_floats[3] *= s;
  return *this;
 }

  /**@brief Multiply this quaternion by q on the right
   * @param q The other quaternion 
   * Equivilant to this = this * q */
 btQuaternion& operator*=(const btQuaternion& q)
 {
  setValue(m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(),
   m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(),
   m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(),
   m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z());
  return *this;
 }
  /**@brief Return the dot product between this quaternion and another
   * @param q The other quaternion */
 btScalar dot(const btQuaternion& q) const
 {
  return m_floats[0] * q.x() + m_floats[1] * q.y() + m_floats[2] * q.z() + m_floats[3] * q.m_floats[3];
 }

  /**@brief Return the length squared of the quaternion */
 btScalar length2() const
 {
  return dot(*this);
 }

  /**@brief Return the length of the quaternion */
 btScalar length() const
 {
  return btSqrt(length2());
 }

  /**@brief Normalize the quaternion 
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
 btQuaternion& normalize() 
 {
  return *this /= length();
 }

  /**@brief Return a scaled version of this quaternion
   * @param s The scale factor */
  btQuaternion
 operator*(const btScalar& s) const
 {
  return btQuaternion(x() * s, y() * s, z() * s, m_floats[3] * s);
 }


  /**@brief Return an inversely scaled versionof this quaternion
   * @param s The inverse scale factor */
 btQuaternion operator/(const btScalar& s) const
 {
  btAssert(s != btScalar(0.0));
  return *this * (btScalar(1.0) / s);
 }

  /**@brief Inversely scale this quaternion
   * @param s The scale factor */
 btQuaternion& operator/=(const btScalar& s) 
 {
  btAssert(s != btScalar(0.0));
  return *this *= btScalar(1.0) / s;
 }

  /**@brief Return a normalized version of this quaternion */
 btQuaternion normalized() const 
 {
  return *this / length();
 } 
  /**@brief Return the angle between this quaternion and the other 
   * @param q The other quaternion */
 btScalar angle(const btQuaternion& q) const 
 {
  btScalar s = btSqrt(length2() * q.length2());
  btAssert(s != btScalar(0.0));
  return btAcos(dot(q) / s);
 }
  /**@brief Return the angle of rotation represented by this quaternion */
 btScalar getAngle() const 
 {
  btScalar s = btScalar(2.) * btAcos(m_floats[3]);
  return s;
 }

 /**@brief Return the axis of the rotation represented by this quaternion */
 btVector3 getAxis() const
 {
  btScalar s_squared = btScalar(1.) - btPow(m_floats[3], btScalar(2.));
  if (s_squared < btScalar(10.) * SIMD_EPSILON) //Check for divide by zero
   return btVector3(1.0, 0.0, 0.0);  // Arbitrary
  btScalar s = btSqrt(s_squared);
  return btVector3(m_floats[0] / s, m_floats[1] / s, m_floats[2] / s);
 }

 /**@brief Return the inverse of this quaternion */
 btQuaternion inverse() const
 {
  return btQuaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
 }

  /**@brief Return the sum of this quaternion and the other 
   * @param q2 The other quaternion */
  btQuaternion
 operator+(const btQuaternion& q2) const
 {
  const btQuaternion& q1 = *this;
  return btQuaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
 }

  /**@brief Return the difference between this quaternion and the other 
   * @param q2 The other quaternion */
  btQuaternion
 operator-(const btQuaternion& q2) const
 {
  const btQuaternion& q1 = *this;
  return btQuaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
 }

  /**@brief Return the negative of this quaternion 
   * This simply negates each element */
  btQuaternion operator-() const
 {
  const btQuaternion& q2 = *this;
  return btQuaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2.m_floats[3]);
 }
  /**@todo document this and it's use */
  btQuaternion farthest( const btQuaternion& qd) const 
 {
  btQuaternion diff,sum;
  diff = *this - qd;
  sum = *this + qd;
  if( diff.dot(diff) > sum.dot(sum) )
   return qd;
  return (-qd);
 }

 /**@todo document this and it's use */
  btQuaternion nearest( const btQuaternion& qd) const 
 {
  btQuaternion diff,sum;
  diff = *this - qd;
  sum = *this + qd;
  if( diff.dot(diff) < sum.dot(sum) )
   return qd;
  return (-qd);
 }


  /**@brief Return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
   * @param q The other quaternion to interpolate with 
   * @param t The ratio between this and q to interpolate.  If t = 0 the result is this, if t=1 the result is q.
   * Slerp interpolates assuming constant velocity.  */
 btQuaternion slerp(const btQuaternion& q, const btScalar& t) const
 {
  btScalar theta = angle(q);
  if (theta != btScalar(0.0))
  {
   btScalar d = btScalar(1.0) / btSin(theta);
   btScalar s0 = btSin((btScalar(1.0) - t) * theta);
   btScalar s1 = btSin(t * theta);   
                        if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
                          return btQuaternion((m_floats[0] * s0 + -q.x() * s1) * d,
                                              (m_floats[1] * s0 + -q.y() * s1) * d,
                                              (m_floats[2] * s0 + -q.z() * s1) * d,
                                              (m_floats[3] * s0 + -q.m_floats[3] * s1) * d);
                        else
                          return btQuaternion((m_floats[0] * s0 + q.x() * s1) * d,
                                              (m_floats[1] * s0 + q.y() * s1) * d,
                                              (m_floats[2] * s0 + q.z() * s1) * d,
                                              (m_floats[3] * s0 + q.m_floats[3] * s1) * d);
                        
  }
  else
  {
   return *this;
  }
 }

 static const btQuaternion& getIdentity()
 {
  static const btQuaternion identityQuat(btScalar(0.),btScalar(0.),btScalar(0.),btScalar(1.));
  return identityQuat;
 }

  const btScalar& getW() const { return m_floats[3]; }

 
};


/**@brief Return the negative of a quaternion */
 btQuaternion
operator-(const btQuaternion& q)
{
 return btQuaternion(-q.x(), -q.y(), -q.z(), -q.w());
}



/**@brief Return the product of two quaternions */
 btQuaternion
operator*(const btQuaternion& q1, const btQuaternion& q2) {
 return btQuaternion(q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
  q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
  q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
  q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z()); 
}

 btQuaternion
operator*(const btQuaternion& q, const btVector3& w)
{
 return btQuaternion( q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
  q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
  q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
  -q.x() * w.x() - q.y() * w.y() - q.z() * w.z()); 
}

 btQuaternion
operator*(const btVector3& w, const btQuaternion& q)
{
 return btQuaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
  w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
  w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
  -w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
}

/**@brief Calculate the dot product between two quaternions */
 btScalar 
dot(const btQuaternion& q1, const btQuaternion& q2) 
{ 
 return q1.dot(q2); 
}


/**@brief Return the length of a quaternion */
 btScalar
length(const btQuaternion& q) 
{ 
 return q.length(); 
}

/**@brief Return the angle between two quaternions*/
 btScalar
angle(const btQuaternion& q1, const btQuaternion& q2) 
{ 
 return q1.angle(q2); 
}

/**@brief Return the inverse of a quaternion*/
 btQuaternion
inverse(const btQuaternion& q) 
{
 return q.inverse();
}

/**@brief Return the result of spherical linear interpolation betwen two quaternions 
 * @param q1 The first quaternion
 * @param q2 The second quaternion 
 * @param t The ration between q1 and q2.  t = 0 return q1, t=1 returns q2 
 * Slerp assumes constant velocity between positions. */
 btQuaternion
slerp(const btQuaternion& q1, const btQuaternion& q2, const btScalar& t) 
{
 return q1.slerp(q2, t);
}

 btVector3 
quatRotate(const btQuaternion& rotation, const btVector3& v) 
{
 btQuaternion q = rotation * v;
 q *= rotation.inverse();
 return btVector3(q.getX(),q.getY(),q.getZ());
}

 btQuaternion 
shortestArcQuat(const btVector3& v0, const btVector3& v1) // Game Programming Gems 2.10. make sure v0,v1 are normalized
{
 btVector3 c = v0.cross(v1);
 btScalar  d = v0.dot(v1);

 if (d < -1.0 + SIMD_EPSILON)
 {
  btVector3 n,unused;
  btPlaneSpace1(v0,n,unused);
  return btQuaternion(n.x(),n.y(),n.z(),0.0f); // just pick any vector that is orthogonal to v0
 }

 btScalar  s = btSqrt((1.0f + d) * 2.0f);
 btScalar rs = 1.0f / s;

 return btQuaternion(c.getX()*rs,c.getY()*rs,c.getZ()*rs,s * 0.5f);
}

 btQuaternion 
shortestArcQuatNormalize2(btVector3& v0,btVector3& v1)
{
 v0.normalize();
 v1.normalize();
 return shortestArcQuat(v0,v1);
}

#endif //BT_SIMD__QUATERNION_H_




//// ../src/LinearMath/btQuadWord.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_SIMD_QUADWORD_H
#define BT_SIMD_QUADWORD_H

#include "btScalar.h"
#include "btMinMax.h"


#if defined (__CELLOS_LV2) && defined (__SPU__)
#include <altivec.h>
#endif

/**@brief The btQuadWord class is base class for btVector3 and btQuaternion. 
 * Some issues under PS3 Linux with IBM 2.1 SDK, gcc compiler prevent from using aligned quadword.
 */
#ifndef USE_LIBSPE2
class btQuadWord
#else
class btQuadWord
#endif
{
protected:

 //__CELLOS_LV2__ __SPU__

 public:
  

  /**@brief Return the x value */
   const btScalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
   const btScalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
   const btScalar& getZ() const { return m_floats[2]; }
  /**@brief Set the x value */
   void setX(btScalar x) { m_floats[0] = x;};
  /**@brief Set the y value */
   void setY(btScalar y) { m_floats[1] = y;};
  /**@brief Set the z value */
   void setZ(btScalar z) { m_floats[2] = z;};
  /**@brief Set the w value */
   void setW(btScalar w) { m_floats[3] = w;};
  /**@brief Return the x value */
   const btScalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
   const btScalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
   const btScalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
   const btScalar& w() const { return m_floats[3]; }

 // btScalar&       operator[](int i)       { return (&m_floats[0])[i]; }      
 // const btScalar& operator[](int i) const { return (&m_floats[0])[i]; }
 ///operator btScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
  operator       btScalar *()       { return &m_floats[0]; }
  operator const btScalar *() const { return &m_floats[0]; }

  bool operator==(const btQuadWord& other) const
 {
  return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
 }

  bool operator!=(const btQuadWord& other) const
 {
  return !(*this == other);
 }

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
   void  setValue(const btScalar& x, const btScalar& y, const btScalar& z)
  {
   m_floats[0]=x;
   m_floats[1]=y;
   m_floats[2]=z;
   m_floats[3] = 0.f;
  }

/*  void getValue(btScalar *m) const 
  {
   m[0] = m_floats[0];
   m[1] = m_floats[1];
   m[2] = m_floats[2];
  }
*/
/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
   void setValue(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w)
  {
   m_floats[0]=x;
   m_floats[1]=y;
   m_floats[2]=z;
   m_floats[3]=w;
  }
  /**@brief No initialization constructor */
   btQuadWord()
  // :m_floats[0](btScalar(0.)),m_floats[1](btScalar(0.)),m_floats[2](btScalar(0.)),m_floats[3](btScalar(0.))
  {
  }
 
  /**@brief Three argument constructor (zeros w)
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
   btQuadWord(const btScalar& x, const btScalar& y, const btScalar& z)  
  {
   m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = 0.0f;
  }

/**@brief Initializing constructor
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
   btQuadWord(const btScalar& x, const btScalar& y, const btScalar& z,const btScalar& w) 
  {
   m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = w;
  }

  /**@brief Set each element to the max of the current values and the values of another btQuadWord
   * @param other The other btQuadWord to compare with 
   */
   void setMax(const btQuadWord& other)
  {
   btSetMax(m_floats[0], other.m_floats[0]);
   btSetMax(m_floats[1], other.m_floats[1]);
   btSetMax(m_floats[2], other.m_floats[2]);
   btSetMax(m_floats[3], other.m_floats[3]);
  }
  /**@brief Set each element to the min of the current values and the values of another btQuadWord
   * @param other The other btQuadWord to compare with 
   */
   void setMin(const btQuadWord& other)
  {
   btSetMin(m_floats[0], other.m_floats[0]);
   btSetMin(m_floats[1], other.m_floats[1]);
   btSetMin(m_floats[2], other.m_floats[2]);
   btSetMin(m_floats[3], other.m_floats[3]);
  }



};

#endif //BT_SIMD_QUADWORD_H
//// ../src/BulletCollision/CollisionDispatch/btCollisionObject.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_OBJECT_H
#define BT_COLLISION_OBJECT_H

#include "LinearMath/btTransform.h"

//island management, m_activationState1
#define ACTIVE_TAG 1
#define ISLAND_SLEEPING 2
#define WANTS_DEACTIVATION 3
#define DISABLE_DEACTIVATION 4
#define DISABLE_SIMULATION 5

struct btBroadphaseProxy;
class btCollisionShape;
struct btCollisionShapeData;
#include "LinearMath/btMotionState.h"
#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btAlignedObjectArray.h"

typedef btAlignedObjectArray<class btCollisionObject*> btCollisionObjectArray;

#ifdef BT_USE_DOUBLE_PRECISION
#define btCollisionObjectData btCollisionObjectDoubleData
#define btCollisionObjectDataName "btCollisionObjectDoubleData"
#else
#define btCollisionObjectData btCollisionObjectFloatData
#define btCollisionObjectDataName "btCollisionObjectFloatData"
#endif


/// btCollisionObject can be used to manage collision detection objects. 
/// btCollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
/// They can be added to the btCollisionWorld.
class btCollisionObject
{

protected:

 btTransform m_worldTransform;

 ///m_interpolationWorldTransform is used for CCD and interpolation
 ///it can be either previous or future (predicted) transform
 btTransform m_interpolationWorldTransform;
 //those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities) 
 //without destroying the continuous interpolated motion (which uses this interpolation velocities)
 btVector3 m_interpolationLinearVelocity;
 btVector3 m_interpolationAngularVelocity;
 
 btVector3 m_anisotropicFriction;
 int   m_hasAnisotropicFriction;
 btScalar m_contactProcessingThreshold; 

 btBroadphaseProxy*  m_broadphaseHandle;
 btCollisionShape*  m_collisionShape;
 ///m_extensionPointer is used by some internal low-level Bullet extensions.
 void*     m_extensionPointer;
 
 ///m_rootCollisionShape is temporarily used to store the original collision shape
 ///The m_collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
 ///If it is NULL, the m_collisionShape is not temporarily replaced.
 btCollisionShape*  m_rootCollisionShape;

 int    m_collisionFlags;

 int    m_islandTag1;
 int    m_companionId;

 int    m_activationState1;
 btScalar   m_deactivationTime;

 btScalar  m_friction;
 btScalar  m_restitution;

 ///m_internalType is reserved to distinguish Bullet's btCollisionObject, btRigidBody, btSoftBody, btGhostObject etc.
 ///do not assign your own m_internalType unless you write a new dynamics object class.
 int    m_internalType;

 ///users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer
 void*   m_userObjectPointer;

 ///time of impact calculation
 btScalar  m_hitFraction; 
 
 ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
 btScalar  m_ccdSweptSphereRadius;

 /// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
 btScalar  m_ccdMotionThreshold;
 
 /// If some object should have elaborate collision filtering by sub-classes
 int   m_checkCollideWith;

 virtual bool checkCollideWithOverride(btCollisionObject* /* co */)
 {
  return true;
 }

public:

 

 enum CollisionFlags
 {
  CF_STATIC_OBJECT= 1,
  CF_KINEMATIC_OBJECT= 2,
  CF_NO_CONTACT_RESPONSE = 4,
  CF_CUSTOM_MATERIAL_CALLBACK = 8,//this allows per-triangle material (friction/restitution)
  CF_CHARACTER_OBJECT = 16,
  CF_DISABLE_VISUALIZE_OBJECT = 32, //disable debug drawing
  CF_DISABLE_SPU_COLLISION_PROCESSING = 64//disable parallel/SPU processing
 };

 enum CollisionObjectTypes
 {
  CO_COLLISION_OBJECT =1,
  CO_RIGID_BODY=2,
  ///CO_GHOST_OBJECT keeps track of all objects overlapping its AABB and that pass its collision filter
  ///It is useful for collision sensors, explosion objects, character controller etc.
  CO_GHOST_OBJECT=4,
  CO_SOFT_BODY=8,
  CO_HF_FLUID=16,
  CO_USER_TYPE=32
 };

  bool mergesSimulationIslands() const
 {
  ///static objects, kinematic and object without contact response don't merge islands
  return  ((m_collisionFlags & (CF_STATIC_OBJECT | CF_KINEMATIC_OBJECT | CF_NO_CONTACT_RESPONSE) )==0);
 }

 const btVector3& getAnisotropicFriction() const
 {
  return m_anisotropicFriction;
 }
 void setAnisotropicFriction(const btVector3& anisotropicFriction)
 {
  m_anisotropicFriction = anisotropicFriction;
  m_hasAnisotropicFriction = (anisotropicFriction[0]!=1.f) || (anisotropicFriction[1]!=1.f) || (anisotropicFriction[2]!=1.f);
 }
 bool hasAnisotropicFriction() const
 {
  return m_hasAnisotropicFriction!=0;
 }

 ///the constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
 ///Note that using contacts with positive distance can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
 void setContactProcessingThreshold( btScalar contactProcessingThreshold)
 {
  m_contactProcessingThreshold = contactProcessingThreshold;
 }
 btScalar getContactProcessingThreshold() const
 {
  return m_contactProcessingThreshold;
 }

  bool  isStaticObject() const {
  return (m_collisionFlags & CF_STATIC_OBJECT) != 0;
 }

  bool  isKinematicObject() const
 {
  return (m_collisionFlags & CF_KINEMATIC_OBJECT) != 0;
 }

  bool  isStaticOrKinematicObject() const
 {
  return (m_collisionFlags & (CF_KINEMATIC_OBJECT | CF_STATIC_OBJECT)) != 0 ;
 }

  bool  hasContactResponse() const {
  return (m_collisionFlags & CF_NO_CONTACT_RESPONSE)==0;
 }

 
 btCollisionObject();

 virtual ~btCollisionObject();

 virtual void setCollisionShape(btCollisionShape* collisionShape)
 {
  m_collisionShape = collisionShape;
  m_rootCollisionShape = collisionShape;
 }

  const btCollisionShape* getCollisionShape() const
 {
  return m_collisionShape;
 }

  btCollisionShape* getCollisionShape()
 {
  return m_collisionShape;
 }

  const btCollisionShape* getRootCollisionShape() const
 {
  return m_rootCollisionShape;
 }

  btCollisionShape* getRootCollisionShape()
 {
  return m_rootCollisionShape;
 }

 ///Avoid using this internal API call
 ///internalSetTemporaryCollisionShape is used to temporary replace the actual collision shape by a child collision shape.
 void internalSetTemporaryCollisionShape(btCollisionShape* collisionShape)
 {
  m_collisionShape = collisionShape;
 }

 ///Avoid using this internal API call, the extension pointer is used by some Bullet extensions. 
 ///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
 void*  internalGetExtensionPointer() const
 {
  return m_extensionPointer;
 }
 ///Avoid using this internal API call, the extension pointer is used by some Bullet extensions
 ///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
 void internalSetExtensionPointer(void* pointer)
 {
  m_extensionPointer = pointer;
 }

  int getActivationState() const { return m_activationState1;}
 
 void setActivationState(int newState);

 void setDeactivationTime(btScalar time)
 {
  m_deactivationTime = time;
 }
 btScalar getDeactivationTime() const
 {
  return m_deactivationTime;
 }

 void forceActivationState(int newState);

 void activate(bool forceActivation = false);

  bool isActive() const
 {
  return ((getActivationState() != ISLAND_SLEEPING) && (getActivationState() != DISABLE_SIMULATION));
 }

 void setRestitution(btScalar rest)
 {
  m_restitution = rest;
 }
 btScalar getRestitution() const
 {
  return m_restitution;
 }
 void setFriction(btScalar frict)
 {
  m_friction = frict;
 }
 btScalar getFriction() const
 {
  return m_friction;
 }

 ///reserved for Bullet internal usage
 int getInternalType() const
 {
  return m_internalType;
 }

 btTransform& getWorldTransform()
 {
  return m_worldTransform;
 }

 const btTransform& getWorldTransform() const
 {
  return m_worldTransform;
 }

 void setWorldTransform(const btTransform& worldTrans)
 {
  m_worldTransform = worldTrans;
 }


  btBroadphaseProxy* getBroadphaseHandle()
 {
  return m_broadphaseHandle;
 }

  const btBroadphaseProxy* getBroadphaseHandle() const
 {
  return m_broadphaseHandle;
 }

 void setBroadphaseHandle(btBroadphaseProxy* handle)
 {
  m_broadphaseHandle = handle;
 }


 const btTransform& getInterpolationWorldTransform() const
 {
  return m_interpolationWorldTransform;
 }

 btTransform& getInterpolationWorldTransform()
 {
  return m_interpolationWorldTransform;
 }

 void setInterpolationWorldTransform(const btTransform& trans)
 {
  m_interpolationWorldTransform = trans;
 }

 void setInterpolationLinearVelocity(const btVector3& linvel)
 {
  m_interpolationLinearVelocity = linvel;
 }

 void setInterpolationAngularVelocity(const btVector3& angvel)
 {
  m_interpolationAngularVelocity = angvel;
 }

 const btVector3& getInterpolationLinearVelocity() const
 {
  return m_interpolationLinearVelocity;
 }

 const btVector3& getInterpolationAngularVelocity() const
 {
  return m_interpolationAngularVelocity;
 }

  int getIslandTag() const
 {
  return m_islandTag1;
 }

 void setIslandTag(int tag)
 {
  m_islandTag1 = tag;
 }

  int getCompanionId() const
 {
  return m_companionId;
 }

 void setCompanionId(int id)
 {
  m_companionId = id;
 }

  btScalar   getHitFraction() const
 {
  return m_hitFraction; 
 }

 void setHitFraction(btScalar hitFraction)
 {
  m_hitFraction = hitFraction;
 }

 
  int getCollisionFlags() const
 {
  return m_collisionFlags;
 }

 void setCollisionFlags(int flags)
 {
  m_collisionFlags = flags;
 }
 
 ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
 btScalar   getCcdSweptSphereRadius() const
 {
  return m_ccdSweptSphereRadius;
 }

 ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
 void setCcdSweptSphereRadius(btScalar radius)
 {
  m_ccdSweptSphereRadius = radius;
 }

 btScalar  getCcdMotionThreshold() const
 {
  return m_ccdMotionThreshold;
 }

 btScalar  getCcdSquareMotionThreshold() const
 {
  return m_ccdMotionThreshold*m_ccdMotionThreshold;
 }



 /// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
 void setCcdMotionThreshold(btScalar ccdMotionThreshold)
 {
  m_ccdMotionThreshold = ccdMotionThreshold;
 }

 ///users can point to their objects, userPointer is not used by Bullet
 void* getUserPointer() const
 {
  return m_userObjectPointer;
 }
 
 ///users can point to their objects, userPointer is not used by Bullet
 void setUserPointer(void* userPointer)
 {
  m_userObjectPointer = userPointer;
 }


 inline bool checkCollideWith(btCollisionObject* co)
 {
  if (m_checkCollideWith)
   return checkCollideWithOverride(co);

  return true;
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, class btSerializer* serializer) const;

 virtual void serializeSingleObject(class btSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCollisionObjectDoubleData
{ public: 
 void     *m_broadphaseHandle;
 void     *m_collisionShape;
 btCollisionShapeData *m_rootCollisionShape;
 char     *m_name;

 btTransformDoubleData m_worldTransform;
 btTransformDoubleData m_interpolationWorldTransform;
 btVector3DoubleData  m_interpolationLinearVelocity;
 btVector3DoubleData  m_interpolationAngularVelocity;
 btVector3DoubleData  m_anisotropicFriction;
 double     m_contactProcessingThreshold; 
 double     m_deactivationTime;
 double     m_friction;
 double     m_restitution;
 double     m_hitFraction; 
 double     m_ccdSweptSphereRadius;
 double     m_ccdMotionThreshold;

 int      m_hasAnisotropicFriction;
 int      m_collisionFlags;
 int      m_islandTag1;
 int      m_companionId;
 int      m_activationState1;
 int      m_internalType;
 int      m_checkCollideWith;

 char m_padding[4];
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCollisionObjectFloatData
{ public: 
 void     *m_broadphaseHandle;
 void     *m_collisionShape;
 btCollisionShapeData *m_rootCollisionShape;
 char     *m_name;

 btTransformFloatData m_worldTransform;
 btTransformFloatData m_interpolationWorldTransform;
 btVector3FloatData  m_interpolationLinearVelocity;
 btVector3FloatData  m_interpolationAngularVelocity;
 btVector3FloatData  m_anisotropicFriction;
 float     m_contactProcessingThreshold; 
 float     m_deactivationTime;
 float     m_friction;
 float     m_restitution;
 float     m_hitFraction; 
 float     m_ccdSweptSphereRadius;
 float     m_ccdMotionThreshold;

 int      m_hasAnisotropicFriction;
 int      m_collisionFlags;
 int      m_islandTag1;
 int      m_companionId;
 int      m_activationState1;
 int      m_internalType;
 int      m_checkCollideWith;
};



 int btCollisionObject::calculateSerializeBufferSize() const
{
 return sizeof(btCollisionObjectData);
}



#endif //BT_COLLISION_OBJECT_H
//// ../src/LinearMath/btMotionState.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_MOTIONSTATE_H
#define BT_MOTIONSTATE_H

#include "btTransform.h"

///The btMotionState interface class allows the dynamics world to synchronize and interpolate the updated world transforms with graphics
///For optimizations, potentially only moving objects get synchronized (using setWorldPosition/setWorldOrientation)
class btMotionState
{
 public:
  
  virtual ~btMotionState()
  {
   
  }
  
  virtual void getWorldTransform(btTransform& worldTrans ) const =0;

  //Bullet only calls the update of worldtransform for active objects
  virtual void setWorldTransform(const btTransform& worldTrans)=0;
  
 
};

#endif //BT_MOTIONSTATE_H
//// ../src/LinearMath/btAlignedAllocator.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_ALIGNED_ALLOCATOR
#define BT_ALIGNED_ALLOCATOR

///we probably replace this with our own aligned memory allocator
///so we replace _aligned_malloc and _aligned_free with our own
///that is better portable and more predictable

#include "btScalar.h"
//#define BT_DEBUG_MEMORY_ALLOCATIONS 1
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS

#define btAlignedAlloc(a,b) \
  btAlignedAllocInternal(a,b,__LINE__,__FILE__)

#define btAlignedFree(ptr) \
  btAlignedFreeInternal(ptr,__LINE__,__FILE__)

void* btAlignedAllocInternal (size_t size, int alignment,int line,char* filename);

void btAlignedFreeInternal (void* ptr,int line,char* filename);

#else
 void* btAlignedAllocInternal (size_t size, int alignment);
 void btAlignedFreeInternal (void* ptr);

 #define btAlignedAlloc(size,alignment) btAlignedAllocInternal(size,alignment)
 #define btAlignedFree(ptr) btAlignedFreeInternal(ptr)

#endif
typedef int size_type;

typedef void *(btAlignedAllocFunc)(size_t size, int alignment);
typedef void (btAlignedFreeFunc)(void *memblock);
typedef void *(btAllocFunc)(size_t size);
typedef void (btFreeFunc)(void *memblock);

///The developer can let all Bullet memory allocations go through a custom memory allocator, using btAlignedAllocSetCustom
void btAlignedAllocSetCustom(btAllocFunc *allocFunc, btFreeFunc *freeFunc);
///If the developer has already an custom aligned allocator, then btAlignedAllocSetCustomAligned can be used. The default aligned allocator pre-allocates extra memory using the non-aligned allocator, and instruments it.
void btAlignedAllocSetCustomAligned(btAlignedAllocFunc *allocFunc, btAlignedFreeFunc *freeFunc);


///The btAlignedAllocator is a portable class for aligned memory allocations.
///Default implementations for unaligned and aligned allocations can be overridden by a custom allocator using btAlignedAllocSetCustom and btAlignedAllocSetCustomAligned.
template < typename T , unsigned Alignment >
class btAlignedAllocator {
 
 typedef btAlignedAllocator< T , Alignment > self_type;
 
public:

 //just going down a list:
 btAlignedAllocator() {}
 /*
 btAlignedAllocator( const self_type & ) {}
 */

 template < typename Other >
 btAlignedAllocator( const btAlignedAllocator< Other , Alignment > & ) {}

 typedef const T*         const_pointer;
 typedef const T&         const_reference;
 typedef T*               pointer;
 typedef T&               reference;
 typedef T                value_type;

 pointer       address   ( reference        ref ) const                           { return &ref; }
 const_pointer address   ( const_reference  ref ) const                           { return &ref; }
 pointer       allocate  ( size_type        n   , const_pointer *      hint = 0 ) {
  (void)hint;
  return reinterpret_cast< pointer >(btAlignedAlloc( sizeof(value_type) * n , Alignment ));
 }
 void          construct ( pointer          ptr , const value_type &   value    ) { new (ptr) value_type( value ); }
 void          deallocate( pointer          ptr ) {
  btAlignedFree( reinterpret_cast< void * >( ptr ) );
 }
 void          destroy   ( pointer          ptr )                                 { ptr->~value_type(); }
 

 template < typename O > struct rebind {
  typedef btAlignedAllocator< O , Alignment > other;
 };
 template < typename O >
 self_type & operator=( const btAlignedAllocator< O , Alignment > & ) { return *this; }

 friend bool operator==( const self_type & , const self_type & ) { return true; }
};



#endif //BT_ALIGNED_ALLOCATOR

//// ../src/LinearMath/btAlignedObjectArray.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_OBJECT_ARRAY__
#define BT_OBJECT_ARRAY__

#include "btScalar.h" // has definitions like 
#include "btAlignedAllocator.h"

///If the platform doesn't support placement new, you can disable BT_USE_PLACEMENT_NEW
///then the btAlignedObjectArray doesn't support objects with virtual methods, and non-trivial constructors/destructors
///You can enable BT_USE_MEMCPY, then swapping elements in the array will use memcpy instead of operator=
///see discussion here: http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1231 and
///http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1240

#define BT_USE_PLACEMENT_NEW 1
//#define BT_USE_MEMCPY 1 //disable, because it is cumbersome to find out for each platform where memcpy is defined. It can be in <memory.h> or <string.h> or otherwise...

#ifdef BT_USE_MEMCPY
#include <memory.h>
#include <string.h>
#endif //BT_USE_MEMCPY

#ifdef BT_USE_PLACEMENT_NEW
#include <new> //for placement new
#endif //BT_USE_PLACEMENT_NEW


///The btAlignedObjectArray template class uses a subset of the stl::vector interface for its methods
///It is developed to replace stl::vector to avoid portability issues, including STL alignment issues to add SIMD/SSE data
template <typename T> 
//template <class T> 
class btAlignedObjectArray
{
 btAlignedAllocator<T , 16> m_allocator;

 int     m_size;
 int     m_capacity;
 T*     m_data;
 //PCK: added this line
 bool    m_ownsMemory;

 protected:
   int allocSize(int size)
  {
   return (size ? size*2 : 1);
  }
   void copy(int start,int end, T* dest) const
  {
   int i;
   for (i=start;i<end;++i)
#ifdef BT_USE_PLACEMENT_NEW
    new (&dest[i]) T(m_data[i]);
#else
    dest[i] = m_data[i];
#endif //BT_USE_PLACEMENT_NEW
  }

   void init()
  {
   //PCK: added this line
   m_ownsMemory = true;
   m_data = 0;
   m_size = 0;
   m_capacity = 0;
  }
   void destroy(int first,int last)
  {
   int i;
   for (i=first; i<last;i++)
   {
    m_data[i].~T();
   }
  }

   void* allocate(int size)
  {
   if (size)
    return m_allocator.allocate(size);
   return 0;
  }

   void deallocate()
  {
   if(m_data) {
    //PCK: enclosed the deallocation in this block
    if (m_ownsMemory)
    {
     m_allocator.deallocate(m_data);
    }
    m_data = 0;
   }
  }

 


 public:
  
  btAlignedObjectArray()
  {
   init();
  }

  ~btAlignedObjectArray()
  {
   clear();
  }

  ///Generally it is best to avoid using the copy constructor of an btAlignedObjectArray, and use a (const) reference to the array instead.
  btAlignedObjectArray(const btAlignedObjectArray& otherArray)
  {
   init();

   int otherSize = otherArray.size();
   resize (otherSize);
   otherArray.copy(0, otherSize, m_data);
  }

  
  
  /// return the number of elements in the array
   int size() const
  { 
   return m_size;
  }
  
   const T& at(int n) const
  {
   return m_data[n];
  }

   T& at(int n)
  {
   return m_data[n];
  }

   const T& operator[](int n) const
  {
   return m_data[n];
  }

   T& operator[](int n)
  {
   return m_data[n];
  }
  

  ///clear the array, deallocated memory. Generally it is better to use array.resize(0), to reduce performance overhead of run-time memory (de)allocations.
   void clear()
  {
   destroy(0,size());
   
   deallocate();
   
   init();
  }

   void pop_back()
  {
   m_size--;
   m_data[m_size].~T();
  }

  ///resize changes the number of elements in the array. If the new size is larger, the new elements will be constructed using the optional second argument.
  ///when the new number of elements is smaller, the destructor will be called, but memory will not be freed, to reduce performance overhead of run-time memory (de)allocations.
   void resize(int newsize, const T& fillData=T())
  {
   int curSize = size();

   if (newsize < curSize)
   {
    for(int i = newsize; i < curSize; i++)
    {
     m_data[i].~T();
    }
   } else
   {
    if (newsize > size())
    {
     reserve(newsize);
    }
#ifdef BT_USE_PLACEMENT_NEW
    for (int i=curSize;i<newsize;i++)
    {
     new ( &m_data[i]) T(fillData);
    }
#endif //BT_USE_PLACEMENT_NEW

   }

   m_size = newsize;
  }
 
   T&  expandNonInitializing( )
  { 
   int sz = size();
   if( sz == capacity() )
   {
    reserve( allocSize(size()) );
   }
   m_size++;

   return m_data[sz];  
  }


   T&  expand( const T& fillValue=T())
  { 
   int sz = size();
   if( sz == capacity() )
   {
    reserve( allocSize(size()) );
   }
   m_size++;
#ifdef BT_USE_PLACEMENT_NEW
   new (&m_data[sz]) T(fillValue); //use the in-place new (not really allocating heap memory)
#endif

   return m_data[sz];  
  }


   void push_back(const T& _Val)
  { 
   int sz = size();
   if( sz == capacity() )
   {
    reserve( allocSize(size()) );
   }
   
#ifdef BT_USE_PLACEMENT_NEW
   new ( &m_data[m_size] ) T(_Val);
#else
   m_data[size()] = _Val;   
#endif //BT_USE_PLACEMENT_NEW

   m_size++;
  }

 
  /// return the pre-allocated (reserved) elements, this is at least as large as the total number of elements,see size() and reserve()
   int capacity() const
  { 
   return m_capacity;
  }
  
   void reserve(int _Count)
  { // determine new minimum length of allocated storage
   if (capacity() < _Count)
   { // not enough room, reallocate
    T* s = (T*)allocate(_Count);

    copy(0, size(), s);

    destroy(0,size());

    deallocate();
    
    //PCK: added this line
    m_ownsMemory = true;

    m_data = s;
    
    m_capacity = _Count;

   }
  }


  class less
  {
   public:

    bool operator() ( const T& a, const T& b )
    {
     return ( a < b );
    }
  };
 
  template <typename L>
  void quickSortInternal(L CompareFunc,int lo, int hi)
  {
  //  lo is the lower index, hi is the upper index
  //  of the region of array a that is to be sorted
   int i=lo, j=hi;
   T x=m_data[(lo+hi)/2];

   //  partition
   do
   {    
    while (CompareFunc(m_data[i],x)) 
     i++; 
    while (CompareFunc(x,m_data[j])) 
     j--;
    if (i<=j)
    {
     swap(i,j);
     i++; j--;
    }
   } while (i<=j);

   //  recursion
   if (lo<j) 
    quickSortInternal( CompareFunc, lo, j);
   if (i<hi) 
    quickSortInternal( CompareFunc, i, hi);
  }


  template <typename L>
  void quickSort(L CompareFunc)
  {
   //don't sort 0 or 1 elements
   if (size()>1)
   {
    quickSortInternal(CompareFunc,0,size()-1);
   }
  }


  ///heap sort from http://www.csse.monash.edu.au/~lloyd/tildeAlgDS/Sort/Heap/
  template <typename L>
  void downHeap(T *pArr, int k, int n,L CompareFunc)
  {
   /*  PRE: a[k+1..N] is a heap */
   /* POST:  a[k..N]  is a heap */
   
   T temp = pArr[k - 1];
   /* k has child(s) */
   while (k <= n/2) 
   {
    int child = 2*k;
    
    if ((child < n) && CompareFunc(pArr[child - 1] , pArr[child]))
    {
     child++;
    }
    /* pick larger child */
    if (CompareFunc(temp , pArr[child - 1]))
    {
     /* move child up */
     pArr[k - 1] = pArr[child - 1];
     k = child;
    }
    else
    {
     break;
    }
   }
   pArr[k - 1] = temp;
  } /*downHeap*/

  void swap(int index0,int index1)
  {
#ifdef BT_USE_MEMCPY
   char temp[sizeof(T)];
   memcpy(temp,&m_data[index0],sizeof(T));
   memcpy(&m_data[index0],&m_data[index1],sizeof(T));
   memcpy(&m_data[index1],temp,sizeof(T));
#else
   T temp = m_data[index0];
   m_data[index0] = m_data[index1];
   m_data[index1] = temp;
#endif //BT_USE_PLACEMENT_NEW

  }

 template <typename L>
 void heapSort(L CompareFunc)
 {
  /* sort a[0..N-1],  N.B. 0 to N-1 */
  int k;
  int n = m_size;
  for (k = n/2; k > 0; k--) 
  {
   downHeap(m_data, k, n, CompareFunc);
  }

  /* a[1..N] is now a heap */
  while ( n>=1 ) 
  {
   swap(0,n-1); /* largest of a[0..n-1] */


   n = n - 1;
   /* restore a[1..i-1] heap */
   downHeap(m_data, 1, n, CompareFunc);
  } 
 }

 ///non-recursive binary search, assumes sorted array
 int findBinarySearch(const T& key) const
 {
  int first = 0;
  int last = size()-1;

  //assume sorted array
  while (first <= last) {
   int mid = (first + last) / 2;  // compute mid point.
   if (key > m_data[mid]) 
    first = mid + 1;  // repeat search in top half.
   else if (key < m_data[mid]) 
    last = mid - 1; // repeat search in bottom half.
   else
    return mid;     // found it. return position /////
  }
  return size();    // failed to find key
 }


 int findLinearSearch(const T& key) const
 {
  int index=size();
  int i;

  for (i=0;i<size();i++)
  {
   if (m_data[i] == key)
   {
    index = i;
    break;
   }
  }
  return index;
 }

 void remove(const T& key)
 {

  int findIndex = findLinearSearch(key);
  if (findIndex<size())
  {
   swap( findIndex,size()-1);
   pop_back();
  }
 }

 //PCK: whole function
 void initializeFromBuffer(void *buffer, int size, int capacity)
 {
  clear();
  m_ownsMemory = false;
  m_data = (T*)buffer;
  m_size = size;
  m_capacity = capacity;
 }

 void copyFromArray(const btAlignedObjectArray& otherArray)
 {
  int otherSize = otherArray.size();
  resize (otherSize);
  otherArray.copy(0, otherSize, m_data);
 }

};

#endif //BT_OBJECT_ARRAY__
//// ../src/BulletCollision/CollisionDispatch/btCollisionDispatcher.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION__DISPATCHER_H
#define BT_COLLISION__DISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btAlignedObjectArray.h"

class btIDebugDraw;
class btOverlappingPairCache;
class btPoolAllocator;
class btCollisionConfiguration;

#include "btCollisionCreateFunc.h"

#define USE_DISPATCH_REGISTRY_ARRAY 1

class btCollisionDispatcher;
///user can override this nearcallback for collision filtering and more finegrained control over collision detection
typedef void (*btNearCallback)(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);


///btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
///Time of Impact, Closest Points and Penetration Depth.
class btCollisionDispatcher : public btDispatcher
{

protected:

 int  m_dispatcherFlags;

 btAlignedObjectArray<btPersistentManifold*> m_manifoldsPtr;

 btManifoldResult m_defaultManifoldResult;

 btNearCallback  m_nearCallback;
 
 btPoolAllocator* m_collisionAlgorithmPoolAllocator;

 btPoolAllocator* m_persistentManifoldPoolAllocator;

 btCollisionAlgorithmCreateFunc* m_doubleDispatch[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];

 btCollisionConfiguration* m_collisionConfiguration;


public:

 enum DispatcherFlags
 {
  CD_STATIC_STATIC_REPORTED = 1,
  CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
  CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4
 };

 int getDispatcherFlags() const
 {
  return m_dispatcherFlags;
 }

 void setDispatcherFlags(int flags)
 {
  m_dispatcherFlags = flags;
 }

 ///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
 void registerCollisionCreateFunc(int proxyType0,int proxyType1, btCollisionAlgorithmCreateFunc* createFunc);

 int getNumManifolds() const
 { 
  return int( m_manifoldsPtr.size());
 }

 btPersistentManifold** getInternalManifoldPointer()
 {
  return &m_manifoldsPtr[0];
 }

  btPersistentManifold* getManifoldByIndexInternal(int index)
 {
  return m_manifoldsPtr[index];
 }

  const btPersistentManifold* getManifoldByIndexInternal(int index) const
 {
  return m_manifoldsPtr[index];
 }

 btCollisionDispatcher (btCollisionConfiguration* collisionConfiguration);

 virtual ~btCollisionDispatcher();

 virtual btPersistentManifold* getNewManifold(void* b0,void* b1);
 
 virtual void releaseManifold(btPersistentManifold* manifold);


 virtual void clearManifold(btPersistentManifold* manifold);

   
 btCollisionAlgorithm* findAlgorithm(btCollisionObject* body0,btCollisionObject* body1,btPersistentManifold* sharedManifold = 0);
  
 virtual bool needsCollision(btCollisionObject* body0,btCollisionObject* body1);
 
 virtual bool needsResponse(btCollisionObject* body0,btCollisionObject* body1);
 
 virtual void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher) ;

 void setNearCallback(btNearCallback nearCallback)
 {
  m_nearCallback = nearCallback; 
 }

 btNearCallback getNearCallback() const
 {
  return m_nearCallback;
 }

 //by default, Bullet will use this near callback
 static void  defaultNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);

 virtual void* allocateCollisionAlgorithm(int size);

 virtual void freeCollisionAlgorithm(void* ptr);

 btCollisionConfiguration* getCollisionConfiguration()
 {
  return m_collisionConfiguration;
 }

 const btCollisionConfiguration* getCollisionConfiguration() const
 {
  return m_collisionConfiguration;
 }

 void setCollisionConfiguration(btCollisionConfiguration* config)
 {
  m_collisionConfiguration = config;
 }

 virtual btPoolAllocator* getInternalManifoldPool()
 {
  return m_persistentManifoldPoolAllocator;
 }

 virtual const btPoolAllocator* getInternalManifoldPool() const
 {
  return m_persistentManifoldPoolAllocator;
 }

};

#endif //BT_COLLISION__DISPATCHER_H

//// ../src/BulletCollision/BroadphaseCollision/btDispatcher.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_DISPATCHER_H
#define BT_DISPATCHER_H
#include "LinearMath/btScalar.h"

class btCollisionAlgorithm;
struct btBroadphaseProxy;
class btRigidBody;
class btCollisionObject;
class btOverlappingPairCache;


class btPersistentManifold;
class btStackAlloc;
class btPoolAllocator;

class btDispatcherInfo
{ public: 
 enum DispatchFunc
 {
  DISPATCH_DISCRETE = 1,
  DISPATCH_CONTINUOUS
 };
 btDispatcherInfo()
  :m_timeStep(btScalar(0.)),
  m_stepCount(0),
  m_dispatchFunc(DISPATCH_DISCRETE),
  m_timeOfImpact(btScalar(1.)),
  m_useContinuous(true),
  m_debugDraw(0),
  m_enableSatConvex(false),
  m_enableSPU(true),
  m_useEpa(true),
  m_allowedCcdPenetration(btScalar(0.04)),
  m_useConvexConservativeDistanceUtil(false),
  m_convexConservativeDistanceThreshold(0.0f),
  m_stackAllocator(0)
 {

 }
 btScalar m_timeStep;
 int   m_stepCount;
 int   m_dispatchFunc;
 mutable btScalar m_timeOfImpact;
 bool  m_useContinuous;
 class btIDebugDraw* m_debugDraw;
 bool  m_enableSatConvex;
 bool  m_enableSPU;
 bool  m_useEpa;
 btScalar m_allowedCcdPenetration;
 bool  m_useConvexConservativeDistanceUtil;
 btScalar m_convexConservativeDistanceThreshold;
 btStackAlloc* m_stackAllocator;
};

///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).
class btDispatcher
{


public:
 virtual ~btDispatcher() ;

 virtual btCollisionAlgorithm* findAlgorithm(btCollisionObject* body0,btCollisionObject* body1,btPersistentManifold* sharedManifold=0) = 0;

 virtual btPersistentManifold* getNewManifold(void* body0,void* body1)=0;

 virtual void releaseManifold(btPersistentManifold* manifold)=0;

 virtual void clearManifold(btPersistentManifold* manifold)=0;

 virtual bool needsCollision(btCollisionObject* body0,btCollisionObject* body1) = 0;

 virtual bool needsResponse(btCollisionObject* body0,btCollisionObject* body1)=0;

 virtual void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache,const btDispatcherInfo& dispatchInfo,btDispatcher* dispatcher)  =0;

 virtual int getNumManifolds() const = 0;

 virtual btPersistentManifold* getManifoldByIndexInternal(int index) = 0;

 virtual btPersistentManifold** getInternalManifoldPointer() = 0;

 virtual btPoolAllocator* getInternalManifoldPool() = 0;

 virtual const btPoolAllocator* getInternalManifoldPool() const = 0;

 virtual void* allocateCollisionAlgorithm(int size)  = 0;

 virtual void freeCollisionAlgorithm(void* ptr) = 0;

};


#endif //BT_DISPATCHER_H
//// ../src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_PERSISTENT_MANIFOLD_H
#define BT_PERSISTENT_MANIFOLD_H


#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "btManifoldPoint.h"
#include "LinearMath/btAlignedAllocator.h"

struct btCollisionResult;

///maximum contact breaking and merging threshold
extern btScalar gContactBreakingThreshold;

typedef bool (*ContactDestroyedCallback)(void* userPersistentData);
typedef bool (*ContactProcessedCallback)(btManifoldPoint& cp,void* body0,void* body1);
extern ContactDestroyedCallback gContactDestroyedCallback;
extern ContactProcessedCallback gContactProcessedCallback;

//the enum starts at 1024 to avoid type conflicts with btTypedConstraint
enum btContactManifoldTypes
{
 MIN_CONTACT_MANIFOLD_TYPE = 1024,
 BT_PERSISTENT_MANIFOLD_TYPE
};

#define MANIFOLD_CACHE_SIZE 4

///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///Those contact points are created by the collision narrow phase.
///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///reduces the cache to 4 points, when more then 4 points are added, using following rules:
///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.


ATTRIBUTE_ALIGNED128( class) btPersistentManifold : public btTypedObject
//class btPersistentManifold : public btTypedObject
{

 btManifoldPoint m_pointCache[MANIFOLD_CACHE_SIZE];

 /// this two body pointers can point to the physics rigidbody class.
 /// void* will allow any rigidbody class
 void* m_body0;
 void* m_body1;

 int m_cachedPoints;

 btScalar m_contactBreakingThreshold;
 btScalar m_contactProcessingThreshold;

 
 /// sort cached points so most isolated points come first
 int sortCachedPoints(const btManifoldPoint& pt);

 int  findContactPoint(const btManifoldPoint* unUsed, int numUnused,const btManifoldPoint& pt);

public:

 

 int m_companionIdA;
 int m_companionIdB;

 int m_index1a;

 btPersistentManifold();

 btPersistentManifold(void* body0,void* body1,int , btScalar contactBreakingThreshold,btScalar contactProcessingThreshold)
  : btTypedObject(BT_PERSISTENT_MANIFOLD_TYPE),
 m_body0(body0),m_body1(body1),m_cachedPoints(0),
  m_contactBreakingThreshold(contactBreakingThreshold),
  m_contactProcessingThreshold(contactProcessingThreshold)
 {
 }

  void* getBody0() { return m_body0;}
  void* getBody1() { return m_body1;}

  const void* getBody0() const { return m_body0;}
  const void* getBody1() const { return m_body1;}

 void setBodies(void* body0,void* body1)
 {
  m_body0 = body0;
  m_body1 = body1;
 }

 void clearUserCache(btManifoldPoint& pt);

#ifdef DEBUG_PERSISTENCY
 void DebugPersistency();
#endif //
 
  int getNumContacts() const { return m_cachedPoints;}

  const btManifoldPoint& getContactPoint(int index) const
 {
  btAssert(index < m_cachedPoints);
  return m_pointCache[index];
 }

  btManifoldPoint& getContactPoint(int index)
 {
  btAssert(index < m_cachedPoints);
  return m_pointCache[index];
 }

 ///@todo: get this margin from the current physics / collision environment
 btScalar getContactBreakingThreshold() const;

 btScalar getContactProcessingThreshold() const
 {
  return m_contactProcessingThreshold;
 }
 
 int getCacheEntry(const btManifoldPoint& newPoint) const;

 int addManifoldPoint( const btManifoldPoint& newPoint);

 void removeContactPoint (int index)
 {
  clearUserCache(m_pointCache[index]);

  int lastUsedIndex = getNumContacts() - 1;
//  m_pointCache[index] = m_pointCache[lastUsedIndex];
  if(index != lastUsedIndex) 
  {
   m_pointCache[index] = m_pointCache[lastUsedIndex]; 
   //get rid of duplicated userPersistentData pointer
   m_pointCache[lastUsedIndex].m_userPersistentData = 0;
   m_pointCache[lastUsedIndex].mConstraintRow[0].m_accumImpulse = 0.f;
   m_pointCache[lastUsedIndex].mConstraintRow[1].m_accumImpulse = 0.f;
   m_pointCache[lastUsedIndex].mConstraintRow[2].m_accumImpulse = 0.f;

   m_pointCache[lastUsedIndex].m_appliedImpulse = 0.f;
   m_pointCache[lastUsedIndex].m_lateralFrictionInitialized = false;
   m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0.f;
   m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0.f;
   m_pointCache[lastUsedIndex].m_lifeTime = 0;
  }

  btAssert(m_pointCache[lastUsedIndex].m_userPersistentData==0);
  m_cachedPoints--;
 }
 void replaceContactPoint(const btManifoldPoint& newPoint,int insertIndex)
 {
  btAssert(validContactDistance(newPoint));

#define MAINTAIN_PERSISTENCY 1
#ifdef MAINTAIN_PERSISTENCY
  int lifeTime = m_pointCache[insertIndex].getLifeTime();
  btScalar appliedImpulse = m_pointCache[insertIndex].mConstraintRow[0].m_accumImpulse;
  btScalar appliedLateralImpulse1 = m_pointCache[insertIndex].mConstraintRow[1].m_accumImpulse;
  btScalar appliedLateralImpulse2 = m_pointCache[insertIndex].mConstraintRow[2].m_accumImpulse;
//  bool isLateralFrictionInitialized = m_pointCache[insertIndex].m_lateralFrictionInitialized;
  
  
   
  btAssert(lifeTime>=0);
  void* cache = m_pointCache[insertIndex].m_userPersistentData;
  
  m_pointCache[insertIndex] = newPoint;

  m_pointCache[insertIndex].m_userPersistentData = cache;
  m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
  m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
  m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;
  
  m_pointCache[insertIndex].mConstraintRow[0].m_accumImpulse =  appliedImpulse;
  m_pointCache[insertIndex].mConstraintRow[1].m_accumImpulse = appliedLateralImpulse1;
  m_pointCache[insertIndex].mConstraintRow[2].m_accumImpulse = appliedLateralImpulse2;


  m_pointCache[insertIndex].m_lifeTime = lifeTime;
#else
  clearUserCache(m_pointCache[insertIndex]);
  m_pointCache[insertIndex] = newPoint;
 
#endif
 }

 bool validContactDistance(const btManifoldPoint& pt) const
 {
  if (pt.m_lifeTime >1)
  {
   return pt.m_distance1 <= getContactBreakingThreshold();
  }
  return pt.m_distance1 <= getContactProcessingThreshold();
 
 }
 /// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
 void refreshContactPoints(  const btTransform& trA,const btTransform& trB);

 
  void clearManifold()
 {
  int i;
  for (i=0;i<m_cachedPoints;i++)
  {
   clearUserCache(m_pointCache[i]);
  }
  m_cachedPoints = 0;
 }



}
;





#endif //BT_PERSISTENT_MANIFOLD_H
//// ../src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_MANIFOLD_CONTACT_POINT_H
#define BT_MANIFOLD_CONTACT_POINT_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransformUtil.h"

#ifdef PFX_USE_FREE_VECTORMATH
 #include "physics_effects/base_level/solver/pfx_constraint_row.h"
typedef sce::PhysicsEffects::PfxConstraintRow btConstraintRow;
#else
 // Don't change following order of parameters
 ATTRIBUTE_ALIGNED16(struct) btConstraintRow {
  btScalar m_normal[3];
  btScalar m_rhs;
  btScalar m_jacDiagInv;
  btScalar m_lowerLimit;
  btScalar m_upperLimit;
  btScalar m_accumImpulse;
 };
 typedef btConstraintRow PfxConstraintRow;
#endif //PFX_USE_FREE_VECTORMATH



/// ManifoldContactPoint collects and maintains persistent contactpoints.
/// used to improve stability and performance of rigidbody dynamics response.
class btManifoldPoint
 {
  public:
   btManifoldPoint()
    :m_userPersistentData(0),
    m_appliedImpulse(0.f),
    m_lateralFrictionInitialized(false),
    m_appliedImpulseLateral1(0.f),
    m_appliedImpulseLateral2(0.f),
    m_contactMotion1(0.f),
    m_contactMotion2(0.f),
    m_contactCFM1(0.f),
    m_contactCFM2(0.f),
    m_lifeTime(0)
   {
   }

   btManifoldPoint( const btVector3 &pointA, const btVector3 &pointB, 
     const btVector3 &normal, 
     btScalar distance ) :
     m_localPointA( pointA ), 
     m_localPointB( pointB ), 
     m_normalWorldOnB( normal ), 
     m_distance1( distance ),
     m_combinedFriction(btScalar(0.)),
     m_combinedRestitution(btScalar(0.)),
     m_userPersistentData(0),
     m_appliedImpulse(0.f),
     m_lateralFrictionInitialized(false),
     m_appliedImpulseLateral1(0.f),
     m_appliedImpulseLateral2(0.f),
     m_contactMotion1(0.f),
     m_contactMotion2(0.f),
     m_contactCFM1(0.f),
     m_contactCFM2(0.f),
     m_lifeTime(0)
   {
    mConstraintRow[0].m_accumImpulse = 0.f;
    mConstraintRow[1].m_accumImpulse = 0.f;
    mConstraintRow[2].m_accumImpulse = 0.f;
   }

   

   btVector3 m_localPointA;   
   btVector3 m_localPointB;   
   btVector3 m_positionWorldOnB;
   ///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
   btVector3 m_positionWorldOnA;
   btVector3 m_normalWorldOnB;
  
   btScalar m_distance1;
   btScalar m_combinedFriction;
   btScalar m_combinedRestitution;

         //BP mod, store contact triangles.
         int    m_partId0;
         int      m_partId1;
         int      m_index0;
         int      m_index1;
    
   mutable void* m_userPersistentData;
   btScalar  m_appliedImpulse;

   bool   m_lateralFrictionInitialized;
   btScalar  m_appliedImpulseLateral1;
   btScalar  m_appliedImpulseLateral2;
   btScalar  m_contactMotion1;
   btScalar  m_contactMotion2;
   btScalar  m_contactCFM1;
   btScalar  m_contactCFM2;

   int    m_lifeTime;//lifetime of the contactpoint in frames
   
   btVector3  m_lateralFrictionDir1;
   btVector3  m_lateralFrictionDir2;



   btConstraintRow mConstraintRow[3];


   btScalar getDistance() const
   {
    return m_distance1;
   }
   int getLifeTime() const
   {
    return m_lifeTime;
   }

   const btVector3& getPositionWorldOnA() const {
    return m_positionWorldOnA;
//    return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
   }

   const btVector3& getPositionWorldOnB() const
   {
    return m_positionWorldOnB;
   }

   void setDistance(btScalar dist)
   {
    m_distance1 = dist;
   }
   
   ///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
   btScalar getAppliedImpulse() const
   {
    return m_appliedImpulse;
   }

   

 };

#endif //BT_MANIFOLD_CONTACT_POINT_H
//// ../src/LinearMath/btTransformUtil.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_TRANSFORM_UTIL_H
#define BT_TRANSFORM_UTIL_H

#include "btTransform.h"
#define ANGULAR_MOTION_THRESHOLD btScalar(0.5)*SIMD_HALF_PI




 btVector3 btAabbSupport(const btVector3& halfExtents,const btVector3& supportDir)
{
 return btVector3(supportDir.x() < btScalar(0.0) ? -halfExtents.x() : halfExtents.x(),
      supportDir.y() < btScalar(0.0) ? -halfExtents.y() : halfExtents.y(),
      supportDir.z() < btScalar(0.0) ? -halfExtents.z() : halfExtents.z()); 
}






/// Utils related to temporal transforms
class btTransformUtil
{

public:

 static void integrateTransform(const btTransform& curTrans,const btVector3& linvel,const btVector3& angvel,btScalar timeStep,btTransform& predictedTransform)
 {
  predictedTransform.setOrigin(curTrans.getOrigin() + linvel * timeStep);
// #define QUATERNION_DERIVATIVE
 #ifdef QUATERNION_DERIVATIVE
  btQuaternion predictedOrn = curTrans.getRotation();
  predictedOrn += (angvel * predictedOrn) * (timeStep * btScalar(0.5));
  predictedOrn.normalize();
 #else
  //Exponential map
  //google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

  btVector3 axis;
  btScalar fAngle = angvel.length(); 
  //limit the angular motion
  if (fAngle*timeStep > ANGULAR_MOTION_THRESHOLD)
  {
   fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
  }

  if ( fAngle < btScalar(0.001) )
  {
   // use Taylor's expansions of sync function
   axis   = angvel*( btScalar(0.5)*timeStep-(timeStep*timeStep*timeStep)*(btScalar(0.020833333333))*fAngle*fAngle );
  }
  else
  {
   // sync(fAngle) = sin(c*fAngle)/t
   axis   = angvel*( btSin(btScalar(0.5)*fAngle*timeStep)/fAngle );
  }
  btQuaternion dorn (axis.x(),axis.y(),axis.z(),btCos( fAngle*timeStep*btScalar(0.5) ));
  btQuaternion orn0 = curTrans.getRotation();

  btQuaternion predictedOrn = dorn * orn0;
  predictedOrn.normalize();
 #endif
  predictedTransform.setRotation(predictedOrn);
 }

 static void calculateVelocityQuaternion(const btVector3& pos0,const btVector3& pos1,const btQuaternion& orn0,const btQuaternion& orn1,btScalar timeStep,btVector3& linVel,btVector3& angVel)
 {
  linVel = (pos1 - pos0) / timeStep;
  btVector3 axis;
  btScalar  angle;
  if (orn0 != orn1)
  {
   calculateDiffAxisAngleQuaternion(orn0,orn1,axis,angle);
   angVel = axis * angle / timeStep;
  } else
  {
   angVel.setValue(0,0,0);
  }
 }

 static void calculateDiffAxisAngleQuaternion(const btQuaternion& orn0,const btQuaternion& orn1a,btVector3& axis,btScalar& angle)
 {
  btQuaternion orn1 = orn0.nearest(orn1a);
  btQuaternion dorn = orn1 * orn0.inverse();
  angle = dorn.getAngle();
  axis = btVector3(dorn.x(),dorn.y(),dorn.z());
  axis[3] = btScalar(0.);
  //check for axis length
  btScalar len = axis.length2();
  if (len < SIMD_EPSILON*SIMD_EPSILON)
   axis = btVector3(btScalar(1.),btScalar(0.),btScalar(0.));
  else
   axis /= btSqrt(len);
 }

 static void calculateVelocity(const btTransform& transform0,const btTransform& transform1,btScalar timeStep,btVector3& linVel,btVector3& angVel)
 {
  linVel = (transform1.getOrigin() - transform0.getOrigin()) / timeStep;
  btVector3 axis;
  btScalar  angle;
  calculateDiffAxisAngle(transform0,transform1,axis,angle);
  angVel = axis * angle / timeStep;
 }

 static void calculateDiffAxisAngle(const btTransform& transform0,const btTransform& transform1,btVector3& axis,btScalar& angle)
 {
  btMatrix3x3 dmat = transform1.getBasis() * transform0.getBasis().inverse();
  btQuaternion dorn;
  dmat.getRotation(dorn);

  ///floating point inaccuracy can lead to w component > 1..., which breaks 
  dorn.normalize();
  
  angle = dorn.getAngle();
  axis = btVector3(dorn.x(),dorn.y(),dorn.z());
  axis[3] = btScalar(0.);
  //check for axis length
  btScalar len = axis.length2();
  if (len < SIMD_EPSILON*SIMD_EPSILON)
   axis = btVector3(btScalar(1.),btScalar(0.),btScalar(0.));
  else
   axis /= btSqrt(len);
 }

};


///The btConvexSeparatingDistanceUtil can help speed up convex collision detection 
///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
class btConvexSeparatingDistanceUtil
{
 btQuaternion m_ornA;
 btQuaternion m_ornB;
 btVector3 m_posA;
 btVector3 m_posB;
 
 btVector3 m_separatingNormal;

 btScalar m_boundingRadiusA;
 btScalar m_boundingRadiusB;
 btScalar m_separatingDistance;

public:

 btConvexSeparatingDistanceUtil(btScalar boundingRadiusA,btScalar boundingRadiusB)
  :m_boundingRadiusA(boundingRadiusA),
  m_boundingRadiusB(boundingRadiusB),
  m_separatingDistance(0.f)
 {
 }

 btScalar getConservativeSeparatingDistance()
 {
  return m_separatingDistance;
 }

 void updateSeparatingDistance(const btTransform& transA,const btTransform& transB)
 {
  const btVector3& toPosA = transA.getOrigin();
  const btVector3& toPosB = transB.getOrigin();
  btQuaternion toOrnA = transA.getRotation();
  btQuaternion toOrnB = transB.getRotation();

  if (m_separatingDistance>0.f)
  {
   

   btVector3 linVelA,angVelA,linVelB,angVelB;
   btTransformUtil::calculateVelocityQuaternion(m_posA,toPosA,m_ornA,toOrnA,btScalar(1.),linVelA,angVelA);
   btTransformUtil::calculateVelocityQuaternion(m_posB,toPosB,m_ornB,toOrnB,btScalar(1.),linVelB,angVelB);
   btScalar maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
   btVector3 relLinVel = (linVelB-linVelA);
   btScalar relLinVelocLength = relLinVel.dot(m_separatingNormal);
   if (relLinVelocLength<0.f)
   {
    relLinVelocLength = 0.f;
   }
 
   btScalar projectedMotion = maxAngularProjectedVelocity +relLinVelocLength;
   m_separatingDistance -= projectedMotion;
  }
 
  m_posA = toPosA;
  m_posB = toPosB;
  m_ornA = toOrnA;
  m_ornB = toOrnB;
 }

 void initSeparatingDistance(const btVector3& separatingVector,btScalar separatingDistance,const btTransform& transA,const btTransform& transB)
 {
  m_separatingDistance = separatingDistance;

  if (m_separatingDistance>0.f)
  {
   m_separatingNormal = separatingVector;
   
   const btVector3& toPosA = transA.getOrigin();
   const btVector3& toPosB = transB.getOrigin();
   btQuaternion toOrnA = transA.getRotation();
   btQuaternion toOrnB = transB.getRotation();
   m_posA = toPosA;
   m_posB = toPosB;
   m_ornA = toOrnA;
   m_ornB = toOrnB;
  }
 }

};


#endif //BT_TRANSFORM_UTIL_H

//// ../src/BulletCollision/CollisionDispatch/btManifoldResult.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_MANIFOLD_RESULT_H
#define BT_MANIFOLD_RESULT_H

class btCollisionObject;
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btManifoldPoint;

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"

#include "LinearMath/btTransform.h"

typedef bool (*ContactAddedCallback)(btManifoldPoint& cp, const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1);
extern ContactAddedCallback  gContactAddedCallback;

// XXX Emscripten: Uncomment the next line, to prevent SAFE_HEAP warnings
#define DEBUG_PART_INDEX 1


///btManifoldResult is a helper class to manage  contact results.
class btManifoldResult : public btDiscreteCollisionDetectorInterface__Result
{
protected:

 btPersistentManifold* m_manifoldPtr;

 //we need this for compounds
 btTransform m_rootTransA;
 btTransform m_rootTransB;

 btCollisionObject* m_body0;
 btCollisionObject* m_body1;
 int m_partId0;
 int m_partId1;
 int m_index0;
 int m_index1;
 

public:

 btManifoldResult()
#ifdef DEBUG_PART_INDEX
  :
 m_partId0(-1),
 m_partId1(-1),
 m_index0(-1),
 m_index1(-1)
#endif //DEBUG_PART_INDEX
 {
 }

 btManifoldResult(btCollisionObject* body0,btCollisionObject* body1);

 virtual ~btManifoldResult() {};

 void setPersistentManifold(btPersistentManifold* manifoldPtr)
 {
  m_manifoldPtr = manifoldPtr;
 }

 const btPersistentManifold* getPersistentManifold() const
 {
  return m_manifoldPtr;
 }
 btPersistentManifold* getPersistentManifold()
 {
  return m_manifoldPtr;
 }

 virtual void setShapeIdentifiersA(int partId0,int index0)
 {
  m_partId0=partId0;
  m_index0=index0;
 }

 virtual void setShapeIdentifiersB( int partId1,int index1)
 {
  m_partId1=partId1;
  m_index1=index1;
 }


 virtual void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth);

  void refreshContactPoints()
 {
  btAssert(m_manifoldPtr);
  if (!m_manifoldPtr->getNumContacts())
   return;

  bool isSwapped = m_manifoldPtr->getBody0() != m_body0;

  if (isSwapped)
  {
   m_manifoldPtr->refreshContactPoints(m_rootTransB,m_rootTransA);
  } else
  {
   m_manifoldPtr->refreshContactPoints(m_rootTransA,m_rootTransB);
  }
 }

 const btCollisionObject* getBody0Internal() const
 {
  return m_body0;
 }

 const btCollisionObject* getBody1Internal() const
 {
  return m_body1;
 }
 
};

#endif //BT_MANIFOLD_RESULT_H
//// ../src/BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_DISCRETE_COLLISION_DETECTOR1_INTERFACE_H
#define BT_DISCRETE_COLLISION_DETECTOR1_INTERFACE_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
class btStackAlloc;

/// This interface is made to be used by an iterative approach to do TimeOfImpact calculations
/// This interface allows to query for closest points and penetration depth between two (convex) objects
/// the closest point is on the second object (B), and the normal points from the surface on B towards A.
/// distance is between closest points on B and closest point on A. So you can calculate closest point on A
/// by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB
class btDiscreteCollisionDetectorInterface
{ public: 
 
 struct Result
 {
 
  virtual ~Result(){} 

  ///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
  virtual void setShapeIdentifiersA(int partId0,int index0)=0;
  virtual void setShapeIdentifiersB(int partId1,int index1)=0;
  virtual void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth)=0;
 };

 struct ClosestPointInput
 {
  ClosestPointInput()
   :m_maximumDistanceSquared(btScalar(BT_LARGE_FLOAT)),
   m_stackAlloc(0)
  {
  }

  btTransform m_transformA;
  btTransform m_transformB;
  btScalar m_maximumDistanceSquared;
  btStackAlloc* m_stackAlloc;
 };

 virtual ~btDiscreteCollisionDetectorInterface() {};

 //
 // give either closest points (distance > 0) or penetration (distance)
 // the normal always points from B towards A
 //
 virtual void getClosestPoints(const ClosestPointInput& input,Result& output,class btIDebugDraw* debugDraw,bool swapResults=false) = 0;

};

class btStorageResult : public btDiscreteCollisionDetectorInterface__Result
{ public: 
  btVector3 m_normalOnSurfaceB;
  btVector3 m_closestPointInB;
  btScalar m_distance; //negative means penetration !

  btStorageResult() : m_distance(btScalar(BT_LARGE_FLOAT))
  {

  }
  virtual ~btStorageResult() {};

  virtual void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth)
  {
   if (depth < m_distance)
   {
    m_normalOnSurfaceB = normalOnBInWorld;
    m_closestPointInB = pointInWorld;
    m_distance = depth;
   }
  }
};

#endif //BT_DISCRETE_COLLISION_DETECTOR1_INTERFACE_H

//// ../src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_BROADPHASE_PROXY_H
#define BT_BROADPHASE_PROXY_H

#include "LinearMath/btScalar.h" //for 
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedAllocator.h"


/// btDispatcher uses these types
/// IMPORTANT NOTE:The types are ordered polyhedral, implicit convex and concave
/// to facilitate type checking
/// CUSTOM_POLYHEDRAL_SHAPE_TYPE,CUSTOM_CONVEX_SHAPE_TYPE and CUSTOM_CONCAVE_SHAPE_TYPE can be used to extend Bullet without modifying source code
enum BroadphaseNativeTypes
{
 // polyhedral convex shapes
 BOX_SHAPE_PROXYTYPE,
 TRIANGLE_SHAPE_PROXYTYPE,
 TETRAHEDRAL_SHAPE_PROXYTYPE,
 CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
 CONVEX_HULL_SHAPE_PROXYTYPE,
 CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
 CUSTOM_POLYHEDRAL_SHAPE_TYPE,
//implicit convex shapes
IMPLICIT_CONVEX_SHAPES_START_HERE,
 SPHERE_SHAPE_PROXYTYPE,
 MULTI_SPHERE_SHAPE_PROXYTYPE,
 CAPSULE_SHAPE_PROXYTYPE,
 CONE_SHAPE_PROXYTYPE,
 CONVEX_SHAPE_PROXYTYPE,
 CYLINDER_SHAPE_PROXYTYPE,
 UNIFORM_SCALING_SHAPE_PROXYTYPE,
 MINKOWSKI_SUM_SHAPE_PROXYTYPE,
 MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
 BOX_2D_SHAPE_PROXYTYPE,
 CONVEX_2D_SHAPE_PROXYTYPE,
 CUSTOM_CONVEX_SHAPE_TYPE,
//concave shapes
CONCAVE_SHAPES_START_HERE,
 //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
 TRIANGLE_MESH_SHAPE_PROXYTYPE,
 SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
 ///used for demo integration FAST/Swift collision library and Bullet
 FAST_CONCAVE_MESH_PROXYTYPE,
 //terrain
 TERRAIN_SHAPE_PROXYTYPE,
///Used for GIMPACT Trimesh integration
 GIMPACT_SHAPE_PROXYTYPE,
///Multimaterial mesh
    MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,
 
 EMPTY_SHAPE_PROXYTYPE,
 STATIC_PLANE_PROXYTYPE,
 CUSTOM_CONCAVE_SHAPE_TYPE,
CONCAVE_SHAPES_END_HERE,

 COMPOUND_SHAPE_PROXYTYPE,

 SOFTBODY_SHAPE_PROXYTYPE,
 HFFLUID_SHAPE_PROXYTYPE,
 HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
 INVALID_SHAPE_PROXYTYPE,

 MAX_BROADPHASE_COLLISION_TYPES
 
};


///The btBroadphaseProxy is the main class that can be used with the Bullet broadphases. 
///It stores collision shape type information, collision filter information and a client object, typically a btCollisionObject or btRigidBody.
ATTRIBUTE_ALIGNED16(struct) btBroadphaseProxy
{


 
 ///optional filtering to cull potential collisions
 enum CollisionFilterGroups
 {
         DefaultFilter = 1,
         StaticFilter = 2,
         KinematicFilter = 4,
         DebrisFilter = 8,
   SensorTrigger = 16,
   CharacterFilter = 32,
         AllFilter = -1 //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger
 };

 //Usually the client btCollisionObject or Rigidbody class
 void* m_clientObject;
 short int m_collisionFilterGroup;
 short int m_collisionFilterMask;
 void* m_multiSapParentProxy;  
 int   m_uniqueId;//m_uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.

 btVector3 m_aabbMin;
 btVector3 m_aabbMax;

  int getUid() const
 {
  return m_uniqueId;
 }

 //used for memory pools
 btBroadphaseProxy() :m_clientObject(0),m_multiSapParentProxy(0)
 {
 }

 btBroadphaseProxy(const btVector3& aabbMin,const btVector3& aabbMax,void* userPtr,short int collisionFilterGroup, short int collisionFilterMask,void* multiSapParentProxy=0)
  :m_clientObject(userPtr),
  m_collisionFilterGroup(collisionFilterGroup),
  m_collisionFilterMask(collisionFilterMask),
  m_aabbMin(aabbMin),
  m_aabbMax(aabbMax)
 {
  m_multiSapParentProxy = multiSapParentProxy;
 }

 

 static  bool isPolyhedral(int proxyType)
 {
  return (proxyType  < IMPLICIT_CONVEX_SHAPES_START_HERE);
 }

 static  bool isConvex(int proxyType)
 {
  return (proxyType < CONCAVE_SHAPES_START_HERE);
 }

 static  bool isNonMoving(int proxyType)
 {
  return (isConcave(proxyType) && !(proxyType==GIMPACT_SHAPE_PROXYTYPE));
 }

 static  bool isConcave(int proxyType)
 {
  return ((proxyType > CONCAVE_SHAPES_START_HERE) &&
   (proxyType < CONCAVE_SHAPES_END_HERE));
 }
 static  bool isCompound(int proxyType)
 {
  return (proxyType == COMPOUND_SHAPE_PROXYTYPE);
 }

 static  bool isSoftBody(int proxyType)
 {
  return (proxyType == SOFTBODY_SHAPE_PROXYTYPE);
 }

 static  bool isInfinite(int proxyType)
 {
  return (proxyType == STATIC_PLANE_PROXYTYPE);
 }

 static  bool isConvex2d(int proxyType)
 {
  return (proxyType == BOX_2D_SHAPE_PROXYTYPE) || (proxyType == CONVEX_2D_SHAPE_PROXYTYPE);
 }

 
}
;

class btCollisionAlgorithm;

struct btBroadphaseProxy;



///The btBroadphasePair class contains a pair of aabb-overlapping objects.
///A btDispatcher can search a btCollisionAlgorithm that performs exact/narrowphase collision detection on the actual collision shapes.
ATTRIBUTE_ALIGNED16(struct) btBroadphasePair
{
 btBroadphasePair ()
  :
 m_pProxy0(0),
  m_pProxy1(0),
  m_algorithm(0),
  m_internalInfo1(0)
 {
 }



 btBroadphasePair(const btBroadphasePair& other)
  :  m_pProxy0(other.m_pProxy0),
    m_pProxy1(other.m_pProxy1),
    m_algorithm(other.m_algorithm),
    m_internalInfo1(other.m_internalInfo1)
 {
 }
 btBroadphasePair(btBroadphaseProxy& proxy0,btBroadphaseProxy& proxy1)
 {

  //keep them sorted, so the std::set operations work
  if (proxy0.m_uniqueId < proxy1.m_uniqueId)
        { 
            m_pProxy0 = &proxy0; 
            m_pProxy1 = &proxy1; 
        }
        else 
        { 
   m_pProxy0 = &proxy1; 
            m_pProxy1 = &proxy0; 
        }

  m_algorithm = 0;
  m_internalInfo1 = 0;

 }
 
 btBroadphaseProxy* m_pProxy0;
 btBroadphaseProxy* m_pProxy1;
 
 mutable btCollisionAlgorithm* m_algorithm;
 union { void* m_internalInfo1; int m_internalTmpValue;};//don't use this data, it will be removed in future version.

};

/*
//comparison for set operation, see Solid DT_Encounter
 bool operator<(const btBroadphasePair& a, const btBroadphasePair& b) 
{ 
    return a.m_pProxy0 < b.m_pProxy0 || 
        (a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 < b.m_pProxy1); 
}
*/



class btBroadphasePairSortPredicate
{
 public:

  bool operator() ( const btBroadphasePair& a, const btBroadphasePair& b )
  {
   const int uidA0 = a.m_pProxy0 ? a.m_pProxy0->m_uniqueId : -1;
   const int uidB0 = b.m_pProxy0 ? b.m_pProxy0->m_uniqueId : -1;
   const int uidA1 = a.m_pProxy1 ? a.m_pProxy1->m_uniqueId : -1;
   const int uidB1 = b.m_pProxy1 ? b.m_pProxy1->m_uniqueId : -1;

    return uidA0 > uidB0 || 
    (a.m_pProxy0 == b.m_pProxy0 && uidA1 > uidB1) ||
    (a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 == b.m_pProxy1 && a.m_algorithm > b.m_algorithm); 
  }
};


 bool operator==(const btBroadphasePair& a, const btBroadphasePair& b) 
{
  return (a.m_pProxy0 == b.m_pProxy0) && (a.m_pProxy1 == b.m_pProxy1);
}


#endif //BT_BROADPHASE_PROXY_H

//// ../src/BulletCollision/CollisionDispatch/btCollisionCreateFunc.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_CREATE_FUNC
#define BT_COLLISION_CREATE_FUNC

#include "LinearMath/btAlignedObjectArray.h"
class btCollisionAlgorithm;
class btCollisionObject;

struct btCollisionAlgorithmConstructionInfo;

///Used by the btCollisionDispatcher to register and create instances for btCollisionAlgorithm
class btCollisionAlgorithmCreateFunc
{ public: 
 bool m_swapped;
 
 btCollisionAlgorithmCreateFunc()
  :m_swapped(false)
 {
 }
 virtual ~btCollisionAlgorithmCreateFunc(){};

 virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& , btCollisionObject* body0,btCollisionObject* body1)
 {
  
  (void)body0;
  (void)body1;
  return 0;
 }
};
#endif //BT_COLLISION_CREATE_FUNC

//// ../src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_OVERLAPPING_PAIR_CACHE_H
#define BT_OVERLAPPING_PAIR_CACHE_H


#include "btBroadphaseInterface.h"
#include "btBroadphaseProxy.h"
#include "btOverlappingPairCallback.h"

#include "LinearMath/btAlignedObjectArray.h"
class btDispatcher;

typedef btAlignedObjectArray<btBroadphasePair> btBroadphasePairArray;

class btOverlapCallback
{ public: 
 virtual ~btOverlapCallback()
 {}
 //return true for deletion of the pair
 virtual bool processOverlap(btBroadphasePair& pair) = 0;

};

class btOverlapFilterCallback
{ public: 
 virtual ~btOverlapFilterCallback()
 {}
 // return true when pairs need collision
 virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const = 0;
};







extern int gRemovePairs;
extern int gAddedPairs;
extern int gFindPairs;

const int BT_NULL_PAIR=0xffffffff;

///The btOverlappingPairCache provides an interface for overlapping pair management (add, remove, storage), used by the btBroadphaseInterface broadphases.
///The btHashedOverlappingPairCache and btSortedOverlappingPairCache classes are two implementations.
class btOverlappingPairCache : public btOverlappingPairCallback
{
public:
 virtual ~btOverlappingPairCache() {} // this is needed so we can get to the derived class destructor

 virtual btBroadphasePair* getOverlappingPairArrayPtr() = 0;
 
 virtual const btBroadphasePair* getOverlappingPairArrayPtr() const = 0;

 virtual btBroadphasePairArray& getOverlappingPairArray() = 0;

 virtual void cleanOverlappingPair(btBroadphasePair& pair,btDispatcher* dispatcher) = 0;

 virtual int getNumOverlappingPairs() const = 0;

 virtual void cleanProxyFromPairs(btBroadphaseProxy* proxy,btDispatcher* dispatcher) = 0;

 virtual void setOverlapFilterCallback(btOverlapFilterCallback* callback) = 0;

 virtual void processAllOverlappingPairs(btOverlapCallback*,btDispatcher* dispatcher) = 0;

 virtual btBroadphasePair* findPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) = 0;

 virtual bool hasDeferredRemoval() = 0;

 virtual void setInternalGhostPairCallback(btOverlappingPairCallback* ghostPairCallback)=0;

 virtual void sortOverlappingPairs(btDispatcher* dispatcher) = 0;


};

/// Hash-space based Pair Cache, thanks to Erin Catto, Box2D, http://www.box2d.org, and Pierre Terdiman, Codercorner, http://codercorner.com
class btHashedOverlappingPairCache : public btOverlappingPairCache
{
 btBroadphasePairArray m_overlappingPairArray;
 btOverlapFilterCallback* m_overlapFilterCallback;
 bool  m_blockedForChanges;


public:
 btHashedOverlappingPairCache();
 virtual ~btHashedOverlappingPairCache();

 
 void removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);

 virtual void* removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher);
 
  bool needsBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
 {
  if (m_overlapFilterCallback)
   return m_overlapFilterCallback->needBroadphaseCollision(proxy0,proxy1);

  bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
  collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
  
  return collides;
 }

 // Add a pair and return the new pair. If the pair already exists,
 // no new pair is created and the old one is returned.
 virtual btBroadphasePair*  addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
 {
  gAddedPairs++;

  if (!needsBroadphaseCollision(proxy0,proxy1))
   return 0;

  return internalAddPair(proxy0,proxy1);
 }

 

 void cleanProxyFromPairs(btBroadphaseProxy* proxy,btDispatcher* dispatcher);

 
 virtual void processAllOverlappingPairs(btOverlapCallback*,btDispatcher* dispatcher);

 virtual btBroadphasePair* getOverlappingPairArrayPtr()
 {
  return &m_overlappingPairArray[0];
 }

 const btBroadphasePair* getOverlappingPairArrayPtr() const
 {
  return &m_overlappingPairArray[0];
 }

 btBroadphasePairArray& getOverlappingPairArray()
 {
  return m_overlappingPairArray;
 }

 const btBroadphasePairArray& getOverlappingPairArray() const
 {
  return m_overlappingPairArray;
 }

 void cleanOverlappingPair(btBroadphasePair& pair,btDispatcher* dispatcher);



 btBroadphasePair* findPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1);

 int GetCount() const { return m_overlappingPairArray.size(); }
// btBroadphasePair* GetPairs() { return m_pairs; }

 btOverlapFilterCallback* getOverlapFilterCallback()
 {
  return m_overlapFilterCallback;
 }

 void setOverlapFilterCallback(btOverlapFilterCallback* callback)
 {
  m_overlapFilterCallback = callback;
 }

 int getNumOverlappingPairs() const
 {
  return m_overlappingPairArray.size();
 }
private:
 
 btBroadphasePair*  internalAddPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

 void growTables();

  bool equalsPair(const btBroadphasePair& pair, int proxyId1, int proxyId2)
 { 
  return pair.m_pProxy0->getUid() == proxyId1 && pair.m_pProxy1->getUid() == proxyId2;
 }

 /*
 // Thomas Wang's hash, see: http://www.concentric.net/~Ttwang/tech/inthash.htm
 // This assumes proxyId1 and proxyId2 are 16-bit.
  int getHash(int proxyId1, int proxyId2)
 {
  int key = (proxyId2 << 16) | proxyId1;
  key = ~key + (key << 15);
  key = key ^ (key >> 12);
  key = key + (key << 2);
  key = key ^ (key >> 4);
  key = key * 2057;
  key = key ^ (key >> 16);
  return key;
 }
 */


 
  unsigned int getHash(unsigned int proxyId1, unsigned int proxyId2)
 {
  int key = static_cast<int>(((unsigned int)proxyId1) | (((unsigned int)proxyId2) <<16));
  // Thomas Wang's hash

  key += ~(key << 15);
  key ^=  (key >> 10);
  key +=  (key << 3);
  key ^=  (key >> 6);
  key += ~(key << 11);
  key ^=  (key >> 16);
  return static_cast<unsigned int>(key);
 }
 




  btBroadphasePair* internalFindPair(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1, int hash)
 {
  int proxyId1 = proxy0->getUid();
  int proxyId2 = proxy1->getUid();
  

  int index = m_hashTable[hash];
  
  while( index != BT_NULL_PAIR && equalsPair(m_overlappingPairArray[index], proxyId1, proxyId2) == false)
  {
   index = m_next[index];
  }

  if ( index == BT_NULL_PAIR )
  {
   return NULL;
  }

  btAssert(index < m_overlappingPairArray.size());

  return &m_overlappingPairArray[index];
 }

 virtual bool hasDeferredRemoval()
 {
  return false;
 }

 virtual void setInternalGhostPairCallback(btOverlappingPairCallback* ghostPairCallback)
 {
  m_ghostPairCallback = ghostPairCallback;
 }

 virtual void sortOverlappingPairs(btDispatcher* dispatcher);
 

protected:
 
 btAlignedObjectArray<int> m_hashTable;
 btAlignedObjectArray<int> m_next;
 btOverlappingPairCallback* m_ghostPairCallback;
 
};




///btSortedOverlappingPairCache maintains the objects with overlapping AABB
///Typically managed by the Broadphase, Axis3Sweep or btSimpleBroadphase
class btSortedOverlappingPairCache : public btOverlappingPairCache
{
 protected:
  //avoid brute-force finding all the time
  btBroadphasePairArray m_overlappingPairArray;

  //during the dispatch, check that user doesn't destroy/create proxy
  bool  m_blockedForChanges;

  ///by default, do the removal during the pair traversal
  bool  m_hasDeferredRemoval;
  
  //if set, use the callback instead of the built in filter in needBroadphaseCollision
  btOverlapFilterCallback* m_overlapFilterCallback;

  btOverlappingPairCallback* m_ghostPairCallback;

 public:
   
  btSortedOverlappingPairCache(); 
  virtual ~btSortedOverlappingPairCache();

  virtual void processAllOverlappingPairs(btOverlapCallback*,btDispatcher* dispatcher);

  void* removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher);

  void cleanOverlappingPair(btBroadphasePair& pair,btDispatcher* dispatcher);
  
  btBroadphasePair* addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

  btBroadphasePair* findPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);
   
  
  void cleanProxyFromPairs(btBroadphaseProxy* proxy,btDispatcher* dispatcher);

  void removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);


  inline bool needsBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
  {
   if (m_overlapFilterCallback)
    return m_overlapFilterCallback->needBroadphaseCollision(proxy0,proxy1);

   bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
   collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   
   return collides;
  }
  
  btBroadphasePairArray& getOverlappingPairArray()
  {
   return m_overlappingPairArray;
  }

  const btBroadphasePairArray& getOverlappingPairArray() const
  {
   return m_overlappingPairArray;
  }

  


  btBroadphasePair* getOverlappingPairArrayPtr()
  {
   return &m_overlappingPairArray[0];
  }

  const btBroadphasePair* getOverlappingPairArrayPtr() const
  {
   return &m_overlappingPairArray[0];
  }

  int getNumOverlappingPairs() const
  {
   return m_overlappingPairArray.size();
  }
  
  btOverlapFilterCallback* getOverlapFilterCallback()
  {
   return m_overlapFilterCallback;
  }

  void setOverlapFilterCallback(btOverlapFilterCallback* callback)
  {
   m_overlapFilterCallback = callback;
  }

  virtual bool hasDeferredRemoval()
  {
   return m_hasDeferredRemoval;
  }

  virtual void setInternalGhostPairCallback(btOverlappingPairCallback* ghostPairCallback)
  {
   m_ghostPairCallback = ghostPairCallback;
  }

  virtual void sortOverlappingPairs(btDispatcher* dispatcher);
  

};



///btNullPairCache skips add/removal of overlapping pairs. Userful for benchmarking and unit testing.
class btNullPairCache : public btOverlappingPairCache
{

 btBroadphasePairArray m_overlappingPairArray;

public:

 virtual btBroadphasePair* getOverlappingPairArrayPtr()
 {
  return &m_overlappingPairArray[0];
 }
 const btBroadphasePair* getOverlappingPairArrayPtr() const
 {
  return &m_overlappingPairArray[0];
 }
 btBroadphasePairArray& getOverlappingPairArray()
 {
  return m_overlappingPairArray;
 }
 
 virtual void cleanOverlappingPair(btBroadphasePair& /*pair*/,btDispatcher* /*dispatcher*/)
 {

 }

 virtual int getNumOverlappingPairs() const
 {
  return 0;
 }

 virtual void cleanProxyFromPairs(btBroadphaseProxy* /*proxy*/,btDispatcher* /*dispatcher*/)
 {

 }

 virtual void setOverlapFilterCallback(btOverlapFilterCallback* /*callback*/)
 {
 }

 virtual void processAllOverlappingPairs(btOverlapCallback*,btDispatcher* /*dispatcher*/)
 {
 }

 virtual btBroadphasePair* findPair(btBroadphaseProxy* /*proxy0*/, btBroadphaseProxy* /*proxy1*/)
 {
  return 0;
 }

 virtual bool hasDeferredRemoval()
 {
  return true;
 }

 virtual void setInternalGhostPairCallback(btOverlappingPairCallback* /* ghostPairCallback */)
 {

 }

 virtual btBroadphasePair* addOverlappingPair(btBroadphaseProxy* /*proxy0*/,btBroadphaseProxy* /*proxy1*/)
 {
  return 0;
 }

 virtual void* removeOverlappingPair(btBroadphaseProxy* /*proxy0*/,btBroadphaseProxy* /*proxy1*/,btDispatcher* /*dispatcher*/)
 {
  return 0;
 }

 virtual void removeOverlappingPairsContainingProxy(btBroadphaseProxy* /*proxy0*/,btDispatcher* /*dispatcher*/)
 {
 }
 
 virtual void sortOverlappingPairs(btDispatcher* dispatcher)
 {
        (void) dispatcher;
 }


};


#endif //BT_OVERLAPPING_PAIR_CACHE_H


//// ../src/BulletCollision/BroadphaseCollision/btBroadphaseInterface.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef  BT_BROADPHASE_INTERFACE_H
#define  BT_BROADPHASE_INTERFACE_H



struct btDispatcherInfo;
class btDispatcher;
#include "btBroadphaseProxy.h"

class btOverlappingPairCache;



class btBroadphaseAabbCallback
{ public: 
 virtual ~btBroadphaseAabbCallback() {}
 virtual bool process(const btBroadphaseProxy* proxy) = 0;
};


class btBroadphaseRayCallback : public btBroadphaseAabbCallback
{ public: 
 ///added some cached data to accelerate ray-AABB tests
 btVector3  m_rayDirectionInverse;
 unsigned int m_signs[3];
 btScalar  m_lambda_max;

 virtual ~btBroadphaseRayCallback() {}
};

#include "LinearMath/btVector3.h"

///The btBroadphaseInterface class provides an interface to detect aabb-overlapping object pairs.
///Some implementations for this broadphase interface include btAxisSweep3, bt32BitAxisSweep3 and btDbvtBroadphase.
///The actual overlapping pair management, storage, adding and removing of pairs is dealt by the btOverlappingPairCache class.
class btBroadphaseInterface
{
public:
 virtual ~btBroadphaseInterface() {}

 virtual btBroadphaseProxy* createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy) =0;
 virtual void destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher)=0;
 virtual void setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher)=0;
 virtual void getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const =0;

 virtual void rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin, const btVector3& aabbMax) = 0;

 virtual void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback) = 0;

 ///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
 virtual void calculateOverlappingPairs(btDispatcher* dispatcher)=0;

 virtual btOverlappingPairCache* getOverlappingPairCache()=0;
 virtual const btOverlappingPairCache* getOverlappingPairCache() const =0;

 ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
 ///will add some transform later
 virtual void getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const =0;

 ///reset broadphase internal structures, to ensure determinism/reproducability
 virtual void resetPool(btDispatcher* dispatcher) { (void) dispatcher; };

 virtual void printStats() = 0;

};

#endif //BT_BROADPHASE_INTERFACE_H
//// ../src/BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef OVERLAPPING_PAIR_CALLBACK_H
#define OVERLAPPING_PAIR_CALLBACK_H

class btDispatcher;
struct  btBroadphasePair;

///The btOverlappingPairCallback class is an additional optional broadphase user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
class btOverlappingPairCallback
{
public:
 virtual ~btOverlappingPairCallback()
 {

 }
 
 virtual btBroadphasePair* addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) = 0;

 virtual void* removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher) = 0;

 virtual void removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy0,btDispatcher* dispatcher) = 0;

};

#endif //OVERLAPPING_PAIR_CALLBACK_H
//// ../src/BulletCollision/CollisionShapes/btBoxShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_OBB_BOX_MINKOWSKI_H
#define BT_OBB_BOX_MINKOWSKI_H

#include "btPolyhedralConvexShape.h"
#include "btCollisionMargin.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMinMax.h"

///The btBoxShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
class btBoxShape: public btPolyhedralConvexShape
{

 //btVector3 m_boxHalfExtents1; //use m_implicitShapeDimensions instead


public:

 btVector3 getHalfExtentsWithMargin() const
 {
  btVector3 halfExtents = getHalfExtentsWithoutMargin();
  btVector3 margin(getMargin(),getMargin(),getMargin());
  halfExtents += margin;
  return halfExtents;
 }
 
 const btVector3& getHalfExtentsWithoutMargin() const
 {
  return m_implicitShapeDimensions;//scaling is included, margin is not
 }
 

 virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
 {
  btVector3 halfExtents = getHalfExtentsWithoutMargin();
  btVector3 margin(getMargin(),getMargin(),getMargin());
  halfExtents += margin;
  
  return btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
   btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
   btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
 }

   btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const
 {
  const btVector3& halfExtents = getHalfExtentsWithoutMargin();
  
  return btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
   btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
   btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
 }

 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
 {
  const btVector3& halfExtents = getHalfExtentsWithoutMargin();
 
  for (int i=0;i<numVectors;i++)
  {
   const btVector3& vec = vectors[i];
   supportVerticesOut[i].setValue(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
    btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
    btFsels(vec.z(), halfExtents.z(), -halfExtents.z())); 
  }

 }


 btBoxShape( const btVector3& boxHalfExtents) 
  : btPolyhedralConvexShape()
 {
  m_shapeType = BOX_SHAPE_PROXYTYPE;
  btVector3 margin(getMargin(),getMargin(),getMargin());
  m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;
 };

 virtual void setMargin(btScalar collisionMargin)
 {
  //correct the m_implicitShapeDimensions for the margin
  btVector3 oldMargin(getMargin(),getMargin(),getMargin());
  btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
  
  btConvexInternalShape::setMargin(collisionMargin);
  btVector3 newMargin(getMargin(),getMargin(),getMargin());
  m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

 }
 virtual void setLocalScaling(const btVector3& scaling)
 {
  btVector3 oldMargin(getMargin(),getMargin(),getMargin());
  btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
  btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

  btConvexInternalShape::setLocalScaling(scaling);

  m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

 }

 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i ) const
 {
  //this plane might not be aligned...
  btVector4 plane ;
  getPlaneEquation(plane,i);
  planeNormal = btVector3(plane.getX(),plane.getY(),plane.getZ());
  planeSupport = localGetSupportingVertex(-planeNormal);
 }

 
 virtual int getNumPlanes() const
 {
  return 6;
 } 
 
 virtual int getNumVertices() const 
 {
  return 8;
 }

 virtual int getNumEdges() const
 {
  return 12;
 }


 virtual void getVertex(int i,btVector3& vtx) const
 {
  btVector3 halfExtents = getHalfExtentsWithMargin();

  vtx = btVector3(
    halfExtents.x() * (1-(i&1)) - halfExtents.x() * (i&1),
    halfExtents.y() * (1-((i&2)>>1)) - halfExtents.y() * ((i&2)>>1),
    halfExtents.z() * (1-((i&4)>>2)) - halfExtents.z() * ((i&4)>>2));
 }
 

 virtual void getPlaneEquation(btVector4& plane,int i) const
 {
  btVector3 halfExtents = getHalfExtentsWithoutMargin();

  switch (i)
  {
  case 0:
   plane.setValue(btScalar(1.),btScalar(0.),btScalar(0.),-halfExtents.x());
   break;
  case 1:
   plane.setValue(btScalar(-1.),btScalar(0.),btScalar(0.),-halfExtents.x());
   break;
  case 2:
   plane.setValue(btScalar(0.),btScalar(1.),btScalar(0.),-halfExtents.y());
   break;
  case 3:
   plane.setValue(btScalar(0.),btScalar(-1.),btScalar(0.),-halfExtents.y());
   break;
  case 4:
   plane.setValue(btScalar(0.),btScalar(0.),btScalar(1.),-halfExtents.z());
   break;
  case 5:
   plane.setValue(btScalar(0.),btScalar(0.),btScalar(-1.),-halfExtents.z());
   break;
  default:
   btAssert(0);
  }
 }

 
 virtual void getEdge(int i,btVector3& pa,btVector3& pb) const
 //virtual void getEdge(int i,Edge& edge) const
 {
  int edgeVert0 = 0;
  int edgeVert1 = 0;

  switch (i)
  {
  case 0:
    edgeVert0 = 0;
    edgeVert1 = 1;
   break;
  case 1:
    edgeVert0 = 0;
    edgeVert1 = 2;
   break;
  case 2:
   edgeVert0 = 1;
   edgeVert1 = 3;

   break;
  case 3:
   edgeVert0 = 2;
   edgeVert1 = 3;
   break;
  case 4:
   edgeVert0 = 0;
   edgeVert1 = 4;
   break;
  case 5:
   edgeVert0 = 1;
   edgeVert1 = 5;

   break;
  case 6:
   edgeVert0 = 2;
   edgeVert1 = 6;
   break;
  case 7:
   edgeVert0 = 3;
   edgeVert1 = 7;
   break;
  case 8:
   edgeVert0 = 4;
   edgeVert1 = 5;
   break;
  case 9:
   edgeVert0 = 4;
   edgeVert1 = 6;
   break;
  case 10:
   edgeVert0 = 5;
   edgeVert1 = 7;
   break;
  case 11:
   edgeVert0 = 6;
   edgeVert1 = 7;
   break;
  default:
   btAssert(0);

  }

  getVertex(edgeVert0,pa );
  getVertex(edgeVert1,pb );
 }




 
 virtual bool isInside(const btVector3& pt,btScalar tolerance) const
 {
  btVector3 halfExtents = getHalfExtentsWithoutMargin();

  //btScalar minDist = 2*tolerance;
  
  bool result = (pt.x() <= (halfExtents.x()+tolerance)) &&
      (pt.x() >= (-halfExtents.x()-tolerance)) &&
      (pt.y() <= (halfExtents.y()+tolerance)) &&
      (pt.y() >= (-halfExtents.y()-tolerance)) &&
      (pt.z() <= (halfExtents.z()+tolerance)) &&
      (pt.z() >= (-halfExtents.z()-tolerance));
  
  return result;
 }


 //debugging
 virtual const char* getName()const
 {
  return "Box";
 }

 virtual int  getNumPreferredPenetrationDirections() const
 {
  return 6;
 }
 
 virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const
 {
  switch (index)
  {
  case 0:
   penetrationVector.setValue(btScalar(1.),btScalar(0.),btScalar(0.));
   break;
  case 1:
   penetrationVector.setValue(btScalar(-1.),btScalar(0.),btScalar(0.));
   break;
  case 2:
   penetrationVector.setValue(btScalar(0.),btScalar(1.),btScalar(0.));
   break;
  case 3:
   penetrationVector.setValue(btScalar(0.),btScalar(-1.),btScalar(0.));
   break;
  case 4:
   penetrationVector.setValue(btScalar(0.),btScalar(0.),btScalar(1.));
   break;
  case 5:
   penetrationVector.setValue(btScalar(0.),btScalar(0.),btScalar(-1.));
   break;
  default:
   btAssert(0);
  }
 }

};


#endif //BT_OBB_BOX_MINKOWSKI_H


//// ../src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_POLYHEDRAL_CONVEX_SHAPE_H
#define BT_POLYHEDRAL_CONVEX_SHAPE_H

#include "LinearMath/btMatrix3x3.h"
#include "btConvexInternalShape.h"
class btConvexPolyhedron;


///The btPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
class btPolyhedralConvexShape : public btConvexInternalShape
{
 

protected:
 
 btConvexPolyhedron* m_polyhedron;

public:

 btPolyhedralConvexShape();

 virtual ~btPolyhedralConvexShape();

 ///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
 virtual bool initializePolyhedralFeatures();

 const btConvexPolyhedron* getConvexPolyhedron() const
 {
  return m_polyhedron;
 }

 //brute force implementations

 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
 
 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;
 
 
 virtual int getNumVertices() const = 0 ;
 virtual int getNumEdges() const = 0;
 virtual void getEdge(int i,btVector3& pa,btVector3& pb) const = 0;
 virtual void getVertex(int i,btVector3& vtx) const = 0;
 virtual int getNumPlanes() const = 0;
 virtual void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i ) const = 0;
// virtual int getIndex(int i) const = 0 ; 

 virtual bool isInside(const btVector3& pt,btScalar tolerance) const = 0;
 
};


///The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape
class btPolyhedralConvexAabbCachingShape : public btPolyhedralConvexShape
{

 btVector3 m_localAabbMin;
 btVector3 m_localAabbMax;
 bool  m_isLocalAabbValid;
  
protected:

 void setCachedLocalAabb (const btVector3& aabbMin, const btVector3& aabbMax)
 {
  m_isLocalAabbValid = true;
  m_localAabbMin = aabbMin;
  m_localAabbMax = aabbMax;
 }

 inline void getCachedLocalAabb (btVector3& aabbMin, btVector3& aabbMax) const
 {
  btAssert(m_isLocalAabbValid);
  aabbMin = m_localAabbMin;
  aabbMax = m_localAabbMax;
 }

public:

 btPolyhedralConvexAabbCachingShape();
 
 inline void getNonvirtualAabb(const btTransform& trans,btVector3& aabbMin,btVector3& aabbMax, btScalar margin) const
 {

  //lazy evaluation of local aabb
  btAssert(m_isLocalAabbValid);
  btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
 }

 virtual void setLocalScaling(const btVector3& scaling);

 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 void recalcLocalAabb();

};

#endif //BT_POLYHEDRAL_CONVEX_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btConvexInternalShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_INTERNAL_SHAPE_H
#define BT_CONVEX_INTERNAL_SHAPE_H

#include "btConvexShape.h"
#include "LinearMath/btAabbUtil2.h"


///The btConvexInternalShape is an internal base class, shared by most convex shape implementations.
class btConvexInternalShape : public btConvexShape
{

 protected:

 //local scaling. collisionMargin is not scaled !
 btVector3 m_localScaling;

 btVector3 m_implicitShapeDimensions;
 
 btScalar m_collisionMargin;

 btScalar m_padding;

 btConvexInternalShape();

public:

 

 virtual ~btConvexInternalShape()
 {

 }

 virtual btVector3 localGetSupportingVertex(const btVector3& vec)const;

 const btVector3& getImplicitShapeDimensions() const
 {
  return m_implicitShapeDimensions;
 }

 ///warning: use setImplicitShapeDimensions with care
 ///changing a collision shape while the body is in the world is not recommended,
 ///it is best to remove the body from the world, then make the change, and re-add it
 ///alternatively flush the contact points, see documentation for 'cleanProxyFromPairs'
 void setImplicitShapeDimensions(const btVector3& dimensions)
 {
  m_implicitShapeDimensions = dimensions;
 }

 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
 {
  getAabbSlow(t,aabbMin,aabbMax);
 }


 
 virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;


 virtual void setLocalScaling(const btVector3& scaling);
 virtual const btVector3& getLocalScaling() const 
 {
  return m_localScaling;
 }

 const btVector3& getLocalScalingNV() const 
 {
  return m_localScaling;
 }

 virtual void setMargin(btScalar margin)
 {
  m_collisionMargin = margin;
 }
 virtual btScalar getMargin() const
 {
  return m_collisionMargin;
 }

 btScalar getMarginNV() const
 {
  return m_collisionMargin;
 }

 virtual int  getNumPreferredPenetrationDirections() const
 {
  return 0;
 }
 
 virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const
 {
  (void)penetrationVector;
  (void)index;
  btAssert(0);
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

 
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btConvexInternalShapeData
{ public: 
 btCollisionShapeData m_collisionShapeData;

 btVector3FloatData m_localScaling;

 btVector3FloatData m_implicitShapeDimensions;
 
 float   m_collisionMargin;

 int m_padding;

};



 int btConvexInternalShape::calculateSerializeBufferSize() const
{
 return sizeof(btConvexInternalShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btConvexInternalShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btConvexInternalShapeData* shapeData = (btConvexInternalShapeData*) dataBuffer;
 btCollisionShape::serialize(&shapeData->m_collisionShapeData, serializer);

 m_implicitShapeDimensions.serializeFloat(shapeData->m_implicitShapeDimensions);
 m_localScaling.serializeFloat(shapeData->m_localScaling);
 shapeData->m_collisionMargin = float(m_collisionMargin);

 return "btConvexInternalShapeData";
}




///btConvexInternalAabbCachingShape adds local aabb caching for convex shapes, to avoid expensive bounding box calculations
class btConvexInternalAabbCachingShape : public btConvexInternalShape
{
 btVector3 m_localAabbMin;
 btVector3 m_localAabbMax;
 bool  m_isLocalAabbValid;
 
protected:
     
 btConvexInternalAabbCachingShape();
 
 void setCachedLocalAabb (const btVector3& aabbMin, const btVector3& aabbMax)
 {
  m_isLocalAabbValid = true;
  m_localAabbMin = aabbMin;
  m_localAabbMax = aabbMax;
 }

 inline void getCachedLocalAabb (btVector3& aabbMin, btVector3& aabbMax) const
 {
  btAssert(m_isLocalAabbValid);
  aabbMin = m_localAabbMin;
  aabbMax = m_localAabbMax;
 }

 inline void getNonvirtualAabb(const btTransform& trans,btVector3& aabbMin,btVector3& aabbMax, btScalar margin) const
 {

  //lazy evaluation of local aabb
  btAssert(m_isLocalAabbValid);
  btTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
 }
  
public:
  
 virtual void setLocalScaling(const btVector3& scaling);

 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 void recalcLocalAabb();

};

#endif //BT_CONVEX_INTERNAL_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btConvexShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_SHAPE_INTERFACE1
#define BT_CONVEX_SHAPE_INTERFACE1

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"
#include "LinearMath/btAlignedAllocator.h"

#define MAX_PREFERRED_PENETRATION_DIRECTIONS 10

/// The btConvexShape is an abstract shape interface, implemented by all convex shapes such as btBoxShape, btConvexHullShape etc.
/// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as btGjkPairDetector.
class btConvexShape : public btCollisionShape
{


public:

 

 btConvexShape ();

 virtual ~btConvexShape();

 virtual btVector3 localGetSupportingVertex(const btVector3& vec)const = 0;

 ////////
 #ifndef __SPU__
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const=0;
 #endif //#ifndef __SPU__

 btVector3 localGetSupportVertexWithoutMarginNonVirtual (const btVector3& vec) const;
 btVector3 localGetSupportVertexNonVirtual (const btVector3& vec) const;
 btScalar getMarginNonVirtual () const;
 void getAabbNonVirtual (const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;

 
 //notice that the vectors should be unit length
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const= 0;

 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const =0;

 virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const =0;

 virtual void setLocalScaling(const btVector3& scaling) =0;
 virtual const btVector3& getLocalScaling() const =0;

 virtual void setMargin(btScalar margin)=0;

 virtual btScalar getMargin() const=0;

 virtual int  getNumPreferredPenetrationDirections() const=0;
 
 virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const=0;


 
 
};



#endif //BT_CONVEX_SHAPE_INTERFACE1
//// ../src/BulletCollision/CollisionShapes/btCollisionShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_SHAPE_H
#define BT_COLLISION_SHAPE_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" //for the shape types
class btSerializer;


///The btCollisionShape class provides an interface for collision shapes that can be shared among btCollisionObjects.
class btCollisionShape
{
protected:
 int m_shapeType;
 void* m_userPointer;

public:

 btCollisionShape() : m_shapeType (INVALID_SHAPE_PROXYTYPE), m_userPointer(0)
 {
 }

 virtual ~btCollisionShape()
 {
 }

 ///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const =0;

 virtual void getBoundingSphere(btVector3& center,btScalar& radius) const;

 ///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
 virtual btScalar getAngularMotionDisc() const;

 virtual btScalar getContactBreakingThreshold(btScalar defaultContactThresholdFactor) const;


 ///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
 ///result is conservative
 void calculateTemporalAabb(const btTransform& curTrans,const btVector3& linvel,const btVector3& angvel,btScalar timeStep, btVector3& temporalAabbMin,btVector3& temporalAabbMax) const;



  bool isPolyhedral() const
 {
  return btBroadphaseProxy::isPolyhedral(getShapeType());
 }

  bool isConvex2d() const
 {
  return btBroadphaseProxy::isConvex2d(getShapeType());
 }

  bool isConvex() const
 {
  return btBroadphaseProxy::isConvex(getShapeType());
 }
  bool isNonMoving() const
 {
  return btBroadphaseProxy::isNonMoving(getShapeType());
 }
  bool isConcave() const
 {
  return btBroadphaseProxy::isConcave(getShapeType());
 }
  bool isCompound() const
 {
  return btBroadphaseProxy::isCompound(getShapeType());
 }

  bool isSoftBody() const
 {
  return btBroadphaseProxy::isSoftBody(getShapeType());
 }

 ///isInfinite is used to catch simulation error (aabb check)
  bool isInfinite() const
 {
  return btBroadphaseProxy::isInfinite(getShapeType());
 }

#ifndef __SPU__
 virtual void setLocalScaling(const btVector3& scaling) =0;
 virtual const btVector3& getLocalScaling() const =0;
 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const = 0;


//debugging support
 virtual const char* getName()const =0 ;
#endif //__SPU__

 
 int  getShapeType() const { return m_shapeType; }
 virtual void setMargin(btScalar margin) = 0;
 virtual btScalar getMargin() const = 0;

 
 ///optional user data pointer
 void setUserPointer(void*  userPtr)
 {
  m_userPointer = userPtr;
 }

 void* getUserPointer() const
 {
  return m_userPointer;
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

 virtual void serializeSingleShape(btSerializer* serializer) const;

}; 

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCollisionShapeData
{ public: 
 char *m_name;
 int  m_shapeType;
 char m_padding[4];
};

 int btCollisionShape::calculateSerializeBufferSize() const
{
 return sizeof(btCollisionShapeData);
}



#endif //BT_COLLISION_SHAPE_H

//// ../src/BulletCollision/CollisionShapes/btCollisionMargin.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_MARGIN_H
#define BT_COLLISION_MARGIN_H

//used by Gjk and some other algorithms

#define CONVEX_DISTANCE_MARGIN btScalar(0.04)// btScalar(0.1)//;//btScalar(0.01)



#endif //BT_COLLISION_MARGIN_H

//// ../src/LinearMath/btAabbUtil2.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_AABB_UTIL2
#define BT_AABB_UTIL2

#include "btTransform.h"
#include "btVector3.h"
#include "btMinMax.h"



 void AabbExpand (btVector3& aabbMin,
           btVector3& aabbMax,
           const btVector3& expansionMin,
           const btVector3& expansionMax)
{
 aabbMin = aabbMin + expansionMin;
 aabbMax = aabbMax + expansionMax;
}

/// conservative test for overlap between two aabbs
 bool TestPointAgainstAabb2(const btVector3 &aabbMin1, const btVector3 &aabbMax1,
        const btVector3 &point)
{
 bool overlap = true;
 overlap = (aabbMin1.getX() > point.getX() || aabbMax1.getX() < point.getX()) ? false : overlap;
 overlap = (aabbMin1.getZ() > point.getZ() || aabbMax1.getZ() < point.getZ()) ? false : overlap;
 overlap = (aabbMin1.getY() > point.getY() || aabbMax1.getY() < point.getY()) ? false : overlap;
 return overlap;
}


/// conservative test for overlap between two aabbs
 bool TestAabbAgainstAabb2(const btVector3 &aabbMin1, const btVector3 &aabbMax1,
        const btVector3 &aabbMin2, const btVector3 &aabbMax2)
{
 bool overlap = true;
 overlap = (aabbMin1.getX() > aabbMax2.getX() || aabbMax1.getX() < aabbMin2.getX()) ? false : overlap;
 overlap = (aabbMin1.getZ() > aabbMax2.getZ() || aabbMax1.getZ() < aabbMin2.getZ()) ? false : overlap;
 overlap = (aabbMin1.getY() > aabbMax2.getY() || aabbMax1.getY() < aabbMin2.getY()) ? false : overlap;
 return overlap;
}

/// conservative test for overlap between triangle and aabb
 bool TestTriangleAgainstAabb2(const btVector3 *vertices,
         const btVector3 &aabbMin, const btVector3 &aabbMax)
{
 const btVector3 &p1 = vertices[0];
 const btVector3 &p2 = vertices[1];
 const btVector3 &p3 = vertices[2];

 if (btMin(btMin(p1[0], p2[0]), p3[0]) > aabbMax[0]) return false;
 if (btMax(btMax(p1[0], p2[0]), p3[0]) < aabbMin[0]) return false;

 if (btMin(btMin(p1[2], p2[2]), p3[2]) > aabbMax[2]) return false;
 if (btMax(btMax(p1[2], p2[2]), p3[2]) < aabbMin[2]) return false;
  
 if (btMin(btMin(p1[1], p2[1]), p3[1]) > aabbMax[1]) return false;
 if (btMax(btMax(p1[1], p2[1]), p3[1]) < aabbMin[1]) return false;
 return true;
}


 int btOutcode(const btVector3& p,const btVector3& halfExtent) 
{
 return (p.getX()  < -halfExtent.getX() ? 0x01 : 0x0) |    
     (p.getX() >  halfExtent.getX() ? 0x08 : 0x0) |
     (p.getY() < -halfExtent.getY() ? 0x02 : 0x0) |    
     (p.getY() >  halfExtent.getY() ? 0x10 : 0x0) |
     (p.getZ() < -halfExtent.getZ() ? 0x4 : 0x0) |    
     (p.getZ() >  halfExtent.getZ() ? 0x20 : 0x0);
}



 bool btRayAabb2(const btVector3& rayFrom,
          const btVector3& rayInvDirection,
          const unsigned int raySign[3],
          const btVector3 bounds[2],
          btScalar& tmin,
          btScalar lambda_min,
          btScalar lambda_max)
{
 btScalar tmax, tymin, tymax, tzmin, tzmax;
 tmin = (bounds[raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
 tmax = (bounds[1-raySign[0]].getX() - rayFrom.getX()) * rayInvDirection.getX();
 tymin = (bounds[raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();
 tymax = (bounds[1-raySign[1]].getY() - rayFrom.getY()) * rayInvDirection.getY();

 if ( (tmin > tymax) || (tymin > tmax) )
  return false;

 if (tymin > tmin)
  tmin = tymin;

 if (tymax < tmax)
  tmax = tymax;

 tzmin = (bounds[raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();
 tzmax = (bounds[1-raySign[2]].getZ() - rayFrom.getZ()) * rayInvDirection.getZ();

 if ( (tmin > tzmax) || (tzmin > tmax) )
  return false;
 if (tzmin > tmin)
  tmin = tzmin;
 if (tzmax < tmax)
  tmax = tzmax;
 return ( (tmin < lambda_max) && (tmax > lambda_min) );
}

 bool btRayAabb(const btVector3& rayFrom, 
         const btVector3& rayTo, 
         const btVector3& aabbMin, 
         const btVector3& aabbMax,
       btScalar& param, btVector3& normal) 
{
 btVector3 aabbHalfExtent = (aabbMax-aabbMin)* btScalar(0.5);
 btVector3 aabbCenter = (aabbMax+aabbMin)* btScalar(0.5);
 btVector3 source = rayFrom - aabbCenter;
 btVector3 target = rayTo - aabbCenter;
 int sourceOutcode = btOutcode(source,aabbHalfExtent);
 int targetOutcode = btOutcode(target,aabbHalfExtent);
 if ((sourceOutcode & targetOutcode) == 0x0)
 {
  btScalar lambda_enter = btScalar(0.0);
  btScalar lambda_exit  = param;
  btVector3 r = target - source;
  int i;
  btScalar normSign = 1;
  btVector3 hitNormal(0,0,0);
  int bit=1;

  for (int j=0;j<2;j++)
  {
   for (i = 0; i != 3; ++i)
   {
    if (sourceOutcode & bit)
    {
     btScalar lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
     if (lambda_enter <= lambda)
     {
      lambda_enter = lambda;
      hitNormal.setValue(0,0,0);
      hitNormal[i] = normSign;
     }
    }
    else if (targetOutcode & bit) 
    {
     btScalar lambda = (-source[i] - aabbHalfExtent[i]*normSign) / r[i];
     btSetMin(lambda_exit, lambda);
    }
    bit<<=1;
   }
   normSign = btScalar(-1.);
  }
  if (lambda_enter <= lambda_exit)
  {
   param = lambda_enter;
   normal = hitNormal;
   return true;
  }
 }
 return false;
}



 void btTransformAabb(const btVector3& halfExtents, btScalar margin,const btTransform& t,btVector3& aabbMinOut,btVector3& aabbMaxOut)
{
 btVector3 halfExtentsWithMargin = halfExtents+btVector3(margin,margin,margin);
 btMatrix3x3 abs_b = t.getBasis().absolute();  
 btVector3 center = t.getOrigin();
 btVector3 extent = btVector3(abs_b[0].dot(halfExtentsWithMargin),
     abs_b[1].dot(halfExtentsWithMargin),
    abs_b[2].dot(halfExtentsWithMargin));
 aabbMinOut = center - extent;
 aabbMaxOut = center + extent;
}


 void btTransformAabb(const btVector3& localAabbMin,const btVector3& localAabbMax, btScalar margin,const btTransform& trans,btVector3& aabbMinOut,btVector3& aabbMaxOut)
{
  btAssert(localAabbMin.getX() <= localAabbMax.getX());
  btAssert(localAabbMin.getY() <= localAabbMax.getY());
  btAssert(localAabbMin.getZ() <= localAabbMax.getZ());
  btVector3 localHalfExtents = btScalar(0.5)*(localAabbMax-localAabbMin);
  localHalfExtents+=btVector3(margin,margin,margin);

  btVector3 localCenter = btScalar(0.5)*(localAabbMax+localAabbMin);
  btMatrix3x3 abs_b = trans.getBasis().absolute();  
  btVector3 center = trans(localCenter);
  btVector3 extent = btVector3(abs_b[0].dot(localHalfExtents),
      abs_b[1].dot(localHalfExtents),
     abs_b[2].dot(localHalfExtents));
  aabbMinOut = center-extent;
  aabbMaxOut = center+extent;
}

#define USE_BANCHLESS 1
#ifdef USE_BANCHLESS
 //This block replaces the block below and uses no branches, and replaces the 8 bit return with a 32 bit return for improved performance (~3x on XBox 360)
  unsigned testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
 {  
  return static_cast<unsigned int>(btSelect((unsigned)((aabbMin1[0] <= aabbMax2[0]) & (aabbMax1[0] >= aabbMin2[0])
   & (aabbMin1[2] <= aabbMax2[2]) & (aabbMax1[2] >= aabbMin2[2])
   & (aabbMin1[1] <= aabbMax2[1]) & (aabbMax1[1] >= aabbMin2[1])),
   1, 0));
 }
#else
  bool testQuantizedAabbAgainstQuantizedAabb(const unsigned short int* aabbMin1,const unsigned short int* aabbMax1,const unsigned short int* aabbMin2,const unsigned short int* aabbMax2)
 {
  bool overlap = true;
  overlap = (aabbMin1[0] > aabbMax2[0] || aabbMax1[0] < aabbMin2[0]) ? false : overlap;
  overlap = (aabbMin1[2] > aabbMax2[2] || aabbMax1[2] < aabbMin2[2]) ? false : overlap;
  overlap = (aabbMin1[1] > aabbMax2[1] || aabbMax1[1] < aabbMin2[1]) ? false : overlap;
  return overlap;
 }
#endif //USE_BANCHLESS

#endif //BT_AABB_UTIL2


//// ../src/BulletCollision/CollisionShapes/btSphereShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BT_SPHERE_MINKOWSKI_H
#define BT_SPHERE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btSphereShape implements an implicit sphere, centered around a local origin with radius.
class btSphereShape : public btConvexInternalShape

{
 
public:
 

 btSphereShape (btScalar radius) : btConvexInternalShape ()
 {
  m_shapeType = SPHERE_SHAPE_PROXYTYPE;
  m_implicitShapeDimensions.setX(radius);
  m_collisionMargin = radius;
 }
 
 virtual btVector3 localGetSupportingVertex(const btVector3& vec)const;
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
 //notice that the vectors should be unit length
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;


 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;


 btScalar getRadius() const { return m_implicitShapeDimensions.getX() * m_localScaling.getX();}

 void setUnscaledRadius(btScalar radius)
 {
  m_implicitShapeDimensions.setX(radius);
  btConvexInternalShape::setMargin(radius);
 }

 //debugging
 virtual const char* getName()const {return "SPHERE";}

 virtual void setMargin(btScalar margin)
 {
  btConvexInternalShape::setMargin(margin);
 }
 virtual btScalar getMargin() const
 {
  //to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
  //this means, non-uniform scaling is not supported anymore
  return getRadius();
 }


};


#endif //BT_SPHERE_MINKOWSKI_H
//// ../src/BulletCollision/CollisionShapes/btCapsuleShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CAPSULE_SHAPE_H
#define BT_CAPSULE_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


///The btCapsuleShape represents a capsule around the Y axis, there is also the btCapsuleShapeX aligned around the X axis and btCapsuleShapeZ around the Z axis.
///The total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
///The btCapsuleShape is a convex hull of two spheres. The btMultiSphereShape is a more general collision shape that takes the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.
class btCapsuleShape : public btConvexInternalShape
{
protected:
 int m_upAxis;

protected:
 ///only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.
 btCapsuleShape() : btConvexInternalShape() {m_shapeType = CAPSULE_SHAPE_PROXYTYPE;};

public:
 btCapsuleShape(btScalar radius,btScalar height);

 ///CollisionShape Interface
 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 /// btConvexShape Interface
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
 
 virtual void setMargin(btScalar collisionMargin)
 {
  //correct the m_implicitShapeDimensions for the margin
  btVector3 oldMargin(getMargin(),getMargin(),getMargin());
  btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
  
  btConvexInternalShape::setMargin(collisionMargin);
  btVector3 newMargin(getMargin(),getMargin(),getMargin());
  m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

 }

 virtual void getAabb (const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
 {
   btVector3 halfExtents(getRadius(),getRadius(),getRadius());
   halfExtents[m_upAxis] = getRadius() + getHalfHeight();
   halfExtents += btVector3(getMargin(),getMargin(),getMargin());
   btMatrix3x3 abs_b = t.getBasis().absolute();  
   btVector3 center = t.getOrigin();
   btVector3 extent = btVector3(abs_b[0].dot(halfExtents),abs_b[1].dot(halfExtents),abs_b[2].dot(halfExtents));    
   
   aabbMin = center - extent;
   aabbMax = center + extent;
 }

 virtual const char* getName()const 
 {
  return "CapsuleShape";
 }

 int getUpAxis() const
 {
  return m_upAxis;
 }

 btScalar getRadius() const
 {
  int radiusAxis = (m_upAxis+2)%3;
  return m_implicitShapeDimensions[radiusAxis];
 }

 btScalar getHalfHeight() const
 {
  return m_implicitShapeDimensions[m_upAxis];
 }

 virtual void setLocalScaling(const btVector3& scaling)
 {
  btVector3 oldMargin(getMargin(),getMargin(),getMargin());
  btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
  btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

  btConvexInternalShape::setLocalScaling(scaling);

  m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

///btCapsuleShapeX represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShapeX : public btCapsuleShape
{
public:

 btCapsuleShapeX(btScalar radius,btScalar height);
  
 //debugging
 virtual const char* getName()const
 {
  return "CapsuleX";
 }

 

};

///btCapsuleShapeZ represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btCapsuleShapeZ : public btCapsuleShape
{
public:
 btCapsuleShapeZ(btScalar radius,btScalar height);

  //debugging
 virtual const char* getName()const
 {
  return "CapsuleZ";
 }

 
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCapsuleShapeData
{ public: 
 btConvexInternalShapeData m_convexInternalShapeData;

 int m_upAxis;

 char m_padding[4];
};

 int btCapsuleShape::calculateSerializeBufferSize() const
{
 return sizeof(btCapsuleShapeData);
}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btCapsuleShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btCapsuleShapeData* shapeData = (btCapsuleShapeData*) dataBuffer;
 
 btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData,serializer);

 shapeData->m_upAxis = m_upAxis;
 
 return "btCapsuleShapeData";
}

#endif //BT_CAPSULE_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btCylinderShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CYLINDER_MINKOWSKI_H
#define BT_CYLINDER_MINKOWSKI_H

#include "btBoxShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btVector3.h"

/// The btCylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y axis. btCylinderShapeX is aligned with the X axis and btCylinderShapeZ around the Z axis.
class btCylinderShape : public btConvexInternalShape

{

protected:

 int m_upAxis;

public:

 btVector3 getHalfExtentsWithMargin() const
 {
  btVector3 halfExtents = getHalfExtentsWithoutMargin();
  btVector3 margin(getMargin(),getMargin(),getMargin());
  halfExtents += margin;
  return halfExtents;
 }
 
 const btVector3& getHalfExtentsWithoutMargin() const
 {
  return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
 }

 btCylinderShape (const btVector3& halfExtents);
 
 void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

 virtual void setMargin(btScalar collisionMargin)
 {
  //correct the m_implicitShapeDimensions for the margin
  btVector3 oldMargin(getMargin(),getMargin(),getMargin());
  btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
  
  btConvexInternalShape::setMargin(collisionMargin);
  btVector3 newMargin(getMargin(),getMargin(),getMargin());
  m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

 }

 virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
 {

  btVector3 supVertex;
  supVertex = localGetSupportingVertexWithoutMargin(vec);
  
  if ( getMargin()!=btScalar(0.) )
  {
   btVector3 vecnorm = vec;
   if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
   {
    vecnorm.setValue(btScalar(-1.),btScalar(-1.),btScalar(-1.));
   } 
   vecnorm.normalize();
   supVertex+= getMargin() * vecnorm;
  }
  return supVertex;
 }


 //use box inertia
 // virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;


 int getUpAxis() const
 {
  return m_upAxis;
 }

 virtual btScalar getRadius() const
 {
  return getHalfExtentsWithMargin().getX();
 }

 virtual void setLocalScaling(const btVector3& scaling)
 {
  btVector3 oldMargin(getMargin(),getMargin(),getMargin());
  btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
  btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

  btConvexInternalShape::setLocalScaling(scaling);

  m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

 }

 //debugging
 virtual const char* getName()const
 {
  return "CylinderY";
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

};

class btCylinderShapeX : public btCylinderShape
{
public:
 btCylinderShapeX (const btVector3& halfExtents);

 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
 
  //debugging
 virtual const char* getName()const
 {
  return "CylinderX";
 }

 virtual btScalar getRadius() const
 {
  return getHalfExtentsWithMargin().getY();
 }

};

class btCylinderShapeZ : public btCylinderShape
{
public:
 btCylinderShapeZ (const btVector3& halfExtents);

 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

  //debugging
 virtual const char* getName()const
 {
  return "CylinderZ";
 }

 virtual btScalar getRadius() const
 {
  return getHalfExtentsWithMargin().getX();
 }

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCylinderShapeData
{ public: 
 btConvexInternalShapeData m_convexInternalShapeData;

 int m_upAxis;

 char m_padding[4];
};

 int btCylinderShape::calculateSerializeBufferSize() const
{
 return sizeof(btCylinderShapeData);
}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btCylinderShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btCylinderShapeData* shapeData = (btCylinderShapeData*) dataBuffer;
 
 btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData,serializer);

 shapeData->m_upAxis = m_upAxis;
 
 return "btCylinderShapeData";
}



#endif //BT_CYLINDER_MINKOWSKI_H

//// ../src/BulletCollision/CollisionShapes/btConeShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONE_MINKOWSKI_H
#define BT_CONE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btConeShape implements a cone shape primitive, centered around the origin and aligned with the Y axis. The btConeShapeX is aligned around the X axis and btConeShapeZ around the Z axis.
class btConeShape : public btConvexInternalShape

{

 btScalar m_sinAngle;
 btScalar m_radius;
 btScalar m_height;
 int  m_coneIndices[3];
 btVector3 coneLocalSupport(const btVector3& v) const;


public:
 btConeShape (btScalar radius,btScalar height);
 
 virtual btVector3 localGetSupportingVertex(const btVector3& vec) const;
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const;
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

 btScalar getRadius() const { return m_radius;}
 btScalar getHeight() const { return m_height;}


 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const
 {
  btTransform identity;
  identity.setIdentity();
  btVector3 aabbMin,aabbMax;
  getAabb(identity,aabbMin,aabbMax);

  btVector3 halfExtents = (aabbMax-aabbMin)*btScalar(0.5);

  btScalar margin = getMargin();

  btScalar lx=btScalar(2.)*(halfExtents.x()+margin);
  btScalar ly=btScalar(2.)*(halfExtents.y()+margin);
  btScalar lz=btScalar(2.)*(halfExtents.z()+margin);
  const btScalar x2 = lx*lx;
  const btScalar y2 = ly*ly;
  const btScalar z2 = lz*lz;
  const btScalar scaledmass = mass * btScalar(0.08333333);

  inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

//  inertia.x() = scaledmass * (y2+z2);
//  inertia.y() = scaledmass * (x2+z2);
//  inertia.z() = scaledmass * (x2+y2);
 }


  virtual const char* getName()const 
  {
   return "Cone";
  }
  
  ///choose upAxis index
  void setConeUpIndex(int upIndex);
  
  int getConeUpIndex() const
  {
   return m_coneIndices[1];
  }

 virtual void setLocalScaling(const btVector3& scaling);

};

///btConeShape implements a Cone shape, around the X axis
class btConeShapeX : public btConeShape
{
 public:
  btConeShapeX(btScalar radius,btScalar height);
};

///btConeShapeZ implements a Cone shape, around the Z axis
class btConeShapeZ : public btConeShape
{
 public:
  btConeShapeZ(btScalar radius,btScalar height);
};
#endif //BT_CONE_MINKOWSKI_H

//// ../src/BulletCollision/CollisionShapes/btStaticPlaneShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_STATIC_PLANE_SHAPE_H
#define BT_STATIC_PLANE_SHAPE_H

#include "btConcaveShape.h"


///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
class btStaticPlaneShape : public btConcaveShape
{
protected:
 btVector3 m_localAabbMin;
 btVector3 m_localAabbMax;
 
 btVector3 m_planeNormal;
 btScalar      m_planeConstant;
 btVector3 m_localScaling;

public:
 btStaticPlaneShape(const btVector3& planeNormal,btScalar planeConstant);

 virtual ~btStaticPlaneShape();


 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 virtual void processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual void setLocalScaling(const btVector3& scaling);
 virtual const btVector3& getLocalScaling() const;
 
 const btVector3& getPlaneNormal() const
 {
  return m_planeNormal;
 }

 const btScalar& getPlaneConstant() const
 {
  return m_planeConstant;
 }

 //debugging
 virtual const char* getName()const {return "STATICPLANE";}

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btStaticPlaneShapeData
{ public: 
 btCollisionShapeData m_collisionShapeData;

 btVector3FloatData m_localScaling;
 btVector3FloatData m_planeNormal;
 float   m_planeConstant;
 char m_pad[4];
};


 int btStaticPlaneShape::calculateSerializeBufferSize() const
{
 return sizeof(btStaticPlaneShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btStaticPlaneShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btStaticPlaneShapeData* planeData = (btStaticPlaneShapeData*) dataBuffer;
 btCollisionShape::serialize(&planeData->m_collisionShapeData,serializer);

 m_localScaling.serializeFloat(planeData->m_localScaling);
 m_planeNormal.serializeFloat(planeData->m_planeNormal);
 planeData->m_planeConstant = float(m_planeConstant);
  
 return "btStaticPlaneShapeData";
}


#endif //BT_STATIC_PLANE_SHAPE_H



//// ../src/BulletCollision/CollisionShapes/btConcaveShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONCAVE_SHAPE_H
#define BT_CONCAVE_SHAPE_H

#include "btCollisionShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "btTriangleCallback.h"

/// PHY_ScalarType enumerates possible scalar types.
/// See the btStridingMeshInterface or btHeightfieldTerrainShape for its use
typedef enum PHY_ScalarType {
 PHY_FLOAT,
 PHY_DOUBLE,
 PHY_INTEGER,
 PHY_SHORT,
 PHY_FIXEDPOINT88,
 PHY_UCHAR
} PHY_ScalarType;

///The btConcaveShape class provides an interface for non-moving (static) concave shapes.
///It has been implemented by the btStaticPlaneShape, btBvhTriangleMeshShape and btHeightfieldTerrainShape.
class btConcaveShape : public btCollisionShape
{
protected:
 btScalar m_collisionMargin;

public:
 btConcaveShape();

 virtual ~btConcaveShape();

 virtual void processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const = 0;

 virtual btScalar getMargin() const {
  return m_collisionMargin;
 }
 virtual void setMargin(btScalar collisionMargin)
 {
  m_collisionMargin = collisionMargin;
 }



};

#endif //BT_CONCAVE_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btTriangleCallback.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TRIANGLE_CALLBACK_H
#define BT_TRIANGLE_CALLBACK_H

#include "LinearMath/btVector3.h"


///The btTriangleCallback provides a callback for each overlapping triangle when calling processAllTriangles.
///This callback is called by processAllTriangles for all btConcaveShape derived class, such as  btBvhTriangleMeshShape, btStaticPlaneShape and btHeightfieldTerrainShape.
class btTriangleCallback
{
public:

 virtual ~btTriangleCallback();
 virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex) = 0;
};

class btInternalTriangleIndexCallback
{
public:

 virtual ~btInternalTriangleIndexCallback();
 virtual void internalProcessTriangleIndex(btVector3* triangle,int partId,int  triangleIndex) = 0;
};



#endif //BT_TRIANGLE_CALLBACK_H
//// ../src/BulletCollision/CollisionShapes/btConvexHullShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_HULL_SHAPE_H
#define BT_CONVEX_HULL_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"


///The btConvexHullShape implements an implicit convex hull of an array of vertices.
///Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.
class btConvexHullShape : public btPolyhedralConvexAabbCachingShape
{
 btAlignedObjectArray<btVector3> m_unscaledPoints;

public:
 

 
 ///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive btScalar (x,y,z), the striding defines the number of bytes between each point, in memory.
 ///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
 ///btConvexHullShape make an internal copy of the points.
 btConvexHullShape(const btScalar* points=0,int numPoints=0, int stride=sizeof(btVector3));

 void addPoint(const btVector3& point);

 
 btVector3* getUnscaledPoints()
 {
  return &m_unscaledPoints[0];
 }

 const btVector3* getUnscaledPoints() const
 {
  return &m_unscaledPoints[0];
 }

 ///getPoints is obsolete, please use getUnscaledPoints
 const btVector3* getPoints() const
 {
  return getUnscaledPoints();
 }

 


  btVector3 getScaledPoint(int i) const
 {
  return m_unscaledPoints[i] * m_localScaling;
 }

  int getNumPoints() const 
 {
  return m_unscaledPoints.size();
 }

 virtual btVector3 localGetSupportingVertex(const btVector3& vec)const;
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
 


 //debugging
 virtual const char* getName()const {return "Convex";}

 
 virtual int getNumVertices() const;
 virtual int getNumEdges() const;
 virtual void getEdge(int i,btVector3& pa,btVector3& pb) const;
 virtual void getVertex(int i,btVector3& vtx) const;
 virtual int getNumPlanes() const;
 virtual void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i ) const;
 virtual bool isInside(const btVector3& pt,btScalar tolerance) const;

 ///in case we receive negative scaling
 virtual void setLocalScaling(const btVector3& scaling);

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btConvexHullShapeData
{ public: 
 btConvexInternalShapeData m_convexInternalShapeData;

 btVector3FloatData *m_unscaledPointsFloatPtr;
 btVector3DoubleData *m_unscaledPointsDoublePtr;

 int  m_numUnscaledPoints;
 char m_padding3[4];

};


 int btConvexHullShape::calculateSerializeBufferSize() const
{
 return sizeof(btConvexHullShapeData);
}


#endif //BT_CONVEX_HULL_SHAPE_H

//// ../src/BulletCollision/CollisionShapes/btTriangleMesh.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TRIANGLE_MESH_H
#define BT_TRIANGLE_MESH_H

#include "btTriangleIndexVertexArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

///The btTriangleMesh class is a convenience class derived from btTriangleIndexVertexArray, that provides storage for a concave triangle mesh. It can be used as data for the btBvhTriangleMeshShape.
///It allows either 32bit or 16bit indices, and 4 (x-y-z-w) or 3 (x-y-z) component vertices.
///If you want to share triangle/index data between graphics mesh and collision mesh (btBvhTriangleMeshShape), you can directly use btTriangleIndexVertexArray or derive your own class from btStridingMeshInterface.
///Performance of btTriangleMesh and btTriangleIndexVertexArray used in a btBvhTriangleMeshShape is the same.
class btTriangleMesh : public btTriangleIndexVertexArray
{
 btAlignedObjectArray<btVector3> m_4componentVertices;
 btAlignedObjectArray<float>  m_3componentVertices;

 btAlignedObjectArray<unsigned int>  m_32bitIndices;
 btAlignedObjectArray<unsigned short int>  m_16bitIndices;
 bool m_use32bitIndices;
 bool m_use4componentVertices;
 

 public:
  btScalar m_weldingThreshold;

  btTriangleMesh (bool use32bitIndices=true,bool use4componentVertices=true);

  bool getUse32bitIndices() const
  {
   return m_use32bitIndices;
  }

  bool getUse4componentVertices() const
  {
   return m_use4componentVertices;
  }
  ///By default addTriangle won't search for duplicate vertices, because the search is very slow for large triangle meshes.
  ///In general it is better to directly use btTriangleIndexVertexArray instead.
  void addTriangle(const btVector3& vertex0,const btVector3& vertex1,const btVector3& vertex2, bool removeDuplicateVertices=false);
  
  int getNumTriangles() const;

  virtual void preallocateVertices(int numverts){(void) numverts;}
  virtual void preallocateIndices(int numindices){(void) numindices;}

  ///findOrAddVertex is an internal method, use addTriangle instead
  int  findOrAddVertex(const btVector3& vertex, bool removeDuplicateVertices);
  ///addIndex is an internal method, use addTriangle instead
  void addIndex(int index);
  
};

#endif //BT_TRIANGLE_MESH_H

//// ../src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TRIANGLE_INDEX_VERTEX_ARRAY_H
#define BT_TRIANGLE_INDEX_VERTEX_ARRAY_H

#include "btStridingMeshInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btScalar.h"


///The btIndexedMesh indexes a single vertex and index array. Multiple btIndexedMesh objects can be passed into a btTriangleIndexVertexArray using addIndexedMesh.
///Instead of the number of indices, we pass the number of triangles.
ATTRIBUTE_ALIGNED16( struct) btIndexedMesh
{
 

   int                     m_numTriangles;
   const unsigned char *   m_triangleIndexBase;
   int                     m_triangleIndexStride;
   int                     m_numVertices;
   const unsigned char *   m_vertexBase;
   int                     m_vertexStride;

   // The index type is set when adding an indexed mesh to the
   // btTriangleIndexVertexArray, do not set it manually
   PHY_ScalarType m_indexType;

   // The vertex type has a default type similar to Bullet's precision mode (float or double)
   // but can be set manually if you for example run Bullet with double precision but have
   // mesh data in single precision..
   PHY_ScalarType m_vertexType;


   btIndexedMesh()
    :m_indexType(PHY_INTEGER),
#ifdef BT_USE_DOUBLE_PRECISION
      m_vertexType(PHY_DOUBLE)
#else // BT_USE_DOUBLE_PRECISION
      m_vertexType(PHY_FLOAT)
#endif // BT_USE_DOUBLE_PRECISION
      {
      }
}
;


typedef btAlignedObjectArray<btIndexedMesh> IndexedMeshArray;

///The btTriangleIndexVertexArray allows to access multiple triangle meshes, by indexing into existing triangle/index arrays.
///Additional meshes can be added using addIndexedMesh
///No duplcate is made of the vertex/index data, it only indexes into external vertex/index arrays.
///So keep those arrays around during the lifetime of this btTriangleIndexVertexArray.
class btTriangleIndexVertexArray : public btStridingMeshInterface
{
protected:
 IndexedMeshArray m_indexedMeshes;
 int m_pad[2];
 mutable int m_hasAabb; // using int instead of bool to maintain alignment
 mutable btVector3 m_aabbMin;
 mutable btVector3 m_aabbMax;

public:

 

 btTriangleIndexVertexArray() : m_hasAabb(0)
 {
 }

 virtual ~btTriangleIndexVertexArray();

 //just to be backwards compatible
 btTriangleIndexVertexArray(int numTriangles,int* triangleIndexBase,int triangleIndexStride,int numVertices,btScalar* vertexBase,int vertexStride);
 
 void addIndexedMesh(const btIndexedMesh& mesh, PHY_ScalarType indexType = PHY_INTEGER)
 {
  m_indexedMeshes.push_back(mesh);
  m_indexedMeshes[m_indexedMeshes.size()-1].m_indexType = indexType;
 }
 
 
 virtual void getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& vertexStride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0);

 virtual void getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& vertexStride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0) const;

 /// unLockVertexBase finishes the access to a subpart of the triangle mesh
 /// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
 virtual void unLockVertexBase(int subpart) {(void)subpart;}

 virtual void unLockReadOnlyVertexBase(int subpart) const {(void)subpart;}

 /// getNumSubParts returns the number of seperate subparts
 /// each subpart has a continuous array of vertices and indices
 virtual int  getNumSubParts() const { 
  return (int)m_indexedMeshes.size();
 }

 IndexedMeshArray& getIndexedMeshArray()
 {
  return m_indexedMeshes;
 }

 const IndexedMeshArray& getIndexedMeshArray() const
 {
  return m_indexedMeshes;
 }

 virtual void preallocateVertices(int numverts){(void) numverts;}
 virtual void preallocateIndices(int numindices){(void) numindices;}

 virtual bool hasPremadeAabb() const;
 virtual void setPremadeAabb(const btVector3& aabbMin, const btVector3& aabbMax ) const;
 virtual void getPremadeAabb(btVector3* aabbMin, btVector3* aabbMax ) const;

}
;

#endif //BT_TRIANGLE_INDEX_VERTEX_ARRAY_H
//// ../src/BulletCollision/CollisionShapes/btStridingMeshInterface.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_STRIDING_MESHINTERFACE_H
#define BT_STRIDING_MESHINTERFACE_H

#include "LinearMath/btVector3.h"
#include "btTriangleCallback.h"
#include "btConcaveShape.h"





/// The btStridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in combination with btBvhTriangleMeshShape and some other collision shapes.
/// Using index striding of 3*sizeof(integer) it can use triangle arrays, using index striding of 1*sizeof(integer) it can handle triangle strips.
/// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.
class  btStridingMeshInterface
{
 protected:
 
  btVector3 m_scaling;

 public:
  btStridingMeshInterface() :m_scaling(btScalar(1.),btScalar(1.),btScalar(1.))
  {

  }

  virtual ~btStridingMeshInterface();



  virtual void InternalProcessAllTriangles(btInternalTriangleIndexCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

  ///brute force method to calculate aabb
  void calculateAabbBruteForce(btVector3& aabbMin,btVector3& aabbMax);

  /// get read and write access to a subpart of a triangle mesh
  /// this subpart has a continuous array of vertices and indices
  /// in this way the mesh can be handled as chunks of memory with striding
  /// very similar to OpenGL vertexarray support
  /// make a call to unLockVertexBase when the read and write access is finished 
  virtual void getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0)=0;
  
  virtual void getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0) const=0;
 
  /// unLockVertexBase finishes the access to a subpart of the triangle mesh
  /// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
  virtual void unLockVertexBase(int subpart)=0;

  virtual void unLockReadOnlyVertexBase(int subpart) const=0;


  /// getNumSubParts returns the number of seperate subparts
  /// each subpart has a continuous array of vertices and indices
  virtual int  getNumSubParts() const=0;

  virtual void preallocateVertices(int numverts)=0;
  virtual void preallocateIndices(int numindices)=0;

  virtual bool hasPremadeAabb() const { return false; }
  virtual void setPremadeAabb(const btVector3& aabbMin, const btVector3& aabbMax ) const
                {
                        (void) aabbMin;
                        (void) aabbMax;
                }
  virtual void getPremadeAabb(btVector3* aabbMin, btVector3* aabbMax ) const
        {
            (void) aabbMin;
            (void) aabbMax;
        }

  const btVector3& getScaling() const {
   return m_scaling;
  }
  void setScaling(const btVector3& scaling)
  {
   m_scaling = scaling;
  }

  virtual int calculateSerializeBufferSize() const;

  ///fills the dataBuffer and returns the struct name (and 0 on failure)
  virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

class btIntIndexData
{ public: 
 int m_value;
};

class btShortIntIndexData
{ public: 
 short m_value;
 char m_pad[2];
};

class btShortIntIndexTripletData
{ public: 
 short m_values[3];
 char m_pad[2];
};

class btCharIndexTripletData
{ public: 
 unsigned char m_values[3];
 char m_pad;
};


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btMeshPartData
{ public: 
 btVector3FloatData   *m_vertices3f;
 btVector3DoubleData   *m_vertices3d;

 btIntIndexData    *m_indices32;
 btShortIntIndexTripletData *m_3indices16;
 btCharIndexTripletData  *m_3indices8;

 btShortIntIndexData   *m_indices16;//backwards compatibility

 int                     m_numTriangles;//length of m_indices = m_numTriangles
 int                     m_numVertices;
};


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btStridingMeshInterfaceData
{ public: 
 btMeshPartData *m_meshPartsPtr;
 btVector3FloatData m_scaling;
 int m_numMeshParts;
 char m_padding[4];
};




 int btStridingMeshInterface::calculateSerializeBufferSize() const
{
 return sizeof(btStridingMeshInterfaceData);
}



#endif //BT_STRIDING_MESHINTERFACE_H
//// ../src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BT_CONVEX_TRIANGLEMESH_SHAPE_H
#define BT_CONVEX_TRIANGLEMESH_SHAPE_H


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


/// The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
/// A small benefit of this class is that it uses the btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing btConvexHullShape instead.
class btConvexTriangleMeshShape : public btPolyhedralConvexAabbCachingShape
{

 class btStridingMeshInterface* m_stridingMesh;

public:
 btConvexTriangleMeshShape(btStridingMeshInterface* meshInterface, bool calcAabb = true);

 class btStridingMeshInterface* getMeshInterface()
 {
  return m_stridingMesh;
 }
 const class btStridingMeshInterface* getMeshInterface() const
 {
  return m_stridingMesh;
 }
 
 virtual btVector3 localGetSupportingVertex(const btVector3& vec)const;
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;
 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
 
 //debugging
 virtual const char* getName()const {return "ConvexTrimesh";}
 
 virtual int getNumVertices() const;
 virtual int getNumEdges() const;
 virtual void getEdge(int i,btVector3& pa,btVector3& pb) const;
 virtual void getVertex(int i,btVector3& vtx) const;
 virtual int getNumPlanes() const;
 virtual void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i ) const;
 virtual bool isInside(const btVector3& pt,btScalar tolerance) const;

 
 virtual void setLocalScaling(const btVector3& scaling);
 virtual const btVector3& getLocalScaling() const;

 ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
 ///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
 ///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
 ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
 ///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
 void calculatePrincipalAxisTransform(btTransform& principal, btVector3& inertia, btScalar& volume) const;

};



#endif //BT_CONVEX_TRIANGLEMESH_SHAPE_H



//// ../src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_BVH_TRIANGLE_MESH_SHAPE_H

#include "btTriangleMeshShape.h"
#include "btOptimizedBvh.h"
#include "LinearMath/btAlignedAllocator.h"
#include "btTriangleInfoMap.h"

///The btBvhTriangleMeshShape is a static-triangle mesh shape with several optimizations, such as bounding volume hierarchy and cache friendly traversal for PlayStation 3 Cell SPU. It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
class btBvhTriangleMeshShape : public btTriangleMeshShape
{

 btOptimizedBvh* m_bvh;
 btTriangleInfoMap* m_triangleInfoMap;

 bool m_useQuantizedAabbCompression;
 bool m_ownsBvh;
 bool m_pad[11];////need padding due to alignment

public:

 

 
 btBvhTriangleMeshShape(btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true);

 ///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
 btBvhTriangleMeshShape(btStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression,const btVector3& bvhAabbMin,const btVector3& bvhAabbMax, bool buildBvh = true);
 
 virtual ~btBvhTriangleMeshShape();

 bool getOwnsBvh () const
 {
  return m_ownsBvh;
 }


 
 void performRaycast (btTriangleCallback* callback, const btVector3& raySource, const btVector3& rayTarget);
 void performConvexcast (btTriangleCallback* callback, const btVector3& boxSource, const btVector3& boxTarget, const btVector3& boxMin, const btVector3& boxMax);

 virtual void processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

 void refitTree(const btVector3& aabbMin,const btVector3& aabbMax);

 ///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
 void partialRefitTree(const btVector3& aabbMin,const btVector3& aabbMax);

 //debugging
 virtual const char* getName()const {return "BVHTRIANGLEMESH";}


 virtual void setLocalScaling(const btVector3& scaling);
 
 btOptimizedBvh* getOptimizedBvh()
 {
  return m_bvh;
 }

 void setOptimizedBvh(btOptimizedBvh* bvh, const btVector3& localScaling);

 void    buildOptimizedBvh();

 bool usesQuantizedAabbCompression() const
 {
  return m_useQuantizedAabbCompression;
 }

 void setTriangleInfoMap(btTriangleInfoMap* triangleInfoMap)
 {
  m_triangleInfoMap = triangleInfoMap;
 }

 const btTriangleInfoMap* getTriangleInfoMap() const
 {
  return m_triangleInfoMap;
 }
 
 btTriangleInfoMap* getTriangleInfoMap()
 {
  return m_triangleInfoMap;
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

 virtual void serializeSingleBvh(btSerializer* serializer) const;

 virtual void serializeSingleTriangleInfoMap(btSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btTriangleMeshShapeData
{ public: 
 btCollisionShapeData m_collisionShapeData;

 btStridingMeshInterfaceData m_meshInterface;

 btQuantizedBvhFloatData  *m_quantizedFloatBvh;
 btQuantizedBvhDoubleData *m_quantizedDoubleBvh;

 btTriangleInfoMapData *m_triangleInfoMap;
 
 float m_collisionMargin;

 char m_pad3[4];
 
};


 int btBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
 return sizeof(btTriangleMeshShapeData);
}



#endif //BT_BVH_TRIANGLE_MESH_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btTriangleMeshShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TRIANGLE_MESH_SHAPE_H
#define BT_TRIANGLE_MESH_SHAPE_H

#include "btConcaveShape.h"
#include "btStridingMeshInterface.h"


///The btTriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use btBvhTriangleMeshShape instead.
class btTriangleMeshShape : public btConcaveShape
{
protected:
 btVector3 m_localAabbMin;
 btVector3 m_localAabbMax;
 btStridingMeshInterface* m_meshInterface;

 ///btTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
 ///Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
 btTriangleMeshShape(btStridingMeshInterface* meshInterface);

public:

 virtual ~btTriangleMeshShape();

 virtual btVector3 localGetSupportingVertex(const btVector3& vec) const;

 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const
 {
  btAssert(0);
  return localGetSupportingVertex(vec);
 }

 void recalcLocalAabb();

 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 virtual void processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual void setLocalScaling(const btVector3& scaling);
 virtual const btVector3& getLocalScaling() const;
 
 btStridingMeshInterface* getMeshInterface()
 {
  return m_meshInterface;
 }

 const btStridingMeshInterface* getMeshInterface() const
 {
  return m_meshInterface;
 }

 const btVector3& getLocalAabbMin() const
 {
  return m_localAabbMin;
 }
 const btVector3& getLocalAabbMax() const
 {
  return m_localAabbMax;
 }



 //debugging
 virtual const char* getName()const {return "TRIANGLEMESH";}

 

};




#endif //BT_TRIANGLE_MESH_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btOptimizedBvh.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///Contains contributions from Disney Studio's

#ifndef BT_OPTIMIZED_BVH_H
#define BT_OPTIMIZED_BVH_H

#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h"

class btStridingMeshInterface;


///The btOptimizedBvh extends the btQuantizedBvh to create AABB tree for triangle meshes, through the btStridingMeshInterface.
class btOptimizedBvh : public btQuantizedBvh
{
 
public:
 

protected:

public:

 btOptimizedBvh();

 virtual ~btOptimizedBvh();

 void build(btStridingMeshInterface* triangles,bool useQuantizedAabbCompression, const btVector3& bvhAabbMin, const btVector3& bvhAabbMax);

 void refit(btStridingMeshInterface* triangles,const btVector3& aabbMin,const btVector3& aabbMax);

 void refitPartial(btStridingMeshInterface* triangles,const btVector3& aabbMin, const btVector3& aabbMax);

 void updateBvhNodes(btStridingMeshInterface* meshInterface,int firstNode,int endNode,int index);

 /// Data buffer MUST be 16 byte aligned
 virtual bool serializeInPlace(void *o_alignedDataBuffer, unsigned i_dataBufferSize, bool i_swapEndian) const
 {
  return btQuantizedBvh::serialize(o_alignedDataBuffer,i_dataBufferSize,i_swapEndian);

 }

 ///deSerializeInPlace loads and initializes a BVH from a buffer in memory 'in place'
 static btOptimizedBvh *deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian);


};


#endif //BT_OPTIMIZED_BVH_H


//// ../src/BulletCollision/BroadphaseCollision/btQuantizedBvh.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_QUANTIZED_BVH_H
#define BT_QUANTIZED_BVH_H

class btSerializer;

//#define DEBUG_CHECK_DEQUANTIZATION 1
#ifdef DEBUG_CHECK_DEQUANTIZATION
#ifdef __SPU__
#define printf spu_printf
#endif //__SPU__

#include <stdio.h>
#include <stdlib.h>
#endif //DEBUG_CHECK_DEQUANTIZATION

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedAllocator.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btQuantizedBvhData btQuantizedBvhDoubleData
#define btOptimizedBvhNodeData btOptimizedBvhNodeDoubleData
#define btQuantizedBvhDataName "btQuantizedBvhDoubleData"
#else
#define btQuantizedBvhData btQuantizedBvhFloatData
#define btOptimizedBvhNodeData btOptimizedBvhNodeFloatData
#define btQuantizedBvhDataName "btQuantizedBvhFloatData"
#endif



//http://msdn.microsoft.com/library/default.asp?url=/library/en-us/vclang/html/vclrf__m128.asp


//Note: currently we have 16 bytes per quantized node
#define MAX_SUBTREE_SIZE_IN_BYTES  2048

// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
// actually) triangles each (since the sign bit is reserved
#define MAX_NUM_PARTS_IN_BITS 10

///btQuantizedBvhNode is a compressed aabb node, 16 bytes.
///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).
ATTRIBUTE_ALIGNED16 (struct) btQuantizedBvhNode
{
 

 //12 bytes
 unsigned short int m_quantizedAabbMin[3];
 unsigned short int m_quantizedAabbMax[3];
 //4 bytes
 int m_escapeIndexOrTriangleIndex;

 bool isLeafNode() const
 {
  //skipindex is negative (internal node), triangleindex >=0 (leafnode)
  return (m_escapeIndexOrTriangleIndex >= 0);
 }
 int getEscapeIndex() const
 {
  btAssert(!isLeafNode());
  return -m_escapeIndexOrTriangleIndex;
 }
 int getTriangleIndex() const
 {
  btAssert(isLeafNode());
  // Get only the lower bits where the triangle index is stored
  return (m_escapeIndexOrTriangleIndex&~((~0)<<(31-MAX_NUM_PARTS_IN_BITS)));
 }
 int getPartId() const
 {
  btAssert(isLeafNode());
  // Get only the highest bits where the part index is stored
  return (m_escapeIndexOrTriangleIndex>>(31-MAX_NUM_PARTS_IN_BITS));
 }
}
;

/// btOptimizedBvhNode contains both internal and leaf node information.
/// Total node size is 44 bytes / node. You can use the compressed version of 16 bytes.
ATTRIBUTE_ALIGNED16 (struct) btOptimizedBvhNode
{
 

 //32 bytes
 btVector3 m_aabbMinOrg;
 btVector3 m_aabbMaxOrg;

 //4
 int m_escapeIndex;

 //8
 //for child nodes
 int m_subPart;
 int m_triangleIndex;
 int m_padding[5];//bad, due to alignment


};


///btBvhSubtreeInfo provides info to gather a subtree of limited size
class btBvhSubtreeInfo
{
public:
 

 //12 bytes
 unsigned short int m_quantizedAabbMin[3];
 unsigned short int m_quantizedAabbMax[3];
 //4 bytes, points to the root of the subtree
 int   m_rootNodeIndex;
 //4 bytes
 int   m_subtreeSize;
 int   m_padding[3];

 btBvhSubtreeInfo()
 {
  //memset(&m_padding[0], 0, sizeof(m_padding));
 }


 void setAabbFromQuantizeNode(const btQuantizedBvhNode& quantizedNode)
 {
  m_quantizedAabbMin[0] = quantizedNode.m_quantizedAabbMin[0];
  m_quantizedAabbMin[1] = quantizedNode.m_quantizedAabbMin[1];
  m_quantizedAabbMin[2] = quantizedNode.m_quantizedAabbMin[2];
  m_quantizedAabbMax[0] = quantizedNode.m_quantizedAabbMax[0];
  m_quantizedAabbMax[1] = quantizedNode.m_quantizedAabbMax[1];
  m_quantizedAabbMax[2] = quantizedNode.m_quantizedAabbMax[2];
 }
}
;


class btNodeOverlapCallback
{
public:
 virtual ~btNodeOverlapCallback() {};

 virtual void processNode(int subPart, int triangleIndex) = 0;
};

#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btAlignedObjectArray.h"



///for code readability:
typedef btAlignedObjectArray<btOptimizedBvhNode> NodeArray;
typedef btAlignedObjectArray<btQuantizedBvhNode> QuantizedNodeArray;
typedef btAlignedObjectArray<btBvhSubtreeInfo>  BvhSubtreeInfoArray;


///The btQuantizedBvh class stores an AABB tree that can be quickly traversed on CPU and Cell SPU.
///It is used by the btBvhTriangleMeshShape as midphase, and by the btMultiSapBroadphase.
///It is recommended to use quantization for better performance and lower memory requirements.
class btQuantizedBvh
{
public:
 enum btTraversalMode
 {
  TRAVERSAL_STACKLESS = 0,
  TRAVERSAL_STACKLESS_CACHE_FRIENDLY,
  TRAVERSAL_RECURSIVE
 };

protected:


 btVector3   m_bvhAabbMin;
 btVector3   m_bvhAabbMax;
 btVector3   m_bvhQuantization;

 int     m_bulletVersion; //for serialization versioning. It could also be used to detect endianess.

 int     m_curNodeIndex;
 //quantization data
 bool    m_useQuantization;



 NodeArray   m_leafNodes;
 NodeArray   m_contiguousNodes;
 QuantizedNodeArray m_quantizedLeafNodes;
 QuantizedNodeArray m_quantizedContiguousNodes;
 
 btTraversalMode m_traversalMode;
 BvhSubtreeInfoArray  m_SubtreeHeaders;

 //This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray
 mutable int m_subtreeHeaderCount;

 



 ///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
 ///this might be refactored into a virtual, it is usually not calculated at run-time
 void setInternalNodeAabbMin(int nodeIndex, const btVector3& aabbMin)
 {
  if (m_useQuantization)
  {
   quantize(&m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] ,aabbMin,0);
  } else
  {
   m_contiguousNodes[nodeIndex].m_aabbMinOrg = aabbMin;

  }
 }
 void setInternalNodeAabbMax(int nodeIndex,const btVector3& aabbMax)
 {
  if (m_useQuantization)
  {
   quantize(&m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0],aabbMax,1);
  } else
  {
   m_contiguousNodes[nodeIndex].m_aabbMaxOrg = aabbMax;
  }
 }

 btVector3 getAabbMin(int nodeIndex) const
 {
  if (m_useQuantization)
  {
   return unQuantize(&m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin[0]);
  }
  //non-quantized
  return m_leafNodes[nodeIndex].m_aabbMinOrg;

 }
 btVector3 getAabbMax(int nodeIndex) const
 {
  if (m_useQuantization)
  {
   return unQuantize(&m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax[0]);
  } 
  //non-quantized
  return m_leafNodes[nodeIndex].m_aabbMaxOrg;
  
 }

 
 void setInternalNodeEscapeIndex(int nodeIndex, int escapeIndex)
 {
  if (m_useQuantization)
  {
   m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
  } 
  else
  {
   m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
  }

 }

 void mergeInternalNodeAabb(int nodeIndex,const btVector3& newAabbMin,const btVector3& newAabbMax) 
 {
  if (m_useQuantization)
  {
   unsigned short int quantizedAabbMin[3];
   unsigned short int quantizedAabbMax[3];
   quantize(quantizedAabbMin,newAabbMin,0);
   quantize(quantizedAabbMax,newAabbMax,1);
   for (int i=0;i<3;i++)
   {
    if (m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] > quantizedAabbMin[i])
     m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[i] = quantizedAabbMin[i];

    if (m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] < quantizedAabbMax[i])
     m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[i] = quantizedAabbMax[i];

   }
  } else
  {
   //non-quantized
   m_contiguousNodes[nodeIndex].m_aabbMinOrg.setMin(newAabbMin);
   m_contiguousNodes[nodeIndex].m_aabbMaxOrg.setMax(newAabbMax);  
  }
 }

 void swapLeafNodes(int firstIndex,int secondIndex);

 void assignInternalNodeFromLeafNode(int internalNode,int leafNodeIndex);

protected:

 

 void buildTree (int startIndex,int endIndex);

 int calcSplittingAxis(int startIndex,int endIndex);

 int sortAndCalcSplittingIndex(int startIndex,int endIndex,int splitAxis);
 
 void walkStacklessTree(btNodeOverlapCallback* nodeCallback,const btVector3& aabbMin,const btVector3& aabbMax) const;

 void walkStacklessQuantizedTreeAgainstRay(btNodeOverlapCallback* nodeCallback, const btVector3& raySource, const btVector3& rayTarget, const btVector3& aabbMin, const btVector3& aabbMax, int startNodeIndex,int endNodeIndex) const;
 void walkStacklessQuantizedTree(btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,int startNodeIndex,int endNodeIndex) const;
 void walkStacklessTreeAgainstRay(btNodeOverlapCallback* nodeCallback, const btVector3& raySource, const btVector3& rayTarget, const btVector3& aabbMin, const btVector3& aabbMax, int startNodeIndex,int endNodeIndex) const;

 ///tree traversal designed for small-memory processors like PS3 SPU
 void walkStacklessQuantizedTreeCacheFriendly(btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const;

 ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
 void walkRecursiveQuantizedTreeAgainstQueryAabb(const btQuantizedBvhNode* currentNode,btNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const;

 ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
 void walkRecursiveQuantizedTreeAgainstQuantizedTree(const btQuantizedBvhNode* treeNodeA,const btQuantizedBvhNode* treeNodeB,btNodeOverlapCallback* nodeCallback) const;
 



 void updateSubtreeHeaders(int leftChildNodexIndex,int rightChildNodexIndex);

public:
 
 

 btQuantizedBvh();

 virtual ~btQuantizedBvh();

 
 ///***************************************** expert/internal use only *************************
 void setQuantizationValues(const btVector3& bvhAabbMin,const btVector3& bvhAabbMax,btScalar quantizationMargin=btScalar(1.0));
 QuantizedNodeArray& getLeafNodeArray() {   return m_quantizedLeafNodes; }
 ///buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized
 void buildInternal();
 ///***************************************** expert/internal use only *************************

 void reportAabbOverlappingNodex(btNodeOverlapCallback* nodeCallback,const btVector3& aabbMin,const btVector3& aabbMax) const;
 void reportRayOverlappingNodex (btNodeOverlapCallback* nodeCallback, const btVector3& raySource, const btVector3& rayTarget) const;
 void reportBoxCastOverlappingNodex(btNodeOverlapCallback* nodeCallback, const btVector3& raySource, const btVector3& rayTarget, const btVector3& aabbMin,const btVector3& aabbMax) const;

   void quantize(unsigned short* out, const btVector3& point,int isMax) const
 {

  btAssert(m_useQuantization);

  btAssert(point.getX() <= m_bvhAabbMax.getX());
  btAssert(point.getY() <= m_bvhAabbMax.getY());
  btAssert(point.getZ() <= m_bvhAabbMax.getZ());

  btAssert(point.getX() >= m_bvhAabbMin.getX());
  btAssert(point.getY() >= m_bvhAabbMin.getY());
  btAssert(point.getZ() >= m_bvhAabbMin.getZ());

  btVector3 v = (point - m_bvhAabbMin) * m_bvhQuantization;
  ///Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
  ///end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
  ///@todo: double-check this
  if (isMax)
  {
   out[0] = (unsigned short) (((unsigned short)(v.getX()+btScalar(1.)) | 1));
   out[1] = (unsigned short) (((unsigned short)(v.getY()+btScalar(1.)) | 1));
   out[2] = (unsigned short) (((unsigned short)(v.getZ()+btScalar(1.)) | 1));
  } else
  {
   out[0] = (unsigned short) (((unsigned short)(v.getX()) & 0xfffe));
   out[1] = (unsigned short) (((unsigned short)(v.getY()) & 0xfffe));
   out[2] = (unsigned short) (((unsigned short)(v.getZ()) & 0xfffe));
  }


#ifdef DEBUG_CHECK_DEQUANTIZATION
  btVector3 newPoint = unQuantize(out);
  if (isMax)
  {
   if (newPoint.getX() < point.getX())
   {
    printf("unconservative X, diffX = %f, oldX=%f,newX=%f\n",newPoint.getX()-point.getX(), newPoint.getX(),point.getX());
   }
   if (newPoint.getY() < point.getY())
   {
    printf("unconservative Y, diffY = %f, oldY=%f,newY=%f\n",newPoint.getY()-point.getY(), newPoint.getY(),point.getY());
   }
   if (newPoint.getZ() < point.getZ())
   {

    printf("unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n",newPoint.getZ()-point.getZ(), newPoint.getZ(),point.getZ());
   }
  } else
  {
   if (newPoint.getX() > point.getX())
   {
    printf("unconservative X, diffX = %f, oldX=%f,newX=%f\n",newPoint.getX()-point.getX(), newPoint.getX(),point.getX());
   }
   if (newPoint.getY() > point.getY())
   {
    printf("unconservative Y, diffY = %f, oldY=%f,newY=%f\n",newPoint.getY()-point.getY(), newPoint.getY(),point.getY());
   }
   if (newPoint.getZ() > point.getZ())
   {
    printf("unconservative Z, diffZ = %f, oldZ=%f,newZ=%f\n",newPoint.getZ()-point.getZ(), newPoint.getZ(),point.getZ());
   }
  }
#endif //DEBUG_CHECK_DEQUANTIZATION

 }


  void quantizeWithClamp(unsigned short* out, const btVector3& point2,int isMax) const
 {

  btAssert(m_useQuantization);

  btVector3 clampedPoint(point2);
  clampedPoint.setMax(m_bvhAabbMin);
  clampedPoint.setMin(m_bvhAabbMax);

  quantize(out,clampedPoint,isMax);

 }
 
  btVector3 unQuantize(const unsigned short* vecIn) const
 {
   btVector3 vecOut;
   vecOut.setValue(
   (btScalar)(vecIn[0]) / (m_bvhQuantization.getX()),
   (btScalar)(vecIn[1]) / (m_bvhQuantization.getY()),
   (btScalar)(vecIn[2]) / (m_bvhQuantization.getZ()));
   vecOut += m_bvhAabbMin;
   return vecOut;
 }

 ///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
 void setTraversalMode(btTraversalMode traversalMode)
 {
  m_traversalMode = traversalMode;
 }


  QuantizedNodeArray& getQuantizedNodeArray()
 { 
  return m_quantizedContiguousNodes;
 }


  BvhSubtreeInfoArray& getSubtreeInfoArray()
 {
  return m_SubtreeHeaders;
 }

////////////////////////////////////////////////////////////////////

 /////Calculate space needed to store BVH for serialization
 unsigned calculateSerializeBufferSize() const;

 /// Data buffer MUST be 16 byte aligned
 virtual bool serialize(void *o_alignedDataBuffer, unsigned i_dataBufferSize, bool i_swapEndian) const;

 ///deSerializeInPlace loads and initializes a BVH from a buffer in memory 'in place'
 static btQuantizedBvh *deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian);

 static unsigned int getAlignmentSerializationPadding();
//////////////////////////////////////////////////////////////////////

 
 virtual int calculateSerializeBufferSizeNew() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

 virtual void deSerializeFloat(struct btQuantizedBvhFloatData& quantizedBvhFloatData);

 virtual void deSerializeDouble(struct btQuantizedBvhDoubleData& quantizedBvhDoubleData);


////////////////////////////////////////////////////////////////////

  bool isQuantized()
 {
  return m_useQuantization;
 }

private:
 // Special "copy" constructor that allows for in-place deserialization
 // Prevents btVector3's default constructor from being called, but doesn't inialize much else
 // ownsMemory should most likely be false if deserializing, and if you are not, don't call this (it also changes the function signature, which we need)
 btQuantizedBvh(btQuantizedBvh &other, bool ownsMemory);

}
;


class btBvhSubtreeInfoData
{ public: 
 int   m_rootNodeIndex;
 int   m_subtreeSize;
 unsigned short m_quantizedAabbMin[3];
 unsigned short m_quantizedAabbMax[3];
};

class btOptimizedBvhNodeFloatData
{ public: 
 btVector3FloatData m_aabbMinOrg;
 btVector3FloatData m_aabbMaxOrg;
 int m_escapeIndex;
 int m_subPart;
 int m_triangleIndex;
 char m_pad[4];
};

class btOptimizedBvhNodeDoubleData
{ public: 
 btVector3DoubleData m_aabbMinOrg;
 btVector3DoubleData m_aabbMaxOrg;
 int m_escapeIndex;
 int m_subPart;
 int m_triangleIndex;
 char m_pad[4];
};


class btQuantizedBvhNodeData
{ public: 
 unsigned short m_quantizedAabbMin[3];
 unsigned short m_quantizedAabbMax[3];
 int m_escapeIndexOrTriangleIndex;
};

class btQuantizedBvhFloatData
{ public: 
 btVector3FloatData   m_bvhAabbMin;
 btVector3FloatData   m_bvhAabbMax;
 btVector3FloatData   m_bvhQuantization;
 int     m_curNodeIndex;
 int     m_useQuantization;
 int     m_numContiguousLeafNodes;
 int     m_numQuantizedContiguousNodes;
 btOptimizedBvhNodeFloatData *m_contiguousNodesPtr;
 btQuantizedBvhNodeData  *m_quantizedContiguousNodesPtr;
 btBvhSubtreeInfoData *m_subTreeInfoPtr;
 int     m_traversalMode;
 int     m_numSubtreeHeaders;
 
};

class btQuantizedBvhDoubleData
{ public: 
 btVector3DoubleData   m_bvhAabbMin;
 btVector3DoubleData   m_bvhAabbMax;
 btVector3DoubleData   m_bvhQuantization;
 int       m_curNodeIndex;
 int       m_useQuantization;
 int       m_numContiguousLeafNodes;
 int       m_numQuantizedContiguousNodes;
 btOptimizedBvhNodeDoubleData *m_contiguousNodesPtr;
 btQuantizedBvhNodeData   *m_quantizedContiguousNodesPtr;

 int       m_traversalMode;
 int       m_numSubtreeHeaders;
 btBvhSubtreeInfoData  *m_subTreeInfoPtr;
};


 int btQuantizedBvh::calculateSerializeBufferSizeNew() const
{
 return sizeof(btQuantizedBvhData);
}



#endif //BT_QUANTIZED_BVH_H
//// ../src/BulletCollision/CollisionShapes/btTriangleInfoMap.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _BT_TRIANGLE_INFO_MAP_H
#define _BT_TRIANGLE_INFO_MAP_H


#include "LinearMath/btHashMap.h"
#include "LinearMath/btSerializer.h"


///for btTriangleInfo m_flags
#define TRI_INFO_V0V1_CONVEX 1
#define TRI_INFO_V1V2_CONVEX 2
#define TRI_INFO_V2V0_CONVEX 4

#define TRI_INFO_V0V1_SWAP_NORMALB 8
#define TRI_INFO_V1V2_SWAP_NORMALB 16
#define TRI_INFO_V2V0_SWAP_NORMALB 32


///The btTriangleInfo structure stores information to adjust collision normals to avoid collisions against internal edges
///it can be generated using 
class btTriangleInfo
{ public: 
 btTriangleInfo()
 {
  m_edgeV0V1Angle = SIMD_2_PI;
  m_edgeV1V2Angle = SIMD_2_PI;
  m_edgeV2V0Angle = SIMD_2_PI;
  m_flags=0;
 }

 int   m_flags;

 btScalar m_edgeV0V1Angle;
 btScalar m_edgeV1V2Angle;
 btScalar m_edgeV2V0Angle;

};

typedef btHashMap<btHashInt,btTriangleInfo> btInternalTriangleInfoMap;


///The btTriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or using btGenerateInternalEdgeInfo.
class btTriangleInfoMap : public btInternalTriangleInfoMap
{ public: 
 btScalar m_convexEpsilon;///used to determine if an edge or contact normal is convex, using the dot product
 btScalar m_planarEpsilon; ///used to determine if a triangle edge is planar with zero angle
 btScalar m_equalVertexThreshold; ///used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold, they are considered to be 'shared'
 btScalar m_edgeDistanceThreshold; ///used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than this distance threshold it is considered to "hit the edge"
 btScalar m_maxEdgeAngleThreshold; //ignore edges that connect triangles at an angle larger than this m_maxEdgeAngleThreshold
 btScalar m_zeroAreaThreshold; ///used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold)
 
 
 btTriangleInfoMap()
 {
  m_convexEpsilon = 0.00f;
  m_planarEpsilon = 0.0001f;
  m_equalVertexThreshold = btScalar(0.0001)*btScalar(0.0001);
  m_edgeDistanceThreshold = btScalar(0.1);
  m_zeroAreaThreshold = btScalar(0.0001)*btScalar(0.0001);
  m_maxEdgeAngleThreshold = SIMD_2_PI;
 }
 virtual ~btTriangleInfoMap() {}

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

 void deSerialize(struct btTriangleInfoMapData& data);

};

class btTriangleInfoData
{ public: 
 int   m_flags;
 float m_edgeV0V1Angle;
 float m_edgeV1V2Angle;
 float m_edgeV2V0Angle;
};

class btTriangleInfoMapData
{ public: 
 int     *m_hashTablePtr;
 int     *m_nextPtr;
 btTriangleInfoData *m_valueArrayPtr;
 int     *m_keyArrayPtr;

 float m_convexEpsilon;
 float m_planarEpsilon;
 float m_equalVertexThreshold; 
 float m_edgeDistanceThreshold;
 float m_zeroAreaThreshold;

 int  m_nextSize;
 int  m_hashTableSize;
 int  m_numValues;
 int  m_numKeys;
 char m_padding[4];
};

 int btTriangleInfoMap::calculateSerializeBufferSize() const
{
 return sizeof(btTriangleInfoMapData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btTriangleInfoMap::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btTriangleInfoMapData* tmapData = (btTriangleInfoMapData*) dataBuffer;
 tmapData->m_convexEpsilon = m_convexEpsilon;
 tmapData->m_planarEpsilon = m_planarEpsilon;
 tmapData->m_equalVertexThreshold = m_equalVertexThreshold;
 tmapData->m_edgeDistanceThreshold = m_edgeDistanceThreshold;
 tmapData->m_zeroAreaThreshold = m_zeroAreaThreshold;
 
 tmapData->m_hashTableSize = m_hashTable.size();

 tmapData->m_hashTablePtr = tmapData->m_hashTableSize ? (int*)serializer->getUniquePointer((void*)&m_hashTable[0]) : 0;
 if (tmapData->m_hashTablePtr)
 { 
  //serialize an int buffer
  int sz = sizeof(int);
  int numElem = tmapData->m_hashTableSize;
  btChunk* chunk = serializer->allocate(sz,numElem);
  int* memPtr = (int*)chunk->m_oldPtr;
  for (int i=0;i<numElem;i++,memPtr++)
  {
   *memPtr = m_hashTable[i];
  }
  serializer->finalizeChunk(chunk,"int",BT_ARRAY_CODE,(void*)&m_hashTable[0]);

 }

 tmapData->m_nextSize = m_next.size();
 tmapData->m_nextPtr = tmapData->m_nextSize? (int*)serializer->getUniquePointer((void*)&m_next[0]): 0;
 if (tmapData->m_nextPtr)
 {
  int sz = sizeof(int);
  int numElem = tmapData->m_nextSize;
  btChunk* chunk = serializer->allocate(sz,numElem);
  int* memPtr = (int*)chunk->m_oldPtr;
  for (int i=0;i<numElem;i++,memPtr++)
  {
   *memPtr = m_next[i];
  }
  serializer->finalizeChunk(chunk,"int",BT_ARRAY_CODE,(void*)&m_next[0]);
 }
 
 tmapData->m_numValues = m_valueArray.size();
 tmapData->m_valueArrayPtr = tmapData->m_numValues ? (btTriangleInfoData*)serializer->getUniquePointer((void*)&m_valueArray[0]): 0;
 if (tmapData->m_valueArrayPtr)
 {
  int sz = sizeof(btTriangleInfoData);
  int numElem = tmapData->m_numValues;
  btChunk* chunk = serializer->allocate(sz,numElem);
  btTriangleInfoData* memPtr = (btTriangleInfoData*)chunk->m_oldPtr;
  for (int i=0;i<numElem;i++,memPtr++)
  {
   memPtr->m_edgeV0V1Angle = m_valueArray[i].m_edgeV0V1Angle;
   memPtr->m_edgeV1V2Angle = m_valueArray[i].m_edgeV1V2Angle;
   memPtr->m_edgeV2V0Angle = m_valueArray[i].m_edgeV2V0Angle;
   memPtr->m_flags = m_valueArray[i].m_flags;
  }
  serializer->finalizeChunk(chunk,"btTriangleInfoData",BT_ARRAY_CODE,(void*) &m_valueArray[0]);
 }
 
 tmapData->m_numKeys = m_keyArray.size();
 tmapData->m_keyArrayPtr = tmapData->m_numKeys ? (int*)serializer->getUniquePointer((void*)&m_keyArray[0]) : 0;
 if (tmapData->m_keyArrayPtr)
 {
  int sz = sizeof(int);
  int numElem = tmapData->m_numValues;
  btChunk* chunk = serializer->allocate(sz,numElem);
  int* memPtr = (int*)chunk->m_oldPtr;
  for (int i=0;i<numElem;i++,memPtr++)
  {
   *memPtr = m_keyArray[i].getUid1();
  }
  serializer->finalizeChunk(chunk,"int",BT_ARRAY_CODE,(void*) &m_keyArray[0]);

 }
 return "btTriangleInfoMapData";
}



///fills the dataBuffer and returns the struct name (and 0 on failure)
 void btTriangleInfoMap::deSerialize(btTriangleInfoMapData& tmapData )
{


 m_convexEpsilon = tmapData.m_convexEpsilon;
 m_planarEpsilon = tmapData.m_planarEpsilon;
 m_equalVertexThreshold = tmapData.m_equalVertexThreshold;
 m_edgeDistanceThreshold = tmapData.m_edgeDistanceThreshold;
 m_zeroAreaThreshold = tmapData.m_zeroAreaThreshold;
 m_hashTable.resize(tmapData.m_hashTableSize);
 int i =0;
 for (i=0;i<tmapData.m_hashTableSize;i++)
 {
  m_hashTable[i] = tmapData.m_hashTablePtr[i];
 }
 m_next.resize(tmapData.m_nextSize);
 for (i=0;i<tmapData.m_nextSize;i++)
 {
  m_next[i] = tmapData.m_nextPtr[i];
 }
 m_valueArray.resize(tmapData.m_numValues);
 for (i=0;i<tmapData.m_numValues;i++)
 {
  m_valueArray[i].m_edgeV0V1Angle = tmapData.m_valueArrayPtr[i].m_edgeV0V1Angle;
  m_valueArray[i].m_edgeV1V2Angle = tmapData.m_valueArrayPtr[i].m_edgeV1V2Angle;
  m_valueArray[i].m_edgeV2V0Angle = tmapData.m_valueArrayPtr[i].m_edgeV2V0Angle;
  m_valueArray[i].m_flags = tmapData.m_valueArrayPtr[i].m_flags;
 }
 
 m_keyArray.resize(tmapData.m_numKeys,btHashInt(0));
 for (i=0;i<tmapData.m_numKeys;i++)
 {
  m_keyArray[i].setUid1(tmapData.m_keyArrayPtr[i]);
 }
}


#endif //_BT_TRIANGLE_INFO_MAP_H
//// ../src/LinearMath/btHashMap.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_HASH_MAP_H
#define BT_HASH_MAP_H

#include "btAlignedObjectArray.h"

///very basic hashable string implementation, compatible with btHashMap
class btHashString
{ public: 
 const char* m_string;
 unsigned int m_hash;

  unsigned int getHash()const
 {
  return m_hash;
 }

 btHashString(const char* name)
  :m_string(name)
 {
  /* magic numbers from http://www.isthe.com/chongo/tech/comp/fnv/ */
  static const unsigned int  InitialFNV = 2166136261u;
  static const unsigned int FNVMultiple = 16777619u;

  /* Fowler / Noll / Vo (FNV) Hash */
  unsigned int hash = InitialFNV;
  
  for(int i = 0; m_string[i]; i++)
  {
   hash = hash ^ (m_string[i]);       /* xor  the low 8 bits */
   hash = hash * FNVMultiple;  /* multiply by the magic number */
  }
  m_hash = hash;
 }

 int portableStringCompare(const char* src, const char* dst) const
 {
   int ret = 0 ;

   while( ! (ret = *(unsigned char *)src - *(unsigned char *)dst) && *dst)
     ++src, ++dst;

   if ( ret < 0 )
     ret = -1 ;
   else if ( ret > 0 )
     ret = 1 ;

   return( ret );
 }

 bool equals(const btHashString& other) const
 {
  return (m_string == other.m_string) ||
   (0==portableStringCompare(m_string,other.m_string));

 }

};

const int BT_HASH_NULL=0xffffffff;


class btHashInt
{
 int m_uid;
public:
 btHashInt(int uid) :m_uid(uid)
 {
 }

 int getUid1() const
 {
  return m_uid;
 }

 void setUid1(int uid)
 {
  m_uid = uid;
 }

 bool equals(const btHashInt& other) const
 {
  return getUid1() == other.getUid1();
 }
 //to our success
  unsigned int getHash()const
 {
  int key = m_uid;
  // Thomas Wang's hash
  key += ~(key << 15); key ^=  (key >> 10); key +=  (key << 3); key ^=  (key >> 6); key += ~(key << 11); key ^=  (key >> 16);
  return key;
 }
};



class btHashPtr
{

 union
 {
  const void* m_pointer;
  int m_hashValues[2];
 };

public:

 btHashPtr(const void* ptr)
  :m_pointer(ptr)
 {
 }

 const void* getPointer() const
 {
  return m_pointer;
 }

 bool equals(const btHashPtr& other) const
 {
  return getPointer() == other.getPointer();
 }

 //to our success
  unsigned int getHash()const
 {
  const bool VOID_IS_8 = ((sizeof(void*)==8));
  
  int key = VOID_IS_8? m_hashValues[0]+m_hashValues[1] : m_hashValues[0];
 
  // Thomas Wang's hash
  key += ~(key << 15); key ^=  (key >> 10); key +=  (key << 3); key ^=  (key >> 6); key += ~(key << 11); key ^=  (key >> 16);
  return key;
 }

 
};


template <class Value>
class btHashKeyPtr
{
        int     m_uid;
public:

        btHashKeyPtr(int uid)    :m_uid(uid)
        {
        }

        int     getUid1() const
        {
                return m_uid;
        }

        bool equals(const btHashKeyPtr<Value>& other) const
        {
                return getUid1() == other.getUid1();
        }

        //to our success
               unsigned int getHash()const
        {
                int key = m_uid;
                // Thomas Wang's hash
                key += ~(key << 15); key ^=  (key >> 10); key +=  (key << 3); key ^=  (key >> 6); key += ~(key << 11); key ^=  (key >> 16);
                return key;
        }

        
};


template <class Value>
class btHashKey
{
 int m_uid;
public:

 btHashKey(int uid) :m_uid(uid)
 {
 }

 int getUid1() const
 {
  return m_uid;
 }

 bool equals(const btHashKey<Value>& other) const
 {
  return getUid1() == other.getUid1();
 }
 //to our success
  unsigned int getHash()const
 {
  int key = m_uid;
  // Thomas Wang's hash
  key += ~(key << 15); key ^=  (key >> 10); key +=  (key << 3); key ^=  (key >> 6); key += ~(key << 11); key ^=  (key >> 16);
  return key;
 }
};


///The btHashMap template class implements a generic and lightweight hashmap.
///A basic sample of how to use btHashMap is located in Demos\BasicDemo\main.cpp
template <class Key, class Value>
class btHashMap
{

protected:
 btAlignedObjectArray<int>  m_hashTable;
 btAlignedObjectArray<int>  m_next;
 
 btAlignedObjectArray<Value>  m_valueArray;
 btAlignedObjectArray<Key>  m_keyArray;

 void growTables(const Key& /*key*/)
 {
  int newCapacity = m_valueArray.capacity();

  if (m_hashTable.size() < newCapacity)
  {
   //grow hashtable and next table
   int curHashtableSize = m_hashTable.size();

   m_hashTable.resize(newCapacity);
   m_next.resize(newCapacity);

   int i;

   for (i= 0; i < newCapacity; ++i)
   {
    m_hashTable[i] = BT_HASH_NULL;
   }
   for (i = 0; i < newCapacity; ++i)
   {
    m_next[i] = BT_HASH_NULL;
   }

   for(i=0;i<curHashtableSize;i++)
   {
    //const Value& value = m_valueArray[i];
    //const Key& key = m_keyArray[i];

    int hashValue = m_keyArray[i].getHash() & (m_valueArray.capacity()-1); // New hash value with new mask
    m_next[i] = m_hashTable[hashValue];
    m_hashTable[hashValue] = i;
   }


  }
 }

 public:

 void insert(const Key& key, const Value& value) {
  int hash = key.getHash() & (m_valueArray.capacity()-1);

  //replace value if the key is already there
  int index = findIndex(key);
  if (index != BT_HASH_NULL)
  {
   m_valueArray[index]=value;
   return;
  }

  int count = m_valueArray.size();
  int oldCapacity = m_valueArray.capacity();
  m_valueArray.push_back(value);
  m_keyArray.push_back(key);

  int newCapacity = m_valueArray.capacity();
  if (oldCapacity < newCapacity)
  {
   growTables(key);
   //hash with new capacity
   hash = key.getHash() & (m_valueArray.capacity()-1);
  }
  m_next[count] = m_hashTable[hash];
  m_hashTable[hash] = count;
 }

 void remove(const Key& key) {

  int hash = key.getHash() & (m_valueArray.capacity()-1);

  int pairIndex = findIndex(key);
  
  if (pairIndex ==BT_HASH_NULL)
  {
   return;
  }

  // Remove the pair from the hash table.
  int index = m_hashTable[hash];
  btAssert(index != BT_HASH_NULL);

  int previous = BT_HASH_NULL;
  while (index != pairIndex)
  {
   previous = index;
   index = m_next[index];
  }

  if (previous != BT_HASH_NULL)
  {
   btAssert(m_next[previous] == pairIndex);
   m_next[previous] = m_next[pairIndex];
  }
  else
  {
   m_hashTable[hash] = m_next[pairIndex];
  }

  // We now move the last pair into spot of the
  // pair being removed. We need to fix the hash
  // table indices to support the move.

  int lastPairIndex = m_valueArray.size() - 1;

  // If the removed pair is the last pair, we are done.
  if (lastPairIndex == pairIndex)
  {
   m_valueArray.pop_back();
   m_keyArray.pop_back();
   return;
  }

  // Remove the last pair from the hash table.
  int lastHash = m_keyArray[lastPairIndex].getHash() & (m_valueArray.capacity()-1);

  index = m_hashTable[lastHash];
  btAssert(index != BT_HASH_NULL);

  previous = BT_HASH_NULL;
  while (index != lastPairIndex)
  {
   previous = index;
   index = m_next[index];
  }

  if (previous != BT_HASH_NULL)
  {
   btAssert(m_next[previous] == lastPairIndex);
   m_next[previous] = m_next[lastPairIndex];
  }
  else
  {
   m_hashTable[lastHash] = m_next[lastPairIndex];
  }

  // Copy the last pair into the remove pair's spot.
  m_valueArray[pairIndex] = m_valueArray[lastPairIndex];
  m_keyArray[pairIndex] = m_keyArray[lastPairIndex];

  // Insert the last pair into the hash table
  m_next[pairIndex] = m_hashTable[lastHash];
  m_hashTable[lastHash] = pairIndex;

  m_valueArray.pop_back();
  m_keyArray.pop_back();

 }


 int size() const
 {
  return m_valueArray.size();
 }

 const Value* getAtIndex(int index) const
 {
  btAssert(index < m_valueArray.size());

  return &m_valueArray[index];
 }

 Value* getAtIndex(int index)
 {
  btAssert(index < m_valueArray.size());

  return &m_valueArray[index];
 }

 Value* operator[](const Key& key) {
  return find(key);
 }

 const Value* find(const Key& key) const
 {
  int index = findIndex(key);
  if (index == BT_HASH_NULL)
  {
   return NULL;
  }
  return &m_valueArray[index];
 }

 Value* find(const Key& key)
 {
  int index = findIndex(key);
  if (index == BT_HASH_NULL)
  {
   return NULL;
  }
  return &m_valueArray[index];
 }


 int findIndex(const Key& key) const
 {
  unsigned int hash = key.getHash() & (m_valueArray.capacity()-1);

  if (hash >= (unsigned int)m_hashTable.size())
  {
   return BT_HASH_NULL;
  }

  int index = m_hashTable[hash];
  while ((index != BT_HASH_NULL) && key.equals(m_keyArray[index]) == false)
  {
   index = m_next[index];
  }
  return index;
 }

 void clear()
 {
  m_hashTable.clear();
  m_next.clear();
  m_valueArray.clear();
  m_keyArray.clear();
 }

};

#endif //BT_HASH_MAP_H
//// ../src/LinearMath/btSerializer.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SERIALIZER_H
#define BT_SERIALIZER_H

#include "btScalar.h" // has definitions like 
#include "btStackAlloc.h"
#include "btHashMap.h"

#if !defined( __CELLOS_LV2__) && !defined(__MWERKS__)
#include <memory.h>
#endif
#include <string.h>



///only the 32bit versions for now
extern unsigned char sBulletDNAstr[];
extern int sBulletDNAlen;
extern unsigned char sBulletDNAstr64[];
extern int sBulletDNAlen64;

 int btStrLen(const char* str) 
{
    if (!str) 
  return(0);
 int len = 0;
    
 while (*str != 0)
 {
        str++;
        len++;
    }

    return len;
}


class btChunk
{
public:
 int  m_chunkCode;
 int  m_length;
 void *m_oldPtr;
 int  m_dna_nr;
 int  m_number;
};

enum btSerializationFlags
{
 BT_SERIALIZE_NO_BVH = 1,
 BT_SERIALIZE_NO_TRIANGLEINFOMAP = 2,
 BT_SERIALIZE_NO_DUPLICATE_ASSERT = 4
};

class btSerializer
{

public:

 virtual ~btSerializer() {}

 virtual const unsigned char*  getBufferPointer() const = 0;

 virtual int  getCurrentBufferSize() const = 0;

 virtual btChunk* allocate(size_t size, int numElements) = 0;

 virtual void finalizeChunk(btChunk* chunk, const char* structType, int chunkCode,void* oldPtr)= 0;

 virtual  void* findPointer(void* oldPtr)  = 0;

 virtual void* getUniquePointer(void*oldPtr) = 0;

 virtual void startSerialization() = 0;
 
 virtual void finishSerialization() = 0;

 virtual const char* findNameForPointer(const void* ptr) const = 0;

 virtual void registerNameForPointer(const void* ptr, const char* name) = 0;

 virtual void serializeName(const char* ptr) = 0;

 virtual int  getSerializationFlags() const = 0;

 virtual void setSerializationFlags(int flags) = 0;


};



#define BT_HEADER_LENGTH 12
#if defined(__sgi) || defined (__sparc) || defined (__sparc__) || defined (__PPC__) || defined (__ppc__) || defined (__BIG_ENDIAN__)
# define MAKE_ID(a,b,c,d) ( (int)(a)<<24 | (int)(b)<<16 | (c)<<8 | (d) )
#else
# define MAKE_ID(a,b,c,d) ( (int)(d)<<24 | (int)(c)<<16 | (b)<<8 | (a) )
#endif

#define BT_SOFTBODY_CODE  MAKE_ID('S','B','D','Y')
#define BT_COLLISIONOBJECT_CODE MAKE_ID('C','O','B','J')
#define BT_RIGIDBODY_CODE  MAKE_ID('R','B','D','Y')
#define BT_CONSTRAINT_CODE  MAKE_ID('C','O','N','S')
#define BT_BOXSHAPE_CODE  MAKE_ID('B','O','X','S')
#define BT_QUANTIZED_BVH_CODE MAKE_ID('Q','B','V','H')
#define BT_TRIANLGE_INFO_MAP MAKE_ID('T','M','A','P')
#define BT_SHAPE_CODE   MAKE_ID('S','H','A','P')
#define BT_ARRAY_CODE   MAKE_ID('A','R','A','Y')
#define BT_SBMATERIAL_CODE  MAKE_ID('S','B','M','T')
#define BT_SBNODE_CODE   MAKE_ID('S','B','N','D')
#define BT_DNA_CODE    MAKE_ID('D','N','A','1')


class btPointerUid
{ public: 
 union
 {
  void* m_ptr;
  int  m_uniqueIds[2];
 };
};

///The btDefaultSerializer is the main Bullet serialization class.
///The constructor takes an optional argument for backwards compatibility, it is recommended to leave this empty/zero.
class btDefaultSerializer : public btSerializer
{


 btAlignedObjectArray<char*>   mTypes;
 btAlignedObjectArray<short*>   mStructs;
 btAlignedObjectArray<short>   mTlens;
 btHashMap<btHashInt, int>   mStructReverse;
 btHashMap<btHashString,int> mTypeLookup;

 
 btHashMap<btHashPtr,void*> m_chunkP;
 
 btHashMap<btHashPtr,const char*> m_nameMap;

 btHashMap<btHashPtr,btPointerUid> m_uniquePointers;
 int m_uniqueIdGenerator;

 int     m_totalSize;
 unsigned char*  m_buffer;
 int     m_currentSize;
 void*    m_dna;
 int     m_dnaLength;

 int     m_serializationFlags;


 btAlignedObjectArray<btChunk*> m_chunkPtrs;
 
protected:

 virtual void* findPointer(void* oldPtr) 
 {
  void** ptr = m_chunkP.find(oldPtr);
  if (ptr && *ptr)
   return *ptr;
  return 0;
 }

 



  void writeDNA()
  {
   btChunk* dnaChunk = allocate(m_dnaLength,1);
   memcpy(dnaChunk->m_oldPtr,m_dna,m_dnaLength);
   finalizeChunk(dnaChunk,"DNA1",BT_DNA_CODE, m_dna);
  }

  int getReverseType(const char *type) const
  {

   btHashString key(type);
   const int* valuePtr = mTypeLookup.find(key);
   if (valuePtr)
    return *valuePtr;
   
   return -1;
  }

  void initDNA(const char* bdnaOrg,int dnalen)
  {
   ///was already initialized
   if (m_dna)
    return;

   int littleEndian= 1;
   littleEndian= ((char*)&littleEndian)[0];
   

   m_dna = btAlignedAlloc(dnalen,16);
   memcpy(m_dna,bdnaOrg,dnalen);
   m_dnaLength = dnalen;

   int *intPtr=0;
   short *shtPtr=0;
   char *cp = 0;int dataLen =0;long nr=0;
   intPtr = (int*)m_dna;

   /*
    SDNA (4 bytes) (magic number)
    NAME (4 bytes)
    <nr> (4 bytes) amount of names (int)
    <string>
    <string>
   */

   if (strncmp((const char*)m_dna, "SDNA", 4)==0)
   {
    // skip ++ NAME
    intPtr++; intPtr++;
   }

   // Parse names
   if (!littleEndian)
    *intPtr = btSwapEndian(*intPtr);
    
   dataLen = *intPtr;
   
   intPtr++;

   cp = (char*)intPtr;
   int i;
   for ( i=0; i<dataLen; i++)
   {
    
    while (*cp)cp++;
    cp++;
   }
   {
    nr= (long)cp;
   // long mask=3;
    nr= ((nr+3)&~3)-nr;
    while (nr--)
    {
     cp++;
    }
   }

   /*
    TYPE (4 bytes)
    <nr> amount of types (int)
    <string>
    <string>
   */

   intPtr = (int*)cp;
   assert(strncmp(cp, "TYPE", 4)==0); intPtr++;

   if (!littleEndian)
    *intPtr =  btSwapEndian(*intPtr);
   
   dataLen = *intPtr;
   intPtr++;

   
   cp = (char*)intPtr;
   for (i=0; i<dataLen; i++)
   {
    mTypes.push_back(cp);
    while (*cp)cp++;
    cp++;
   }

  {
    nr= (long)cp;
   // long mask=3;
    nr= ((nr+3)&~3)-nr;
    while (nr--)
    {
     cp++;
    }
   }


   /*
    TLEN (4 bytes)
    <len> (short) the lengths of types
    <len>
   */

   // Parse type lens
   intPtr = (int*)cp;
   assert(strncmp(cp, "TLEN", 4)==0); intPtr++;

   dataLen = (int)mTypes.size();

   shtPtr = (short*)intPtr;
   for (i=0; i<dataLen; i++, shtPtr++)
   {
    if (!littleEndian)
     shtPtr[0] = btSwapEndian(shtPtr[0]);
    mTlens.push_back(shtPtr[0]);
   }

   if (dataLen & 1) shtPtr++;

   /*
    STRC (4 bytes)
    <nr> amount of structs (int)
    <typenr>
    <nr_of_elems>
    <typenr>
    <namenr>
    <typenr>
    <namenr>
   */

   intPtr = (int*)shtPtr;
   cp = (char*)intPtr;
   assert(strncmp(cp, "STRC", 4)==0); intPtr++;

   if (!littleEndian)
    *intPtr = btSwapEndian(*intPtr);
   dataLen = *intPtr ; 
   intPtr++;


   shtPtr = (short*)intPtr;
   for (i=0; i<dataLen; i++)
   {
    mStructs.push_back (shtPtr);
    
    if (!littleEndian)
    {
     shtPtr[0]= btSwapEndian(shtPtr[0]);
     shtPtr[1]= btSwapEndian(shtPtr[1]);

     int len = shtPtr[1];
     shtPtr+= 2;

     for (int a=0; a<len; a++, shtPtr+=2)
     {
       shtPtr[0]= btSwapEndian(shtPtr[0]);
       shtPtr[1]= btSwapEndian(shtPtr[1]);
     }

    } else
    {
     shtPtr+= (2*shtPtr[1])+2;
    }
   }

   // build reverse lookups
   for (i=0; i<(int)mStructs.size(); i++)
   {
    short *strc = mStructs.at(i);
    mStructReverse.insert(strc[0], i);
    mTypeLookup.insert(btHashString(mTypes[strc[0]]),i);
   }
  }

public: 
 

 

  btDefaultSerializer(int totalSize=0)
   :m_totalSize(totalSize),
   m_currentSize(0),
   m_dna(0),
   m_dnaLength(0),
   m_serializationFlags(0)
  {
   m_buffer = m_totalSize?(unsigned char*)btAlignedAlloc(totalSize,16):0;
   
   const bool VOID_IS_8 = ((sizeof(void*)==8));

#ifdef BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
   if (VOID_IS_8)
   {
#if _WIN64
    initDNA((const char*)sBulletDNAstr64,sBulletDNAlen64);
#else
    btAssert(0);
#endif
   } else
   {
#ifndef _WIN64
    initDNA((const char*)sBulletDNAstr,sBulletDNAlen);
#else
    btAssert(0);
#endif
   }
 
#else //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
   if (VOID_IS_8)
   {
    initDNA((const char*)sBulletDNAstr64,sBulletDNAlen64);
   } else
   {
    initDNA((const char*)sBulletDNAstr,sBulletDNAlen);
   }
#endif //BT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES
 
  }

  virtual ~btDefaultSerializer() 
  {
   if (m_buffer)
    btAlignedFree(m_buffer);
   if (m_dna)
    btAlignedFree(m_dna);
  }

  void writeHeader(unsigned char* buffer) const
  {
   

#ifdef  BT_USE_DOUBLE_PRECISION
   memcpy(buffer, "BULLETd", 7);
#else
   memcpy(buffer, "BULLETf", 7);
#endif //BT_USE_DOUBLE_PRECISION
 
   int littleEndian= 1;
   littleEndian= ((char*)&littleEndian)[0];

   if (sizeof(void*)==8)
   {
    buffer[7] = '-';
   } else
   {
    buffer[7] = '_';
   }

   if (littleEndian)
   {
    buffer[8]='v';    
   } else
   {
    buffer[8]='V';
   }


   buffer[9] = '2';
   buffer[10] = '7';
   buffer[11] = '8';

  }

  virtual void startSerialization()
  {
   m_uniqueIdGenerator= 1;
   if (m_totalSize)
   {
    unsigned char* buffer = internalAlloc(BT_HEADER_LENGTH);
    writeHeader(buffer);
   }
   
  }

  virtual void finishSerialization()
  {
   writeDNA();

   //if we didn't pre-allocate a buffer, we need to create a contiguous buffer now
   int mysize = 0;
   if (!m_totalSize)
   {
    if (m_buffer)
     btAlignedFree(m_buffer);

    m_currentSize += BT_HEADER_LENGTH;
    m_buffer = (unsigned char*)btAlignedAlloc(m_currentSize,16);

    unsigned char* currentPtr = m_buffer;
    writeHeader(m_buffer);
    currentPtr += BT_HEADER_LENGTH;
    mysize+=BT_HEADER_LENGTH;
    for (int i=0;i< m_chunkPtrs.size();i++)
    {
     int curLength = sizeof(btChunk)+m_chunkPtrs[i]->m_length;
     memcpy(currentPtr,m_chunkPtrs[i], curLength);
     btAlignedFree(m_chunkPtrs[i]);
     currentPtr+=curLength;
     mysize+=curLength;
    }
   }

   mTypes.clear();
   mStructs.clear();
   mTlens.clear();
   mStructReverse.clear();
   mTypeLookup.clear();
   m_chunkP.clear();
   m_nameMap.clear();
   m_uniquePointers.clear();
   m_chunkPtrs.clear();
  }

  virtual void* getUniquePointer(void*oldPtr)
  {
   if (!oldPtr)
    return 0;

   btPointerUid* uptr = (btPointerUid*)m_uniquePointers.find(oldPtr);
   if (uptr)
   {
    return uptr->m_ptr;
   }
   m_uniqueIdGenerator++;
   
   btPointerUid uid;
   uid.m_uniqueIds[0] = m_uniqueIdGenerator;
   uid.m_uniqueIds[1] = m_uniqueIdGenerator;
   m_uniquePointers.insert(oldPtr,uid);
   return uid.m_ptr;

  }

  virtual const unsigned char*  getBufferPointer() const
  {
   return m_buffer;
  }

  virtual int     getCurrentBufferSize() const
  {
   return m_currentSize;
  }

  virtual void finalizeChunk(btChunk* chunk, const char* structType, int chunkCode,void* oldPtr)
  {
   if (!(m_serializationFlags&BT_SERIALIZE_NO_DUPLICATE_ASSERT))
   {
    btAssert(!findPointer(oldPtr));
   }

   chunk->m_dna_nr = getReverseType(structType);
   
   chunk->m_chunkCode = chunkCode;
   
   void* uniquePtr = getUniquePointer(oldPtr);
   
   m_chunkP.insert(oldPtr,uniquePtr);//chunk->m_oldPtr);
   chunk->m_oldPtr = uniquePtr;//oldPtr;
   
  }

  
  virtual unsigned char* internalAlloc(size_t size)
  {
   unsigned char* ptr = 0;

   if (m_totalSize)
   {
    ptr = m_buffer+m_currentSize;
    m_currentSize += int(size);
    btAssert(m_currentSize<m_totalSize);
   } else
   {
    ptr = (unsigned char*)btAlignedAlloc(size,16);
    m_currentSize += int(size);
   }
   return ptr;
  }

  

  virtual btChunk* allocate(size_t size, int numElements)
  {

   unsigned char* ptr = internalAlloc(int(size)*numElements+sizeof(btChunk));

   unsigned char* data = ptr + sizeof(btChunk);
   
   btChunk* chunk = (btChunk*)ptr;
   chunk->m_chunkCode = 0;
   chunk->m_oldPtr = data;
   chunk->m_length = int(size)*numElements;
   chunk->m_number = numElements;
   
   m_chunkPtrs.push_back(chunk);
   

   return chunk;
  }

  virtual const char* findNameForPointer(const void* ptr) const
  {
   const char*const * namePtr = m_nameMap.find(ptr);
   if (namePtr && *namePtr)
    return *namePtr;
   return 0;

  }

  virtual void registerNameForPointer(const void* ptr, const char* name)
  {
   m_nameMap.insert(ptr,name);
  }

  virtual void serializeName(const char* name)
  {
   if (name)
   {
    //don't serialize name twice
    if (findPointer((void*)name))
     return;

    int len = btStrLen(name);
    if (len)
    {

     int newLen = len+1;
     int padding = ((newLen+3)&~3)-newLen;
     newLen += padding;

     //serialize name string now
     btChunk* chunk = allocate(sizeof(char),newLen);
     char* destinationName = (char*)chunk->m_oldPtr;
     for (int i=0;i<len;i++)
     {
      destinationName[i] = name[i];
     }
     destinationName[len] = 0;
     finalizeChunk(chunk,"char",BT_ARRAY_CODE,(void*)name);
    }
   }
  }

  virtual int  getSerializationFlags() const
  {
   return m_serializationFlags;
  }

  virtual void setSerializationFlags(int flags)
  {
   m_serializationFlags = flags;
  }

};


#endif //BT_SERIALIZER_H

//// ../src/LinearMath/btStackAlloc.h
/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
StackAlloc extracted from GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

#ifndef BT_STACK_ALLOC
#define BT_STACK_ALLOC

#include "btScalar.h" //for btAssert
#include "btAlignedAllocator.h"

///The btBlock class is an internal structure for the btStackAlloc memory allocator.
class btBlock
{ public: 
 btBlock*   previous;
 unsigned char*  address;
};

///The StackAlloc class provides some fast stack-based memory allocator (LIFO last-in first-out)
class btStackAlloc
{
public:

 btStackAlloc(unsigned int size) { ctor();create(size); }
 ~btStackAlloc()  { destroy(); }
 
 inline void  create(unsigned int size)
 {
  destroy();
  data  =  (unsigned char*) btAlignedAlloc(size,16);
  totalsize = size;
 }
 inline void  destroy()
 {
  btAssert(usedsize==0);
  //Raise(L"StackAlloc is still in use");

  if(usedsize==0)
  {
   if(!ischild && data)  
    btAlignedFree(data);

   data    = 0;
   usedsize   = 0;
  }
  
 }

 int getAvailableMemory() const
 {
  return static_cast<int>(totalsize - usedsize);
 }

 unsigned char*   allocate(unsigned int size)
 {
  const unsigned int nus(usedsize+size);
  if(nus<totalsize)
  {
   usedsize=nus;
   return(data+(usedsize-size));
  }
  btAssert(0);
  //&& (L"Not enough memory"));
  
  return(0);
 }
  btBlock*  beginBlock()
 {
  btBlock* pb = (btBlock*)allocate(sizeof(btBlock));
  pb->previous = current;
  pb->address  = data+usedsize;
  current   = pb;
  return(pb);
 }
  void  endBlock(btBlock* block)
 {
  btAssert(block==current);
  //Raise(L"Unmatched blocks");
  if(block==current)
  {
   current  = block->previous;
   usedsize = (unsigned int)((block->address-data)-sizeof(btBlock));
  }
 }

private:
 void  ctor()
 {
  data  = 0;
  totalsize = 0;
  usedsize = 0;
  current  = 0;
  ischild  = false;
 }
 unsigned char*  data;
 unsigned int  totalsize;
 unsigned int  usedsize;
 btBlock* current;
 bool  ischild;
};

#endif //BT_STACK_ALLOC
//// ../src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"


///The btScaledBvhTriangleMeshShape allows to instance a scaled version of an existing btBvhTriangleMeshShape.
///Note that each btBvhTriangleMeshShape still can have its own local scaling, independent from this btScaledBvhTriangleMeshShape 'localScaling'
class btScaledBvhTriangleMeshShape : public btConcaveShape
{
 
 
 btVector3 m_localScaling;

 btBvhTriangleMeshShape* m_bvhTriMeshShape;

public:


 btScaledBvhTriangleMeshShape(btBvhTriangleMeshShape* childShape,const btVector3& localScaling);

 virtual ~btScaledBvhTriangleMeshShape();


 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;
 virtual void setLocalScaling(const btVector3& scaling);
 virtual const btVector3& getLocalScaling() const;
 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual void processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

 btBvhTriangleMeshShape* getChildShape()
 {
  return m_bvhTriMeshShape;
 }

 const btBvhTriangleMeshShape* getChildShape() const
 {
  return m_bvhTriMeshShape;
 }

 //debugging
 virtual const char* getName()const {return "SCALEDBVHTRIANGLEMESH";}

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btScaledTriangleMeshShapeData
{ public: 
 btTriangleMeshShapeData m_trimeshShapeData;

 btVector3FloatData m_localScaling;
};


 int btScaledBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
 return sizeof(btScaledTriangleMeshShapeData);
}


///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btScaledBvhTriangleMeshShape::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btScaledTriangleMeshShapeData* scaledMeshData = (btScaledTriangleMeshShapeData*) dataBuffer;
 m_bvhTriMeshShape->serialize(&scaledMeshData->m_trimeshShapeData,serializer);
 scaledMeshData->m_trimeshShapeData.m_collisionShapeData.m_shapeType = SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE;
 m_localScaling.serializeFloat(scaledMeshData->m_localScaling);
 return "btScaledTriangleMeshShapeData";
}


#endif //BT_SCALED_BVH_TRIANGLE_MESH_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btCompoundShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COMPOUND_SHAPE_H
#define BT_COMPOUND_SHAPE_H

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"
#include "LinearMath/btAlignedObjectArray.h"

//class btOptimizedBvh;
struct btDbvt;

ATTRIBUTE_ALIGNED16(struct) btCompoundShapeChild
{
 

 btTransform   m_transform;
 btCollisionShape* m_childShape;
 int     m_childShapeType;
 btScalar   m_childMargin;
 struct btDbvtNode* m_node;
};

 bool operator==(const btCompoundShapeChild& c1, const btCompoundShapeChild& c2)
{
 return  ( c1.m_transform      == c2.m_transform &&
  c1.m_childShape     == c2.m_childShape &&
  c1.m_childShapeType == c2.m_childShapeType &&
  c1.m_childMargin    == c2.m_childMargin );
}

/// The btCompoundShape allows to store multiple other btCollisionShapes
/// This allows for moving concave collision objects. This is more general then the static concave btBvhTriangleMeshShape.
/// It has an (optional) dynamic aabb tree to accelerate early rejection tests. 
/// @todo: This aabb tree can also be use to speed up ray tests on btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
/// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of btCompoundShape)
class btCompoundShape : public btCollisionShape
{
 btAlignedObjectArray<btCompoundShapeChild> m_children;
 btVector3      m_localAabbMin;
 btVector3      m_localAabbMax;

 btDbvt*       m_dynamicAabbTree;

 ///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
 int        m_updateRevision;

 btScalar m_collisionMargin;

protected:
 btVector3 m_localScaling;

public:
 

 btCompoundShape(bool enableDynamicAabbTree = true);

 virtual ~btCompoundShape();

 void addChildShape(const btTransform& localTransform,btCollisionShape* shape);

 /// Remove all children shapes that contain the specified shape
 virtual void removeChildShape(btCollisionShape* shape);

 void removeChildShapeByIndex(int childShapeindex);


 int  getNumChildShapes() const
 {
  return int (m_children.size());
 }

 btCollisionShape* getChildShape(int index)
 {
  return m_children[index].m_childShape;
 }
 const btCollisionShape* getChildShape(int index) const
 {
  return m_children[index].m_childShape;
 }

 btTransform& getChildTransform(int index)
 {
  return m_children[index].m_transform;
 }
 const btTransform& getChildTransform(int index) const
 {
  return m_children[index].m_transform;
 }

 ///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
 void updateChildTransform(int childIndex, const btTransform& newChildTransform, bool shouldRecalculateLocalAabb = true);


 btCompoundShapeChild* getChildList()
 {
  return &m_children[0];
 }

 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 /** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
 Use this yourself if you modify the children or their transforms. */
 virtual void recalculateLocalAabb(); 

 virtual void setLocalScaling(const btVector3& scaling);

 virtual const btVector3& getLocalScaling() const 
 {
  return m_localScaling;
 }

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 virtual void setMargin(btScalar margin)
 {
  m_collisionMargin = margin;
 }
 virtual btScalar getMargin() const
 {
  return m_collisionMargin;
 }
 virtual const char* getName()const
 {
  return "Compound";
 }

 const btDbvt* getDynamicAabbTree() const
 {
  return m_dynamicAabbTree;
 }
 
 btDbvt* getDynamicAabbTree()
 {
  return m_dynamicAabbTree;
 }

 void createAabbTreeFromChildren();

 ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
 ///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
 ///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
 ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
 ///of the collision object by the principal transform.
 void calculatePrincipalAxisTransform(btScalar* masses, btTransform& principal, btVector3& inertia) const;

 int getUpdateRevision() const
 {
  return m_updateRevision;
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCompoundShapeChildData
{ public: 
 btTransformFloatData m_transform;
 btCollisionShapeData *m_childShape;
 int      m_childShapeType;
 float     m_childMargin;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btCompoundShapeData
{ public: 
 btCollisionShapeData  m_collisionShapeData;

 btCompoundShapeChildData *m_childShapePtr;

 int       m_numChildShapes;

 float m_collisionMargin;

};


 int btCompoundShape::calculateSerializeBufferSize() const
{
 return sizeof(btCompoundShapeData);
}







#endif //BT_COMPOUND_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btTetrahedronShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SIMPLEX_1TO4_SHAPE
#define BT_SIMPLEX_1TO4_SHAPE


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"


///The btBU_Simplex1to4 implements tetrahedron, triangle, line, vertex collision shapes. In most cases it is better to use btConvexHullShape instead.
class btBU_Simplex1to4 : public btPolyhedralConvexAabbCachingShape
{
protected:

 int m_numVertices;
 btVector3 m_vertices[4];

public:
 btBU_Simplex1to4();

 btBU_Simplex1to4(const btVector3& pt0);
 btBU_Simplex1to4(const btVector3& pt0,const btVector3& pt1);
 btBU_Simplex1to4(const btVector3& pt0,const btVector3& pt1,const btVector3& pt2);
 btBU_Simplex1to4(const btVector3& pt0,const btVector3& pt1,const btVector3& pt2,const btVector3& pt3);

    
 void reset()
 {
  m_numVertices = 0;
 }
 
 virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 void addVertex(const btVector3& pt);

 //PolyhedralConvexShape interface

 virtual int getNumVertices() const;

 virtual int getNumEdges() const;

 virtual void getEdge(int i,btVector3& pa,btVector3& pb) const;
 
 virtual void getVertex(int i,btVector3& vtx) const;

 virtual int getNumPlanes() const;

 virtual void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i) const;

 virtual int getIndex(int i) const;

 virtual bool isInside(const btVector3& pt,btScalar tolerance) const;


 ///getName is for debugging
 virtual const char* getName()const { return "btBU_Simplex1to4";}

};

#endif //BT_SIMPLEX_1TO4_SHAPE
//// ../src/BulletCollision/CollisionShapes/btEmptyShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_EMPTY_SHAPE_H
#define BT_EMPTY_SHAPE_H

#include "btConcaveShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"




/// The btEmptyShape is a collision shape without actual collision detection shape, so most users should ignore this class.
/// It can be replaced by another shape during runtime, but the inertia tensor should be recomputed.
class btEmptyShape : public btConcaveShape
{
public:
 btEmptyShape();

 virtual ~btEmptyShape();


 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;


 virtual void setLocalScaling(const btVector3& scaling)
 {
  m_localScaling = scaling;
 }
 virtual const btVector3& getLocalScaling() const 
 {
  return m_localScaling;
 }

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;
 
 virtual const char* getName()const
 {
  return "Empty";
 }

 virtual void processAllTriangles(btTriangleCallback* ,const btVector3& ,const btVector3& ) const
 {
 }

protected:
 btVector3 m_localScaling;

};



#endif //BT_EMPTY_SHAPE_H
//// ../src/BulletCollision/CollisionShapes/btMultiSphereShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_MULTI_SPHERE_MINKOWSKI_H
#define BT_MULTI_SPHERE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btAabbUtil2.h"



///The btMultiSphereShape represents the convex hull of a collection of spheres. You can create special capsules or other smooth volumes.
///It is possible to animate the spheres for deformation, but call 'recalcLocalAabb' after changing any sphere position/radius
class btMultiSphereShape : public btConvexInternalAabbCachingShape
{
 
 btAlignedObjectArray<btVector3> m_localPositionArray;
 btAlignedObjectArray<btScalar>  m_radiArray;
 
public:
 btMultiSphereShape (const btVector3* positions,const btScalar* radi,int numSpheres);

 ///CollisionShape Interface
 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 /// btConvexShape Interface
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
 
 int getSphereCount() const
 {
  return m_localPositionArray.size();
 }

 const btVector3& getSpherePosition(int index) const
 {
  return m_localPositionArray[index];
 }

 btScalar getSphereRadius(int index) const
 {
  return m_radiArray[index];
 }


 virtual const char* getName()const 
 {
  return "MultiSphere";
 }

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};


class btPositionAndRadius
{ public: 
 btVector3FloatData m_pos;
 float  m_radius;
};

class btMultiSphereShapeData
{ public: 
 btConvexInternalShapeData m_convexInternalShapeData;

 btPositionAndRadius *m_localPositionArrayPtr;
 int    m_localPositionArraySize;
 char m_padding[4];
};



 int btMultiSphereShape::calculateSerializeBufferSize() const
{
 return sizeof(btMultiSphereShapeData);
}



#endif //BT_MULTI_SPHERE_MINKOWSKI_H
//// ../src/BulletCollision/CollisionShapes/btUniformScalingShape.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_UNIFORM_SCALING_SHAPE_H
#define BT_UNIFORM_SCALING_SHAPE_H

#include "btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btUniformScalingShape allows to re-use uniform scaled instances of btConvexShape in a memory efficient way.
///Istead of using btUniformScalingShape, it is better to use the non-uniform setLocalScaling method on convex shapes that implement it.
class btUniformScalingShape : public btConvexShape
{
 btConvexShape* m_childConvexShape;

 btScalar m_uniformScalingFactor;
 
 public:
 
 btUniformScalingShape( btConvexShape* convexChildShape, btScalar uniformScalingFactor);
 
 virtual ~btUniformScalingShape();
 
 virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

 virtual btVector3 localGetSupportingVertex(const btVector3& vec)const;

 virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

 btScalar getUniformScalingFactor() const
 {
  return m_uniformScalingFactor;
 }

 btConvexShape* getChildShape() 
 {
  return m_childConvexShape;
 }

 const btConvexShape* getChildShape() const
 {
  return m_childConvexShape;
 }

 virtual const char* getName()const 
 {
  return "UniformScalingShape";
 }
 


 ///////////////////////////


 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

 virtual void setLocalScaling(const btVector3& scaling) ;
 virtual const btVector3& getLocalScaling() const ;

 virtual void setMargin(btScalar margin);
 virtual btScalar getMargin() const;

 virtual int  getNumPreferredPenetrationDirections() const;
 
 virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const;


};

#endif //BT_UNIFORM_SCALING_SHAPE_H
//// ../src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H
#define BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btCollisionDispatcher.h"

class btPersistentManifold;

/// btSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
/// Also provides the most basic sample for custom/user btCollisionAlgorithm
class btSphereSphereCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
 bool m_ownManifold;
 btPersistentManifold* m_manifoldPtr;
 
public:
 btSphereSphereCollisionAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* body0,btCollisionObject* body1);

 btSphereSphereCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
  : btActivatingCollisionAlgorithm(ci) {}

 virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

 virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

 virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
 {
  if (m_manifoldPtr && m_ownManifold)
  {
   manifoldArray.push_back(m_manifoldPtr);
  }
 }
 
 virtual ~btSphereSphereCollisionAlgorithm();

 struct CreateFunc :public  btCollisionAlgorithmCreateFunc
 {
  virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
  {
   void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btSphereSphereCollisionAlgorithm));
   return new(mem) btSphereSphereCollisionAlgorithm(0,ci,body0,body1);
  }
 };

};

#endif //BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H

//// ../src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __BT_ACTIVATING_COLLISION_ALGORITHM_H
#define __BT_ACTIVATING_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

///This class is not enabled yet (work-in-progress) to more aggressively activate objects.
class btActivatingCollisionAlgorithm : public btCollisionAlgorithm
{
// btCollisionObject* m_colObj0;
// btCollisionObject* m_colObj1;

public:

 btActivatingCollisionAlgorithm (const btCollisionAlgorithmConstructionInfo& ci);

 btActivatingCollisionAlgorithm (const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* colObj0,btCollisionObject* colObj1);

 virtual ~btActivatingCollisionAlgorithm();

};
#endif //__BT_ACTIVATING_COLLISION_ALGORITHM_H
//// ../src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_ALGORITHM_H
#define BT_COLLISION_ALGORITHM_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"

struct btBroadphaseProxy;
class btDispatcher;
class btManifoldResult;
class btCollisionObject;
struct btDispatcherInfo;
class btPersistentManifold;

typedef btAlignedObjectArray<btPersistentManifold*> btManifoldArray;

class btCollisionAlgorithmConstructionInfo
{ public: 
 btCollisionAlgorithmConstructionInfo()
  :m_dispatcher1(0),
  m_manifold(0)
 {
 }
 btCollisionAlgorithmConstructionInfo(btDispatcher* dispatcher,int temp)
  :m_dispatcher1(dispatcher)
 {
  (void)temp;
 }

 btDispatcher* m_dispatcher1;
 btPersistentManifold* m_manifold;

// int getDispatcherId();

};


///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
///It is persistent over frames
class btCollisionAlgorithm
{

protected:

 btDispatcher* m_dispatcher;

protected:
// int getDispatcherId();
 
public:

 btCollisionAlgorithm() {};

 btCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci);

 virtual ~btCollisionAlgorithm() {};

 virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut) = 0;

 virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut) = 0;

 virtual void getAllContactManifolds(btManifoldArray& manifoldArray) = 0;
};


#endif //BT_COLLISION_ALGORITHM_H
//// ../src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_DEFAULT_COLLISION_CONFIGURATION
#define BT_DEFAULT_COLLISION_CONFIGURATION

#include "btCollisionConfiguration.h"
class btVoronoiSimplexSolver;
class btConvexPenetrationDepthSolver;

class btDefaultCollisionConstructionInfo
{ public: 
 btStackAlloc*  m_stackAlloc;
 btPoolAllocator* m_persistentManifoldPool;
 btPoolAllocator* m_collisionAlgorithmPool;
 int     m_defaultMaxPersistentManifoldPoolSize;
 int     m_defaultMaxCollisionAlgorithmPoolSize;
 int     m_customCollisionAlgorithmMaxElementSize;
 int     m_defaultStackAllocatorSize;
 int     m_useEpaPenetrationAlgorithm;

 btDefaultCollisionConstructionInfo()
  :m_stackAlloc(0),
  m_persistentManifoldPool(0),
  m_collisionAlgorithmPool(0),
  m_defaultMaxPersistentManifoldPoolSize(4096),
  m_defaultMaxCollisionAlgorithmPoolSize(4096),
  m_customCollisionAlgorithmMaxElementSize(0),
  m_defaultStackAllocatorSize(0),
  m_useEpaPenetrationAlgorithm(true)
 {
 }
};



///btCollisionConfiguration allows to configure Bullet collision detection
///stack allocator, pool memory allocators
///@todo: describe the meaning
class btDefaultCollisionConfiguration : public btCollisionConfiguration
{

protected:

 int m_persistentManifoldPoolSize;
 
 btStackAlloc* m_stackAlloc;
 bool m_ownsStackAllocator;

 btPoolAllocator* m_persistentManifoldPool;
 bool m_ownsPersistentManifoldPool;


 btPoolAllocator* m_collisionAlgorithmPool;
 bool m_ownsCollisionAlgorithmPool;

 //default simplex/penetration depth solvers
 btVoronoiSimplexSolver* m_simplexSolver;
 btConvexPenetrationDepthSolver* m_pdSolver;
 
 //default CreationFunctions, filling the m_doubleDispatch table
 btCollisionAlgorithmCreateFunc* m_convexConvexCreateFunc;
 btCollisionAlgorithmCreateFunc* m_convexConcaveCreateFunc;
 btCollisionAlgorithmCreateFunc* m_swappedConvexConcaveCreateFunc;
 btCollisionAlgorithmCreateFunc* m_compoundCreateFunc;
 btCollisionAlgorithmCreateFunc* m_swappedCompoundCreateFunc;
 btCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
 btCollisionAlgorithmCreateFunc* m_sphereSphereCF;
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
 btCollisionAlgorithmCreateFunc* m_sphereBoxCF;
 btCollisionAlgorithmCreateFunc* m_boxSphereCF;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

 btCollisionAlgorithmCreateFunc* m_boxBoxCF;
 btCollisionAlgorithmCreateFunc* m_sphereTriangleCF;
 btCollisionAlgorithmCreateFunc* m_triangleSphereCF;
 btCollisionAlgorithmCreateFunc* m_planeConvexCF;
 btCollisionAlgorithmCreateFunc* m_convexPlaneCF;
 
public:


 btDefaultCollisionConfiguration(const btDefaultCollisionConstructionInfo& constructionInfo = btDefaultCollisionConstructionInfo());

 virtual ~btDefaultCollisionConfiguration();

  ///memory pools
 virtual btPoolAllocator* getPersistentManifoldPool()
 {
  return m_persistentManifoldPool;
 }

 virtual btPoolAllocator* getCollisionAlgorithmPool()
 {
  return m_collisionAlgorithmPool;
 }

 virtual btStackAlloc* getStackAllocator()
 {
  return m_stackAlloc;
 }

 virtual btVoronoiSimplexSolver* getSimplexSolver()
 {
  return m_simplexSolver;
 }


 virtual btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1);

 ///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
 ///By default, this feature is disabled for best performance.
 ///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
 ///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
 ///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
 ///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
 ///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
 void setConvexConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);

};

#endif //BT_DEFAULT_COLLISION_CONFIGURATION

//// ../src/BulletCollision/CollisionDispatch/btCollisionConfiguration.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_CONFIGURATION
#define BT_COLLISION_CONFIGURATION

struct btCollisionAlgorithmCreateFunc;

class btStackAlloc;
class btPoolAllocator;

///btCollisionConfiguration allows to configure Bullet collision detection
///stack allocator size, default collision algorithms and persistent manifold pool size
///@todo: describe the meaning
class btCollisionConfiguration
{

public:

 virtual ~btCollisionConfiguration()
 {
 }

 ///memory pools
 virtual btPoolAllocator* getPersistentManifoldPool() = 0;

 virtual btPoolAllocator* getCollisionAlgorithmPool() = 0;

 virtual btStackAlloc* getStackAllocator() = 0;

 virtual btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1) =0;

};

#endif //BT_COLLISION_CONFIGURATION

//// ../src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SIMPLE_BROADPHASE_H
#define BT_SIMPLE_BROADPHASE_H


#include "btOverlappingPairCache.h"


class btSimpleBroadphaseProxy : public btBroadphaseProxy
{ public: 
 int   m_nextFree;
 
// int   m_handleId;

 
 btSimpleBroadphaseProxy() {};

 btSimpleBroadphaseProxy(const btVector3& minpt,const btVector3& maxpt,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,void* multiSapProxy)
 :btBroadphaseProxy(minpt,maxpt,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy)
 {
  (void)shapeType;
 }
 
 
  void SetNextFree(int next) {m_nextFree = next;}
  int GetNextFree() const {return m_nextFree;}

 


};

///The SimpleBroadphase is just a unit-test for btAxisSweep3, bt32BitAxisSweep3, or btDbvtBroadphase, so use those classes instead.
///It is a brute force aabb culling broadphase based on O(n^2) aabb checks
class btSimpleBroadphase : public btBroadphaseInterface
{

protected:

 int  m_numHandles;      // number of active handles
 int  m_maxHandles;      // max number of handles
 int  m_LastHandleIndex;       
 
 btSimpleBroadphaseProxy* m_pHandles;      // handles pool

 void* m_pHandlesRawPtr;
 int  m_firstFreeHandle;  // free handles list
 
 int allocHandle()
 {
  btAssert(m_numHandles < m_maxHandles);
  int freeHandle = m_firstFreeHandle;
  m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
  m_numHandles++;
  if(freeHandle > m_LastHandleIndex)
  {
   m_LastHandleIndex = freeHandle;
  }
  return freeHandle;
 }

 void freeHandle(btSimpleBroadphaseProxy* proxy)
 {
  int handle = int(proxy-m_pHandles);
  btAssert(handle >= 0 && handle < m_maxHandles);
  if(handle == m_LastHandleIndex)
  {
   m_LastHandleIndex--;
  }
  proxy->SetNextFree(m_firstFreeHandle);
  m_firstFreeHandle = handle;

  proxy->m_clientObject = 0;

  m_numHandles--;
 }

 btOverlappingPairCache* m_pairCache;
 bool m_ownsPairCache;

 int m_invalidPair;

 
 
 inline btSimpleBroadphaseProxy* getSimpleProxyFromProxy(btBroadphaseProxy* proxy)
 {
  btSimpleBroadphaseProxy* proxy0 = static_cast<btSimpleBroadphaseProxy*>(proxy);
  return proxy0;
 }

 inline const btSimpleBroadphaseProxy* getSimpleProxyFromProxy(btBroadphaseProxy* proxy) const
 {
  const btSimpleBroadphaseProxy* proxy0 = static_cast<const btSimpleBroadphaseProxy*>(proxy);
  return proxy0;
 }

 ///reset broadphase internal structures, to ensure determinism/reproducability
 virtual void resetPool(btDispatcher* dispatcher);


 void validate();

protected:


 

public:
 btSimpleBroadphase(int maxProxies=16384,btOverlappingPairCache* overlappingPairCache=0);
 virtual ~btSimpleBroadphase();


  static bool aabbOverlap(btSimpleBroadphaseProxy* proxy0,btSimpleBroadphaseProxy* proxy1);


 virtual btBroadphaseProxy* createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy);

 virtual void calculateOverlappingPairs(btDispatcher* dispatcher);

 virtual void destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
 virtual void setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher);
 virtual void getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const;

 virtual void rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin,const btVector3& aabbMax);
 virtual void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);
  
 btOverlappingPairCache* getOverlappingPairCache()
 {
  return m_pairCache;
 }
 const btOverlappingPairCache* getOverlappingPairCache() const
 {
  return m_pairCache;
 }

 bool testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);


 ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
 ///will add some transform later
 virtual void getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
 {
  aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
  aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
 }

 virtual void printStats()
 {
//  printf("btSimpleBroadphase.h\n");
//  printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
 }
};



#endif //BT_SIMPLE_BROADPHASE_H

//// ../src/BulletCollision/BroadphaseCollision/btAxisSweep3.h
//Bullet Continuous Collision Detection and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

//
// btAxisSweep3.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.

#ifndef BT_AXIS_SWEEP_3_H
#define BT_AXIS_SWEEP_3_H

#include "LinearMath/btVector3.h"
#include "btOverlappingPairCache.h"
#include "btBroadphaseInterface.h"
#include "btBroadphaseProxy.h"
#include "btOverlappingPairCallback.h"
#include "btDbvtBroadphase.h"

//#define DEBUG_BROADPHASE 1
#define USE_OVERLAP_TEST_ON_REMOVES 1

/// The internal templace class btAxisSweep3Internal implements the sweep and prune broadphase.
/// It uses quantized integers to represent the begin and end points for each of the 3 axis.
/// Dont use this class directly, use btAxisSweep3 or bt32BitAxisSweep3 instead.
template <typename BP_FP_INT_TYPE>
class btAxisSweep3Internal : public btBroadphaseInterface
{
protected:

 BP_FP_INT_TYPE m_bpHandleMask;
 BP_FP_INT_TYPE m_handleSentinel;

public:
 
 

 class Edge
 {
 public:
  BP_FP_INT_TYPE m_pos;   // low bit is min/max
  BP_FP_INT_TYPE m_handle;

  BP_FP_INT_TYPE IsMax() const {return static_cast<BP_FP_INT_TYPE>(m_pos & 1);}
 };

public:
 class Handle : public btBroadphaseProxy
 {
 public:
 
 
  // indexes into the edge arrays
  BP_FP_INT_TYPE m_minEdges[3], m_maxEdges[3];  // 6 * 2 = 12
//  BP_FP_INT_TYPE m_uniqueId;
  btBroadphaseProxy* m_dbvtProxy;//for faster raycast
  //void* m_pOwner; this is now in btBroadphaseProxy.m_clientObject
 
   void SetNextFree(BP_FP_INT_TYPE next) {m_minEdges[0] = next;}
   BP_FP_INT_TYPE GetNextFree() const {return m_minEdges[0];}
 };  // 24 bytes + 24 for Edge structures = 44 bytes total per entry

 
protected:
 btVector3 m_worldAabbMin;      // overall system bounds
 btVector3 m_worldAabbMax;      // overall system bounds

 btVector3 m_quantize;      // scaling factor for quantization

 BP_FP_INT_TYPE m_numHandles;      // number of active handles
 BP_FP_INT_TYPE m_maxHandles;      // max number of handles
 Handle* m_pHandles;      // handles pool
 
 BP_FP_INT_TYPE m_firstFreeHandle;  // free handles list

 Edge* m_pEdges[3];      // edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)
 void* m_pEdgesRawPtr[3];

 btOverlappingPairCache* m_pairCache;

 ///btOverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to btOverlappingPairCache.
 btOverlappingPairCallback* m_userPairCallback;
 
 bool m_ownsPairCache;

 int m_invalidPair;

 ///additional dynamic aabb structure, used to accelerate ray cast queries.
 ///can be disabled using a optional argument in the constructor
 btDbvtBroadphase* m_raycastAccelerator;
 btOverlappingPairCache* m_nullPairCache;


 // allocation/deallocation
 BP_FP_INT_TYPE allocHandle();
 void freeHandle(BP_FP_INT_TYPE handle);
 

 bool testOverlap2D(const Handle* pHandleA, const Handle* pHandleB,int axis0,int axis1);

#ifdef DEBUG_BROADPHASE
 void debugPrintAxis(int axis,bool checkCardinality=true);
#endif //DEBUG_BROADPHASE

 //Overlap* AddOverlap(BP_FP_INT_TYPE handleA, BP_FP_INT_TYPE handleB);
 //void RemoveOverlap(BP_FP_INT_TYPE handleA, BP_FP_INT_TYPE handleB);

 

 void sortMinDown(int axis, BP_FP_INT_TYPE edge, btDispatcher* dispatcher, bool updateOverlaps );
 void sortMinUp(int axis, BP_FP_INT_TYPE edge, btDispatcher* dispatcher, bool updateOverlaps );
 void sortMaxDown(int axis, BP_FP_INT_TYPE edge, btDispatcher* dispatcher, bool updateOverlaps );
 void sortMaxUp(int axis, BP_FP_INT_TYPE edge, btDispatcher* dispatcher, bool updateOverlaps );

public:

 btAxisSweep3Internal(const btVector3& worldAabbMin,const btVector3& worldAabbMax, BP_FP_INT_TYPE handleMask, BP_FP_INT_TYPE handleSentinel, BP_FP_INT_TYPE maxHandles = 16384, btOverlappingPairCache* pairCache=0,bool disableRaycastAccelerator = false);

 virtual ~btAxisSweep3Internal();

 BP_FP_INT_TYPE getNumHandles() const
 {
  return m_numHandles;
 }

 virtual void calculateOverlappingPairs(btDispatcher* dispatcher);
 
 BP_FP_INT_TYPE addHandle(const btVector3& aabbMin,const btVector3& aabbMax, void* pOwner,short int collisionFilterGroup,short int collisionFilterMask,btDispatcher* dispatcher,void* multiSapProxy);
 void removeHandle(BP_FP_INT_TYPE handle,btDispatcher* dispatcher);
 void updateHandle(BP_FP_INT_TYPE handle, const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* dispatcher);
  Handle* getHandle(BP_FP_INT_TYPE index) const {return m_pHandles + index;}

 virtual void resetPool(btDispatcher* dispatcher);

 void processAllOverlappingPairs(btOverlapCallback* callback);

 //Broadphase Interface
 virtual btBroadphaseProxy* createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask,btDispatcher* dispatcher,void* multiSapProxy);
 virtual void destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
 virtual void setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* dispatcher);
 virtual void  getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const;
 
 virtual void rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin, const btVector3& aabbMax);
 virtual void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);

 
 void quantize(BP_FP_INT_TYPE* out, const btVector3& point, int isMax) const;
 ///unQuantize should be conservative: aabbMin/aabbMax should be larger then 'getAabb' result
 void unQuantize(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const;
 
 bool testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

 btOverlappingPairCache* getOverlappingPairCache()
 {
  return m_pairCache;
 }
 const btOverlappingPairCache* getOverlappingPairCache() const
 {
  return m_pairCache;
 }

 void setOverlappingPairUserCallback(btOverlappingPairCallback* pairCallback)
 {
  m_userPairCallback = pairCallback;
 }
 const btOverlappingPairCallback* getOverlappingPairUserCallback() const
 {
  return m_userPairCallback;
 }

 ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
 ///will add some transform later
 virtual void getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
 {
  aabbMin = m_worldAabbMin;
  aabbMax = m_worldAabbMax;
 }

 virtual void printStats()
 {
/*  printf("btAxisSweep3.h\n");
  printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
  printf("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.getX(),m_worldAabbMin.getY(),m_worldAabbMin.getZ(),
   m_worldAabbMax.getX(),m_worldAabbMax.getY(),m_worldAabbMax.getZ());
   */

 }

};

////////////////////////////////////////////////////////////////////




#ifdef DEBUG_BROADPHASE
#include <stdio.h>

template <typename BP_FP_INT_TYPE>
void btAxisSweep3<BP_FP_INT_TYPE>::debugPrintAxis(int axis, bool checkCardinality)
{
 int numEdges = m_pHandles[0].m_maxEdges[axis];
 printf("SAP Axis %d, numEdges=%d\n",axis,numEdges);

 int i;
 for (i=0;i<numEdges+1;i++)
 {
  Edge* pEdge = m_pEdges[axis] + i;
  Handle* pHandlePrev = getHandle(pEdge->m_handle);
  int handleIndex = pEdge->IsMax()? pHandlePrev->m_maxEdges[axis] : pHandlePrev->m_minEdges[axis];
  char beginOrEnd;
  beginOrEnd=pEdge->IsMax()?'E':'B';
  printf(" [%c,h=%d,p=%x,i=%d]\n",beginOrEnd,pEdge->m_handle,pEdge->m_pos,handleIndex);
 }

 if (checkCardinality)
  btAssert(numEdges == m_numHandles*2+1);
}
#endif //DEBUG_BROADPHASE

template <typename BP_FP_INT_TYPE>
btBroadphaseProxy* btAxisSweep3Internal<BP_FP_INT_TYPE>::createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,btDispatcher* dispatcher,void* multiSapProxy)
{
  (void)shapeType;
  BP_FP_INT_TYPE handleId = addHandle(aabbMin,aabbMax, userPtr,collisionFilterGroup,collisionFilterMask,dispatcher,multiSapProxy);
  
  Handle* handle = getHandle(handleId);
  
  if (m_raycastAccelerator)
  {
   btBroadphaseProxy* rayProxy = m_raycastAccelerator->createProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,dispatcher,0);
   handle->m_dbvtProxy = rayProxy;
  }
  return handle;
}



template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher)
{
 Handle* handle = static_cast<Handle*>(proxy);
 if (m_raycastAccelerator)
  m_raycastAccelerator->destroyProxy(handle->m_dbvtProxy,dispatcher);
 removeHandle(static_cast<BP_FP_INT_TYPE>(handle->m_uniqueId), dispatcher);
}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* dispatcher)
{
 Handle* handle = static_cast<Handle*>(proxy);
 handle->m_aabbMin = aabbMin;
 handle->m_aabbMax = aabbMax;
 updateHandle(static_cast<BP_FP_INT_TYPE>(handle->m_uniqueId), aabbMin, aabbMax,dispatcher);
 if (m_raycastAccelerator)
  m_raycastAccelerator->setAabb(handle->m_dbvtProxy,aabbMin,aabbMax,dispatcher);

}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback,const btVector3& aabbMin,const btVector3& aabbMax)
{
 if (m_raycastAccelerator)
 {
  m_raycastAccelerator->rayTest(rayFrom,rayTo,rayCallback,aabbMin,aabbMax);
 } else
 {
  //choose axis?
  BP_FP_INT_TYPE axis = 0;
  //for each proxy
  for (BP_FP_INT_TYPE i=1;i<m_numHandles*2+1;i++)
  {
   if (m_pEdges[axis][i].IsMax())
   {
    rayCallback.process(getHandle(m_pEdges[axis][i].m_handle));
   }
  }
 }
}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback)
{
 if (m_raycastAccelerator)
 {
  m_raycastAccelerator->aabbTest(aabbMin,aabbMax,callback);
 } else
 {
  //choose axis?
  BP_FP_INT_TYPE axis = 0;
  //for each proxy
  for (BP_FP_INT_TYPE i=1;i<m_numHandles*2+1;i++)
  {
   if (m_pEdges[axis][i].IsMax())
   {
    Handle* handle = getHandle(m_pEdges[axis][i].m_handle);
    if (TestAabbAgainstAabb2(aabbMin,aabbMax,handle->m_aabbMin,handle->m_aabbMax))
    {
     callback.process(handle);
    }
   }
  }
 }
}



template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const
{
 Handle* pHandle = static_cast<Handle*>(proxy);
 aabbMin = pHandle->m_aabbMin;
 aabbMax = pHandle->m_aabbMax;
}


template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::unQuantize(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const
{
 Handle* pHandle = static_cast<Handle*>(proxy);

 unsigned short vecInMin[3];
 unsigned short vecInMax[3];

 vecInMin[0] = m_pEdges[0][pHandle->m_minEdges[0]].m_pos ;
 vecInMax[0] = m_pEdges[0][pHandle->m_maxEdges[0]].m_pos +1 ;
 vecInMin[1] = m_pEdges[1][pHandle->m_minEdges[1]].m_pos ;
 vecInMax[1] = m_pEdges[1][pHandle->m_maxEdges[1]].m_pos +1 ;
 vecInMin[2] = m_pEdges[2][pHandle->m_minEdges[2]].m_pos ;
 vecInMax[2] = m_pEdges[2][pHandle->m_maxEdges[2]].m_pos +1 ;
 
 aabbMin.setValue((btScalar)(vecInMin[0]) / (m_quantize.getX()),(btScalar)(vecInMin[1]) / (m_quantize.getY()),(btScalar)(vecInMin[2]) / (m_quantize.getZ()));
 aabbMin += m_worldAabbMin;
 
 aabbMax.setValue((btScalar)(vecInMax[0]) / (m_quantize.getX()),(btScalar)(vecInMax[1]) / (m_quantize.getY()),(btScalar)(vecInMax[2]) / (m_quantize.getZ()));
 aabbMax += m_worldAabbMin;
}




template <typename BP_FP_INT_TYPE>
btAxisSweep3Internal<BP_FP_INT_TYPE>::btAxisSweep3Internal(const btVector3& worldAabbMin,const btVector3& worldAabbMax, BP_FP_INT_TYPE handleMask, BP_FP_INT_TYPE handleSentinel,BP_FP_INT_TYPE userMaxHandles, btOverlappingPairCache* pairCache , bool disableRaycastAccelerator)
:m_bpHandleMask(handleMask),
m_handleSentinel(handleSentinel),
m_pairCache(pairCache),
m_userPairCallback(0),
m_ownsPairCache(false),
m_invalidPair(0),
m_raycastAccelerator(0)
{
 BP_FP_INT_TYPE maxHandles = static_cast<BP_FP_INT_TYPE>(userMaxHandles+1);//need to add one sentinel handle

 if (!m_pairCache)
 {
  void* ptr = btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16);
  m_pairCache = new(ptr) btHashedOverlappingPairCache();
  m_ownsPairCache = true;
 }

 if (!disableRaycastAccelerator)
 {
  m_nullPairCache = new (btAlignedAlloc(sizeof(btNullPairCache),16)) btNullPairCache();
  m_raycastAccelerator = new (btAlignedAlloc(sizeof(btDbvtBroadphase),16)) btDbvtBroadphase(m_nullPairCache);//m_pairCache);
  m_raycastAccelerator->m_deferedcollide = true;//don't add/remove pairs
 }

 //btAssert(bounds.HasVolume());

 // init bounds
 m_worldAabbMin = worldAabbMin;
 m_worldAabbMax = worldAabbMax;

 btVector3 aabbSize = m_worldAabbMax - m_worldAabbMin;

 BP_FP_INT_TYPE maxInt = m_handleSentinel;

 m_quantize = btVector3(btScalar(maxInt),btScalar(maxInt),btScalar(maxInt)) / aabbSize;

 // allocate handles buffer, using btAlignedAlloc, and put all handles on free list
 m_pHandles = new Handle[maxHandles];
 
 m_maxHandles = maxHandles;
 m_numHandles = 0;

 // handle 0 is reserved as the null index, and is also used as the sentinel
 m_firstFreeHandle = 1;
 {
  for (BP_FP_INT_TYPE i = m_firstFreeHandle; i < maxHandles; i++)
   m_pHandles[i].SetNextFree(static_cast<BP_FP_INT_TYPE>(i + 1));
  m_pHandles[maxHandles - 1].SetNextFree(0);
 }

 {
  // allocate edge buffers
  for (int i = 0; i < 3; i++)
  {
   m_pEdgesRawPtr[i] = btAlignedAlloc(sizeof(Edge)*maxHandles*2,16);
   m_pEdges[i] = new(m_pEdgesRawPtr[i]) Edge[maxHandles * 2];
  }
 }
 //removed overlap management

 // make boundary sentinels
 
 m_pHandles[0].m_clientObject = 0;

 for (int axis = 0; axis < 3; axis++)
 {
  m_pHandles[0].m_minEdges[axis] = 0;
  m_pHandles[0].m_maxEdges[axis] = 1;

  m_pEdges[axis][0].m_pos = 0;
  m_pEdges[axis][0].m_handle = 0;
  m_pEdges[axis][1].m_pos = m_handleSentinel;
  m_pEdges[axis][1].m_handle = 0;
#ifdef DEBUG_BROADPHASE
  debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

 }

}

template <typename BP_FP_INT_TYPE>
btAxisSweep3Internal<BP_FP_INT_TYPE>::~btAxisSweep3Internal()
{
 if (m_raycastAccelerator)
 {
  m_nullPairCache->~btOverlappingPairCache();
  btAlignedFree(m_nullPairCache);
  m_raycastAccelerator->~btDbvtBroadphase();
  btAlignedFree (m_raycastAccelerator);
 }

 for (int i = 2; i >= 0; i--)
 {
  btAlignedFree(m_pEdgesRawPtr[i]);
 }
 delete [] m_pHandles;

 if (m_ownsPairCache)
 {
  m_pairCache->~btOverlappingPairCache();
  btAlignedFree(m_pairCache);
 }
}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::quantize(BP_FP_INT_TYPE* out, const btVector3& point, int isMax) const
{
#ifdef OLD_CLAMPING_METHOD
 ///problem with this clamping method is that the floating point during quantization might still go outside the range [(0|isMax) .. (m_handleSentinel&m_bpHandleMask]|isMax]
 ///see http://code.google.com/p/bullet/issues/detail?id=87
 btVector3 clampedPoint(point);
 clampedPoint.setMax(m_worldAabbMin);
 clampedPoint.setMin(m_worldAabbMax);
 btVector3 v = (clampedPoint - m_worldAabbMin) * m_quantize;
 out[0] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getX() & m_bpHandleMask) | isMax);
 out[1] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getY() & m_bpHandleMask) | isMax);
 out[2] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getZ() & m_bpHandleMask) | isMax);
#else
 btVector3 v = (point - m_worldAabbMin) * m_quantize;
 out[0]=(v[0]<=0)?(BP_FP_INT_TYPE)isMax:(v[0]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[0]&m_bpHandleMask)|isMax);
 out[1]=(v[1]<=0)?(BP_FP_INT_TYPE)isMax:(v[1]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[1]&m_bpHandleMask)|isMax);
 out[2]=(v[2]<=0)?(BP_FP_INT_TYPE)isMax:(v[2]>=m_handleSentinel)?(BP_FP_INT_TYPE)((m_handleSentinel&m_bpHandleMask)|isMax):(BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[2]&m_bpHandleMask)|isMax);
#endif //OLD_CLAMPING_METHOD
}


template <typename BP_FP_INT_TYPE>
BP_FP_INT_TYPE btAxisSweep3Internal<BP_FP_INT_TYPE>::allocHandle()
{
 btAssert(m_firstFreeHandle);

 BP_FP_INT_TYPE handle = m_firstFreeHandle;
 m_firstFreeHandle = getHandle(handle)->GetNextFree();
 m_numHandles++;

 return handle;
}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::freeHandle(BP_FP_INT_TYPE handle)
{
 btAssert(handle > 0 && handle < m_maxHandles);

 getHandle(handle)->SetNextFree(m_firstFreeHandle);
 m_firstFreeHandle = handle;

 m_numHandles--;
}


template <typename BP_FP_INT_TYPE>
BP_FP_INT_TYPE btAxisSweep3Internal<BP_FP_INT_TYPE>::addHandle(const btVector3& aabbMin,const btVector3& aabbMax, void* pOwner,short int collisionFilterGroup,short int collisionFilterMask,btDispatcher* dispatcher,void* multiSapProxy)
{
 // quantize the bounds
 BP_FP_INT_TYPE min[3], max[3];
 quantize(min, aabbMin, 0);
 quantize(max, aabbMax, 1);

 // allocate a handle
 BP_FP_INT_TYPE handle = allocHandle();
 

 Handle* pHandle = getHandle(handle);
 
 pHandle->m_uniqueId = static_cast<int>(handle);
 //pHandle->m_pOverlaps = 0;
 pHandle->m_clientObject = pOwner;
 pHandle->m_collisionFilterGroup = collisionFilterGroup;
 pHandle->m_collisionFilterMask = collisionFilterMask;
 pHandle->m_multiSapParentProxy = multiSapProxy;

 // compute current limit of edge arrays
 BP_FP_INT_TYPE limit = static_cast<BP_FP_INT_TYPE>(m_numHandles * 2);

 
 // insert new edges just inside the max boundary edge
 for (BP_FP_INT_TYPE axis = 0; axis < 3; axis++)
 {

  m_pHandles[0].m_maxEdges[axis] += 2;

  m_pEdges[axis][limit + 1] = m_pEdges[axis][limit - 1];

  m_pEdges[axis][limit - 1].m_pos = min[axis];
  m_pEdges[axis][limit - 1].m_handle = handle;

  m_pEdges[axis][limit].m_pos = max[axis];
  m_pEdges[axis][limit].m_handle = handle;

  pHandle->m_minEdges[axis] = static_cast<BP_FP_INT_TYPE>(limit - 1);
  pHandle->m_maxEdges[axis] = limit;
 }

 // now sort the new edges to their correct position
 sortMinDown(0, pHandle->m_minEdges[0], dispatcher,false);
 sortMaxDown(0, pHandle->m_maxEdges[0], dispatcher,false);
 sortMinDown(1, pHandle->m_minEdges[1], dispatcher,false);
 sortMaxDown(1, pHandle->m_maxEdges[1], dispatcher,false);
 sortMinDown(2, pHandle->m_minEdges[2], dispatcher,true);
 sortMaxDown(2, pHandle->m_maxEdges[2], dispatcher,true);


 return handle;
}


template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::removeHandle(BP_FP_INT_TYPE handle,btDispatcher* dispatcher)
{

 Handle* pHandle = getHandle(handle);

 //explicitly remove the pairs containing the proxy
 //we could do it also in the sortMinUp (passing true)
 ///@todo: compare performance
 if (!m_pairCache->hasDeferredRemoval())
 {
  m_pairCache->removeOverlappingPairsContainingProxy(pHandle,dispatcher);
 }

 // compute current limit of edge arrays
 int limit = static_cast<int>(m_numHandles * 2);
 
 int axis;

 for (axis = 0;axis<3;axis++)
 {
  m_pHandles[0].m_maxEdges[axis] -= 2;
 }

 // remove the edges by sorting them up to the end of the list
 for ( axis = 0; axis < 3; axis++)
 {
  Edge* pEdges = m_pEdges[axis];
  BP_FP_INT_TYPE max = pHandle->m_maxEdges[axis];
  pEdges[max].m_pos = m_handleSentinel;

  sortMaxUp(axis,max,dispatcher,false);


  BP_FP_INT_TYPE i = pHandle->m_minEdges[axis];
  pEdges[i].m_pos = m_handleSentinel;


  sortMinUp(axis,i,dispatcher,false);

  pEdges[limit-1].m_handle = 0;
  pEdges[limit-1].m_pos = m_handleSentinel;
  
#ifdef DEBUG_BROADPHASE
   debugPrintAxis(axis,false);
#endif //DEBUG_BROADPHASE


 }


 // free the handle
 freeHandle(handle);

 
}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::resetPool(btDispatcher* dispatcher)
{
 if (m_numHandles == 0)
 {
  m_firstFreeHandle = 1;
  {
   for (BP_FP_INT_TYPE i = m_firstFreeHandle; i < m_maxHandles; i++)
    m_pHandles[i].SetNextFree(static_cast<BP_FP_INT_TYPE>(i + 1));
   m_pHandles[m_maxHandles - 1].SetNextFree(0);
  }
 }
}       


extern int gOverlappingPairs;
//#include <stdio.h>

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::calculateOverlappingPairs(btDispatcher* dispatcher)
{

 if (m_pairCache->hasDeferredRemoval())
 {
 
  btBroadphasePairArray& overlappingPairArray = m_pairCache->getOverlappingPairArray();

  //perform a sort, to find duplicates and to sort 'invalid' pairs to the end
  overlappingPairArray.quickSort(btBroadphasePairSortPredicate());

  overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
  m_invalidPair = 0;

  
  int i;

  btBroadphasePair previousPair;
  previousPair.m_pProxy0 = 0;
  previousPair.m_pProxy1 = 0;
  previousPair.m_algorithm = 0;
  
  
  for (i=0;i<overlappingPairArray.size();i++)
  {
  
   btBroadphasePair& pair = overlappingPairArray[i];

   bool isDuplicate = (pair == previousPair);

   previousPair = pair;

   bool needsRemoval = false;

   if (!isDuplicate)
   {
    ///important to use an AABB test that is consistent with the broadphase
    bool hasOverlap = testAabbOverlap(pair.m_pProxy0,pair.m_pProxy1);

    if (hasOverlap)
    {
     needsRemoval = false;//callback->processOverlap(pair);
    } else
    {
     needsRemoval = true;
    }
   } else
   {
    //remove duplicate
    needsRemoval = true;
    //should have no algorithm
    btAssert(!pair.m_algorithm);
   }
   
   if (needsRemoval)
   {
    m_pairCache->cleanOverlappingPair(pair,dispatcher);

  //  m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
  //  m_overlappingPairArray.pop_back();
    pair.m_pProxy0 = 0;
    pair.m_pProxy1 = 0;
    m_invalidPair++;
    gOverlappingPairs--;
   } 
   
  }

 ///if you don't like to skip the invalid pairs in the array, execute following code:
 #define CLEAN_INVALID_PAIRS 1
 #ifdef CLEAN_INVALID_PAIRS

  //perform a sort, to sort 'invalid' pairs to the end
  overlappingPairArray.quickSort(btBroadphasePairSortPredicate());

  overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
  m_invalidPair = 0;
 #endif//CLEAN_INVALID_PAIRS
  
  //printf("overlappingPairArray.size()=%d\n",overlappingPairArray.size());
 }

}


template <typename BP_FP_INT_TYPE>
bool btAxisSweep3Internal<BP_FP_INT_TYPE>::testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
{
 const Handle* pHandleA = static_cast<Handle*>(proxy0);
 const Handle* pHandleB = static_cast<Handle*>(proxy1);
 
 //optimization 1: check the array index (memory address), instead of the m_pos

 for (int axis = 0; axis < 3; axis++)
 { 
  if (pHandleA->m_maxEdges[axis] < pHandleB->m_minEdges[axis] || 
   pHandleB->m_maxEdges[axis] < pHandleA->m_minEdges[axis]) 
  { 
   return false; 
  } 
 } 
 return true;
}

template <typename BP_FP_INT_TYPE>
bool btAxisSweep3Internal<BP_FP_INT_TYPE>::testOverlap2D(const Handle* pHandleA, const Handle* pHandleB,int axis0,int axis1)
{
 //optimization 1: check the array index (memory address), instead of the m_pos

 if (pHandleA->m_maxEdges[axis0] < pHandleB->m_minEdges[axis0] || 
  pHandleB->m_maxEdges[axis0] < pHandleA->m_minEdges[axis0] ||
  pHandleA->m_maxEdges[axis1] < pHandleB->m_minEdges[axis1] ||
  pHandleB->m_maxEdges[axis1] < pHandleA->m_minEdges[axis1]) 
 { 
  return false; 
 } 
 return true;
}

template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::updateHandle(BP_FP_INT_TYPE handle, const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* dispatcher)
{
// btAssert(bounds.IsFinite());
 //btAssert(bounds.HasVolume());

 Handle* pHandle = getHandle(handle);

 // quantize the new bounds
 BP_FP_INT_TYPE min[3], max[3];
 quantize(min, aabbMin, 0);
 quantize(max, aabbMax, 1);

 // update changed edges
 for (int axis = 0; axis < 3; axis++)
 {
  BP_FP_INT_TYPE emin = pHandle->m_minEdges[axis];
  BP_FP_INT_TYPE emax = pHandle->m_maxEdges[axis];

  int dmin = (int)min[axis] - (int)m_pEdges[axis][emin].m_pos;
  int dmax = (int)max[axis] - (int)m_pEdges[axis][emax].m_pos;

  m_pEdges[axis][emin].m_pos = min[axis];
  m_pEdges[axis][emax].m_pos = max[axis];

  // expand (only adds overlaps)
  if (dmin < 0)
   sortMinDown(axis, emin,dispatcher,true);

  if (dmax > 0)
   sortMaxUp(axis, emax,dispatcher,true);

  // shrink (only removes overlaps)
  if (dmin > 0)
   sortMinUp(axis, emin,dispatcher,true);

  if (dmax < 0)
   sortMaxDown(axis, emax,dispatcher,true);

#ifdef DEBUG_BROADPHASE
 debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE
 }

 
}




// sorting a min edge downwards can only ever *add* overlaps
template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMinDown(int axis, BP_FP_INT_TYPE edge, btDispatcher* /* dispatcher */, bool updateOverlaps)
{

 Edge* pEdge = m_pEdges[axis] + edge;
 Edge* pPrev = pEdge - 1;
 Handle* pHandleEdge = getHandle(pEdge->m_handle);

 while (pEdge->m_pos < pPrev->m_pos)
 {
  Handle* pHandlePrev = getHandle(pPrev->m_handle);

  if (pPrev->IsMax())
  {
   // if previous edge is a maximum check the bounds and add an overlap if necessary
   const int axis1 = (1  << axis) & 3;
   const int axis2 = (1  << axis1) & 3;
   if (updateOverlaps && testOverlap2D(pHandleEdge, pHandlePrev,axis1,axis2))
   {
    m_pairCache->addOverlappingPair(pHandleEdge,pHandlePrev);
    if (m_userPairCallback)
     m_userPairCallback->addOverlappingPair(pHandleEdge,pHandlePrev);

    //AddOverlap(pEdge->m_handle, pPrev->m_handle);

   }

   // update edge reference in other handle
   pHandlePrev->m_maxEdges[axis]++;
  }
  else
   pHandlePrev->m_minEdges[axis]++;

  pHandleEdge->m_minEdges[axis]--;

  // swap the edges
  Edge swap = *pEdge;
  *pEdge = *pPrev;
  *pPrev = swap;

  // decrement
  pEdge--;
  pPrev--;
 }

#ifdef DEBUG_BROADPHASE
 debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

}

// sorting a min edge upwards can only ever *remove* overlaps
template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMinUp(int axis, BP_FP_INT_TYPE edge, btDispatcher* dispatcher, bool updateOverlaps)
{
 Edge* pEdge = m_pEdges[axis] + edge;
 Edge* pNext = pEdge + 1;
 Handle* pHandleEdge = getHandle(pEdge->m_handle);

 while (pNext->m_handle && (pEdge->m_pos >= pNext->m_pos))
 {
  Handle* pHandleNext = getHandle(pNext->m_handle);

  if (pNext->IsMax())
  {
   Handle* handle0 = getHandle(pEdge->m_handle);
   Handle* handle1 = getHandle(pNext->m_handle);
   const int axis1 = (1  << axis) & 3;
   const int axis2 = (1  << axis1) & 3;
   
   // if next edge is maximum remove any overlap between the two handles
   if (updateOverlaps 
#ifdef USE_OVERLAP_TEST_ON_REMOVES
    && testOverlap2D(handle0,handle1,axis1,axis2)
#endif //USE_OVERLAP_TEST_ON_REMOVES
    )
   {
    

    m_pairCache->removeOverlappingPair(handle0,handle1,dispatcher); 
    if (m_userPairCallback)
     m_userPairCallback->removeOverlappingPair(handle0,handle1,dispatcher);
    
   }


   // update edge reference in other handle
   pHandleNext->m_maxEdges[axis]--;
  }
  else
   pHandleNext->m_minEdges[axis]--;

  pHandleEdge->m_minEdges[axis]++;

  // swap the edges
  Edge swap = *pEdge;
  *pEdge = *pNext;
  *pNext = swap;

  // increment
  pEdge++;
  pNext++;
 }


}

// sorting a max edge downwards can only ever *remove* overlaps
template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMaxDown(int axis, BP_FP_INT_TYPE edge, btDispatcher* dispatcher, bool updateOverlaps)
{

 Edge* pEdge = m_pEdges[axis] + edge;
 Edge* pPrev = pEdge - 1;
 Handle* pHandleEdge = getHandle(pEdge->m_handle);

 while (pEdge->m_pos < pPrev->m_pos)
 {
  Handle* pHandlePrev = getHandle(pPrev->m_handle);

  if (!pPrev->IsMax())
  {
   // if previous edge was a minimum remove any overlap between the two handles
   Handle* handle0 = getHandle(pEdge->m_handle);
   Handle* handle1 = getHandle(pPrev->m_handle);
   const int axis1 = (1  << axis) & 3;
   const int axis2 = (1  << axis1) & 3;

   if (updateOverlaps  
#ifdef USE_OVERLAP_TEST_ON_REMOVES
    && testOverlap2D(handle0,handle1,axis1,axis2)
#endif //USE_OVERLAP_TEST_ON_REMOVES
    )
   {
    //this is done during the overlappingpairarray iteration/narrowphase collision

    
    m_pairCache->removeOverlappingPair(handle0,handle1,dispatcher);
    if (m_userPairCallback)
     m_userPairCallback->removeOverlappingPair(handle0,handle1,dispatcher);
   


   }

   // update edge reference in other handle
   pHandlePrev->m_minEdges[axis]++;;
  }
  else
   pHandlePrev->m_maxEdges[axis]++;

  pHandleEdge->m_maxEdges[axis]--;

  // swap the edges
  Edge swap = *pEdge;
  *pEdge = *pPrev;
  *pPrev = swap;

  // decrement
  pEdge--;
  pPrev--;
 }

 
#ifdef DEBUG_BROADPHASE
 debugPrintAxis(axis);
#endif //DEBUG_BROADPHASE

}

// sorting a max edge upwards can only ever *add* overlaps
template <typename BP_FP_INT_TYPE>
void btAxisSweep3Internal<BP_FP_INT_TYPE>::sortMaxUp(int axis, BP_FP_INT_TYPE edge, btDispatcher* /* dispatcher */, bool updateOverlaps)
{
 Edge* pEdge = m_pEdges[axis] + edge;
 Edge* pNext = pEdge + 1;
 Handle* pHandleEdge = getHandle(pEdge->m_handle);

 while (pNext->m_handle && (pEdge->m_pos >= pNext->m_pos))
 {
  Handle* pHandleNext = getHandle(pNext->m_handle);

  const int axis1 = (1  << axis) & 3;
  const int axis2 = (1  << axis1) & 3;

  if (!pNext->IsMax())
  {
   // if next edge is a minimum check the bounds and add an overlap if necessary
   if (updateOverlaps && testOverlap2D(pHandleEdge, pHandleNext,axis1,axis2))
   {
    Handle* handle0 = getHandle(pEdge->m_handle);
    Handle* handle1 = getHandle(pNext->m_handle);
    m_pairCache->addOverlappingPair(handle0,handle1);
    if (m_userPairCallback)
     m_userPairCallback->addOverlappingPair(handle0,handle1);
   }

   // update edge reference in other handle
   pHandleNext->m_minEdges[axis]--;
  }
  else
   pHandleNext->m_maxEdges[axis]--;

  pHandleEdge->m_maxEdges[axis]++;

  // swap the edges
  Edge swap = *pEdge;
  *pEdge = *pNext;
  *pNext = swap;

  // increment
  pEdge++;
  pNext++;
 }
 
}



////////////////////////////////////////////////////////////////////


/// The btAxisSweep3 is an efficient implementation of the 3d axis sweep and prune broadphase.
/// It uses arrays rather then lists for storage of the 3 axis. Also it operates using 16 bit integer coordinates instead of floats.
/// For large worlds and many objects, use bt32BitAxisSweep3 or btDbvtBroadphase instead. bt32BitAxisSweep3 has higher precision and allows more then 16384 objects at the cost of more memory and bit of performance.
class btAxisSweep3 : public btAxisSweep3Internal<unsigned short int>
{
public:

 btAxisSweep3(const btVector3& worldAabbMin,const btVector3& worldAabbMax, unsigned short int maxHandles = 16384, btOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false);

};

/// The bt32BitAxisSweep3 allows higher precision quantization and more objects compared to the btAxisSweep3 sweep and prune.
/// This comes at the cost of more memory per handle, and a bit slower performance.
/// It uses arrays rather then lists for storage of the 3 axis.
class bt32BitAxisSweep3 : public btAxisSweep3Internal<unsigned int>
{
public:

 bt32BitAxisSweep3(const btVector3& worldAabbMin,const btVector3& worldAabbMax, unsigned int maxHandles = 1500000, btOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false);

};

#endif

//// ../src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btDbvtBroadphase implementation by Nathanael Presson
#ifndef BT_DBVT_BROADPHASE_H
#define BT_DBVT_BROADPHASE_H

#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

//
// Compile time config
//

#define DBVT_BP_PROFILE     0
//#define DBVT_BP_SORTPAIRS    1
#define DBVT_BP_PREVENTFALSEUPDATE  0
#define DBVT_BP_ACCURATESLEEPING  0
#define DBVT_BP_ENABLE_BENCHMARK  0
#define DBVT_BP_MARGIN     (btScalar)0.05

#if DBVT_BP_PROFILE
#define DBVT_BP_PROFILING_RATE 256
#include "LinearMath/btQuickprof.h"
#endif

//
// btDbvtProxy
//
class btDbvtProxy : btBroadphaseProxy
{ public: 
 /* Fields  */ 
 //btDbvtAabbMm aabb;
 btDbvtNode*  leaf;
 btDbvtProxy* links[2];
 int    stage;
 /* ctor   */ 
 btDbvtProxy(const btVector3& aabbMin,const btVector3& aabbMax,void* userPtr,short int collisionFilterGroup, short int collisionFilterMask) :
 btBroadphaseProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask)
 {
  links[0]=links[1]=0;
 }
};

typedef btAlignedObjectArray<btDbvtProxy*> btDbvtProxyArray;

///The btDbvtBroadphase implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see btDbvt).
///One tree is used for static/non-moving objects, and another tree is used for dynamic objects. Objects can move from one tree to the other.
///This is a very fast broadphase, especially for very dynamic worlds where many objects are moving. Its insert/add and remove of objects is generally faster than the sweep and prune broadphases btAxisSweep3 and bt32BitAxisSweep3.
class btDbvtBroadphase : btBroadphaseInterface
{ public: 
 /* Config  */ 
 enum {
  DYNAMIC_SET   = 0, /* Dynamic set index */ 
  FIXED_SET   = 1, /* Fixed set index  */ 
  STAGECOUNT   = 2 /* Number of stages  */ 
 };
 /* Fields  */ 
 btDbvt     m_sets[2];     // Dbvt sets
 btDbvtProxy*   m_stageRoots[STAGECOUNT+1]; // Stages list
 btOverlappingPairCache* m_paircache;    // Pair cache
 btScalar    m_prediction;    // Velocity prediction
 int      m_stageCurrent;    // Current stage
 int      m_fupdates;     // % of fixed updates per frame
 int      m_dupdates;     // % of dynamic updates per frame
 int      m_cupdates;     // % of cleanup updates per frame
 int      m_newpairs;     // Number of pairs created
 int      m_fixedleft;    // Fixed optimization left
 unsigned    m_updates_call;    // Number of updates call
 unsigned    m_updates_done;    // Number of updates done
 btScalar    m_updates_ratio;   // m_updates_done/m_updates_call
 int      m_pid;      // Parse id
 int      m_cid;      // Cleanup index
 int      m_gid;      // Gen id
 bool     m_releasepaircache;   // Release pair cache on delete
 bool     m_deferedcollide;   // Defere dynamic/static collision to collide call
 bool     m_needcleanup;    // Need to run cleanup?
#if DBVT_BP_PROFILE
 btClock     m_clock;
 struct {
  unsigned long  m_total;
  unsigned long  m_ddcollide;
  unsigned long  m_fdcollide;
  unsigned long  m_cleanup;
  unsigned long  m_jobcount;
 }    m_profiling;
#endif
 /* Methods  */ 
 btDbvtBroadphase(btOverlappingPairCache* paircache=0);
 ~btDbvtBroadphase();
 void       collide(btDispatcher* dispatcher);
 void       optimize();
 
 /* btBroadphaseInterface Implementation */
 btBroadphaseProxy*    createProxy(const btVector3& aabbMin,const btVector3& aabbMax,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,btDispatcher* dispatcher,void* multiSapProxy);
 virtual void     destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
 virtual void     setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* dispatcher);
 virtual void     rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin, const btVector3& aabbMax);
 virtual void     aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);

 virtual void     getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const;
 virtual void     calculateOverlappingPairs(btDispatcher* dispatcher);
 virtual btOverlappingPairCache* getOverlappingPairCache();
 virtual const btOverlappingPairCache* getOverlappingPairCache() const;
 virtual void     getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const;
 virtual void     printStats();


 ///reset broadphase internal structures, to ensure determinism/reproducability
 virtual void resetPool(btDispatcher* dispatcher);

 void performDeferredRemoval(btDispatcher* dispatcher);
 
 void setVelocityPrediction(btScalar prediction)
 {
  m_prediction = prediction;
 }
 btScalar getVelocityPrediction() const
 {
  return m_prediction;
 }

 ///this setAabbForceUpdate is similar to setAabb but always forces the aabb update. 
 ///it is not part of the btBroadphaseInterface but specific to btDbvtBroadphase.
 ///it bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
 ///http://code.google.com/p/bullet/issues/detail?id=223
 void       setAabbForceUpdate(  btBroadphaseProxy* absproxy,const btVector3& aabbMin,const btVector3& aabbMax,btDispatcher* /*dispatcher*/);

 static void      benchmark(btBroadphaseInterface*);


};

#endif
//// ../src/BulletCollision/BroadphaseCollision/btDbvt.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btDbvt implementation by Nathanael Presson

#ifndef BT_DYNAMIC_BOUNDING_VOLUME_TREE_H
#define BT_DYNAMIC_BOUNDING_VOLUME_TREE_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btAabbUtil2.h"

//
// Compile time configuration
//


// Implementation profiles
#define DBVT_IMPL_GENERIC  0 // Generic implementation 
#define DBVT_IMPL_SSE   1 // SSE

// Template implementation of ICollide
#ifdef _WIN32
#if (defined (_MSC_VER) && _MSC_VER >= 1400)
#define DBVT_USE_TEMPLATE  1
#else
#define DBVT_USE_TEMPLATE  0
#endif
#else
#define DBVT_USE_TEMPLATE  0
#endif

// Use only intrinsics instead of inline asm
#define DBVT_USE_INTRINSIC_SSE 1

// Using memmov for collideOCL
#define DBVT_USE_MEMMOVE  1

// Enable benchmarking code
#define DBVT_ENABLE_BENCHMARK 0

// Inlining
#define DBVT_INLINE    

// Specific methods implementation

//SSE gives errors on a MSVC 7.1
#if defined (BT_USE_SSE) && defined (_WIN32)
#define DBVT_SELECT_IMPL  DBVT_IMPL_SSE
#define DBVT_MERGE_IMPL   DBVT_IMPL_SSE
#define DBVT_INT0_IMPL   DBVT_IMPL_SSE
#else
#define DBVT_SELECT_IMPL  DBVT_IMPL_GENERIC
#define DBVT_MERGE_IMPL   DBVT_IMPL_GENERIC
#define DBVT_INT0_IMPL   DBVT_IMPL_GENERIC
#endif

#if (DBVT_SELECT_IMPL==DBVT_IMPL_SSE)|| \
 (DBVT_MERGE_IMPL==DBVT_IMPL_SSE)|| \
 (DBVT_INT0_IMPL==DBVT_IMPL_SSE)
#include <emmintrin.h>
#endif

//
// Auto config and checks
//

#if DBVT_USE_TEMPLATE
#define DBVT_VIRTUAL
#define DBVT_VIRTUAL_DTOR(a)
#define      template <typename T>
#define ICollide& policy    T& policy
#define DBVT_CHECKTYPE    static const ICollide& typechecker=*(T*)1;(void)typechecker;
#else
#define DBVT_VIRTUAL_DTOR(a)  virtual ~a() {}
#define DBVT_VIRTUAL    virtual
#define 
#define ICollide& policy    ICollide& policy
#define DBVT_CHECKTYPE
#endif

#if DBVT_USE_MEMMOVE
#if !defined( __CELLOS_LV2__) && !defined(__MWERKS__)
#include <memory.h>
#endif
#include <string.h>
#endif

#ifndef DBVT_USE_TEMPLATE
#error "DBVT_USE_TEMPLATE undefined"
#endif

#ifndef DBVT_USE_MEMMOVE
#error "DBVT_USE_MEMMOVE undefined"
#endif

#ifndef DBVT_ENABLE_BENCHMARK
#error "DBVT_ENABLE_BENCHMARK undefined"
#endif

#ifndef DBVT_SELECT_IMPL
#error "DBVT_SELECT_IMPL undefined"
#endif

#ifndef DBVT_MERGE_IMPL
#error "DBVT_MERGE_IMPL undefined"
#endif

#ifndef DBVT_INT0_IMPL
#error "DBVT_INT0_IMPL undefined"
#endif

//
// Defaults volumes
//

/* btDbvtAabbMm   */ 
class btDbvtAabbMm
{ public: 
 DBVT_INLINE btVector3   Center() const { return((mi+mx)/2); }
 DBVT_INLINE btVector3   Lengths() const { return(mx-mi); }
 DBVT_INLINE btVector3   Extents() const { return((mx-mi)/2); }
 DBVT_INLINE const btVector3& Mins() const { return(mi); }
 DBVT_INLINE const btVector3& Maxs() const { return(mx); }
 static inline btDbvtAabbMm  FromCE(const btVector3& c,const btVector3& e);
 static inline btDbvtAabbMm  FromCR(const btVector3& c,btScalar r);
 static inline btDbvtAabbMm  FromMM(const btVector3& mi,const btVector3& mx);
 static inline btDbvtAabbMm  FromPoints(const btVector3* pts,int n);
 static inline btDbvtAabbMm  FromPoints(const btVector3** ppts,int n);
 DBVT_INLINE void    Expand(const btVector3& e);
 DBVT_INLINE void    SignedExpand(const btVector3& e);
 DBVT_INLINE bool    Contain(const btDbvtAabbMm& a) const;
 DBVT_INLINE int     Classify(const btVector3& n,btScalar o,int s) const;
 DBVT_INLINE btScalar   ProjectMinimum(const btVector3& v,unsigned signs) const;
 DBVT_INLINE friend bool   Intersect( const btDbvtAabbMm& a,
  const btDbvtAabbMm& b);
 
 DBVT_INLINE friend bool   Intersect( const btDbvtAabbMm& a,
  const btVector3& b);

 DBVT_INLINE friend btScalar  Proximity( const btDbvtAabbMm& a,
  const btDbvtAabbMm& b);
 DBVT_INLINE friend int   Select(  const btDbvtAabbMm& o,
  const btDbvtAabbMm& a,
  const btDbvtAabbMm& b);
 DBVT_INLINE friend void   Merge(  const btDbvtAabbMm& a,
  const btDbvtAabbMm& b,
  btDbvtAabbMm& r);
 DBVT_INLINE friend bool   NotEqual( const btDbvtAabbMm& a,
  const btDbvtAabbMm& b);
private:
 DBVT_INLINE void    AddSpan(const btVector3& d,btScalar& smi,btScalar& smx) const;
private:
 btVector3 mi,mx;
};

// Types 
typedef btDbvtAabbMm btDbvtVolume;

/* btDbvtNode    */ 
class btDbvtNode
{ public: 
 btDbvtVolume volume;
 btDbvtNode*  parent;
 DBVT_INLINE bool isleaf() const  { return(childs[1]==0); }
 DBVT_INLINE bool isinternal() const { return(!isleaf()); }
 union
 {
  btDbvtNode* childs[2];
  void* data;
  int  dataAsInt;
 };
};

///The btDbvt class implements a fast dynamic bounding volume tree based on axis aligned bounding boxes (aabb tree).
///This btDbvt is used for soft body collision detection and for the btDbvtBroadphase. It has a fast insert, remove and update of nodes.
///Unlike the btQuantizedBvh, nodes can be dynamically moved around, which allows for change in topology of the underlying data structure.
class btDbvt
{ public: 
 /* Stack element */ 
 struct sStkNN
 {
  const btDbvtNode* a;
  const btDbvtNode* b;
  sStkNN() {}
  sStkNN(const btDbvtNode* na,const btDbvtNode* nb) : a(na),b(nb) {}
 };
 struct sStkNP
 {
  const btDbvtNode* node;
  int   mask;
  sStkNP(const btDbvtNode* n,unsigned m) : node(n),mask(m) {}
 };
 struct btDbvt::sStkNPS
 {
  const btDbvtNode* node;
  int   mask;
  btScalar value;
  btDbvt::sStkNPS() {}
  btDbvt::sStkNPS(const btDbvtNode* n,unsigned m,btScalar v) : node(n),mask(m),value(v) {}
 };
 struct sStkCLN
 {
  const btDbvtNode* node;
  btDbvtNode*  parent;
  sStkCLN(const btDbvtNode* n,btDbvtNode* p) : node(n),parent(p) {}
 };
 // Policies/Interfaces

 /* ICollide */ 
 struct ICollide
 {  
  DBVT_VIRTUAL_DTOR(ICollide)
   DBVT_VIRTUAL void Process(const btDbvtNode*,const btDbvtNode*)  {}
  DBVT_VIRTUAL void Process(const btDbvtNode*)     {}
  DBVT_VIRTUAL void Process(const btDbvtNode* n,btScalar)   { Process(n); }
  DBVT_VIRTUAL bool Descent(const btDbvtNode*)     { return(true); }
  DBVT_VIRTUAL bool AllLeaves(const btDbvtNode*)     { return(true); }
 };
 /* IWriter */ 
 struct IWriter
 {
  virtual ~IWriter() {}
  virtual void  Prepare(const btDbvtNode* root,int numnodes)=0;
  virtual void  WriteNode(const btDbvtNode*,int index,int parent,int child0,int child1)=0;
  virtual void  WriteLeaf(const btDbvtNode*,int index,int parent)=0;
 };
 /* IClone */ 
 struct IClone
 {
  virtual ~IClone() {}
  virtual void  CloneLeaf(btDbvtNode*) {}
 };

 // Constants
 enum {
  SIMPLE_STACKSIZE = 64,
  DOUBLE_STACKSIZE = SIMPLE_STACKSIZE*2
 };

 // Fields
 btDbvtNode*  m_root;
 btDbvtNode*  m_free;
 int    m_lkhd;
 int    m_leaves;
 unsigned  m_opath;

 
 btAlignedObjectArray<sStkNN> m_stkStack;


 // Methods
 btDbvt();
 ~btDbvt();
 void   clear();
 bool   empty() const { return(0==m_root); }
 void   optimizeBottomUp();
 void   optimizeTopDown(int bu_treshold=128);
 void   optimizeIncremental(int passes);
 btDbvtNode*  insert(const btDbvtVolume& box,void* data);
 void   update(btDbvtNode* leaf,int lookahead=-1);
 void   update(btDbvtNode* leaf,btDbvtVolume& volume);
 bool   update(btDbvtNode* leaf,btDbvtVolume& volume,const btVector3& velocity,btScalar margin);
 bool   update(btDbvtNode* leaf,btDbvtVolume& volume,const btVector3& velocity);
 bool   update(btDbvtNode* leaf,btDbvtVolume& volume,btScalar margin); 
 void   remove(btDbvtNode* leaf);
 void   write(btDbvt::IWriter* iwriter) const;
 void   clone(btDbvt& dest,btDbvt::IClone* iclone=0) const;
 static int  maxdepth(const btDbvtNode* node);
 static int  countLeaves(const btDbvtNode* node);
 static void  extractLeaves(const btDbvtNode* node,btAlignedObjectArray<const btDbvtNode*>& leaves);
#if DBVT_ENABLE_BENCHMARK
 static void  benchmark();
#else
 static void  benchmark(){}
#endif
 // ICollide& policy must support ICollide policy/interface
 
  static void  enumNodes( const btDbvtNode* root,
  ICollide& policy);
 
  static void  enumLeaves( const btDbvtNode* root,
  ICollide& policy);
 
  void  collideTT( const btDbvtNode* root0,
  const btDbvtNode* root1,
  ICollide& policy);

 
  void  collideTTpersistentStack( const btDbvtNode* root0,
    const btDbvtNode* root1,
    ICollide& policy);


 
  void  collideTV( const btDbvtNode* root,
  const btDbvtVolume& volume,
  ICollide& policy);
 ///rayTest is a re-entrant ray test, and can be called in parallel as long as the btAlignedAlloc is thread-safe (uses locking etc)
 ///rayTest is slower than rayTestInternal, because it builds a local stack, using memory allocations, and it recomputes signs/rayDirectionInverses each time
 
  static void  rayTest( const btDbvtNode* root,
  const btVector3& rayFrom,
  const btVector3& rayTo,
  ICollide& policy);
 ///rayTestInternal is faster than rayTest, because it uses a persistent stack (to reduce dynamic memory allocations to a minimum) and it uses precomputed signs/rayInverseDirections
 ///rayTestInternal is used by btDbvtBroadphase to accelerate world ray casts
 
  void  rayTestInternal( const btDbvtNode* root,
        const btVector3& rayFrom,
        const btVector3& rayTo,
        const btVector3& rayDirectionInverse,
        unsigned int *signs,
        btScalar lambda_max,
        const btVector3& aabbMin,
        const btVector3& aabbMax,
        ICollide& policy) const;

 
  static void  collideKDOP(const btDbvtNode* root,
  const btVector3* normals,
  const btScalar* offsets,
  int count,
  ICollide& policy);
 
  static void  collideOCL( const btDbvtNode* root,
  const btVector3* normals,
  const btScalar* offsets,
  const btVector3& sortaxis,
  int count,        
  ICollide& policy,
  bool fullsort=true);
 
  static void  collideTU( const btDbvtNode* root,
  ICollide& policy);
 // Helpers 
 static DBVT_INLINE int nearest(const int* i,const btDbvt::btDbvt::sStkNPS* a,btScalar v,int l,int h)
 {
  int m=0;
  while(l<h)
  {
   m=(l+h)>>1;
   if(a[i[m]].value>=v) l=m+1; else h=m;
  }
  return(h);
 }
 static DBVT_INLINE int allocate( btAlignedObjectArray<int>& ifree,
  btAlignedObjectArray<btDbvt::sStkNPS>& stock,
  const btDbvt::sStkNPS& value)
 {
  int i;
  if(ifree.size()>0)
  { i=ifree[ifree.size()-1];ifree.pop_back();stock[i]=value; }
  else
  { i=stock.size();stock.push_back(value); }
  return(i); 
 }
 //
private:
 btDbvt(const btDbvt&) {} 
};

//
// Inline's
//

//
inline btDbvtAabbMm   btDbvtAabbMm::FromCE(const btVector3& c,const btVector3& e)
{
 btDbvtAabbMm box;
 box.mi=c-e;box.mx=c+e;
 return(box);
}

//
inline btDbvtAabbMm   btDbvtAabbMm::FromCR(const btVector3& c,btScalar r)
{
 return(FromCE(c,btVector3(r,r,r)));
}

//
inline btDbvtAabbMm   btDbvtAabbMm::FromMM(const btVector3& mi,const btVector3& mx)
{
 btDbvtAabbMm box;
 box.mi=mi;box.mx=mx;
 return(box);
}

//
inline btDbvtAabbMm   btDbvtAabbMm::FromPoints(const btVector3* pts,int n)
{
 btDbvtAabbMm box;
 box.mi=box.mx=pts[0];
 for(int i=1;i<n;++i)
 {
  box.mi.setMin(pts[i]);
  box.mx.setMax(pts[i]);
 }
 return(box);
}

//
inline btDbvtAabbMm   btDbvtAabbMm::FromPoints(const btVector3** ppts,int n)
{
 btDbvtAabbMm box;
 box.mi=box.mx=*ppts[0];
 for(int i=1;i<n;++i)
 {
  box.mi.setMin(*ppts[i]);
  box.mx.setMax(*ppts[i]);
 }
 return(box);
}

//
DBVT_INLINE void  btDbvtAabbMm::Expand(const btVector3& e)
{
 mi-=e;mx+=e;
}

//
DBVT_INLINE void  btDbvtAabbMm::SignedExpand(const btVector3& e)
{
 if(e.x()>0) mx.setX(mx.x()+e[0]); else mi.setX(mi.x()+e[0]);
 if(e.y()>0) mx.setY(mx.y()+e[1]); else mi.setY(mi.y()+e[1]);
 if(e.z()>0) mx.setZ(mx.z()+e[2]); else mi.setZ(mi.z()+e[2]);
}

//
DBVT_INLINE bool  btDbvtAabbMm::Contain(const btDbvtAabbMm& a) const
{
 return( (mi.x()<=a.mi.x())&&
  (mi.y()<=a.mi.y())&&
  (mi.z()<=a.mi.z())&&
  (mx.x()>=a.mx.x())&&
  (mx.y()>=a.mx.y())&&
  (mx.z()>=a.mx.z()));
}

//
DBVT_INLINE int  btDbvtAabbMm::Classify(const btVector3& n,btScalar o,int s) const
{
 btVector3   pi,px;
 switch(s)
 {
 case (0+0+0): px=btVector3(mi.x(),mi.y(),mi.z());
  pi=btVector3(mx.x(),mx.y(),mx.z());break;
 case (1+0+0): px=btVector3(mx.x(),mi.y(),mi.z());
  pi=btVector3(mi.x(),mx.y(),mx.z());break;
 case (0+2+0): px=btVector3(mi.x(),mx.y(),mi.z());
  pi=btVector3(mx.x(),mi.y(),mx.z());break;
 case (1+2+0): px=btVector3(mx.x(),mx.y(),mi.z());
  pi=btVector3(mi.x(),mi.y(),mx.z());break;
 case (0+0+4): px=btVector3(mi.x(),mi.y(),mx.z());
  pi=btVector3(mx.x(),mx.y(),mi.z());break;
 case (1+0+4): px=btVector3(mx.x(),mi.y(),mx.z());
  pi=btVector3(mi.x(),mx.y(),mi.z());break;
 case (0+2+4): px=btVector3(mi.x(),mx.y(),mx.z());
  pi=btVector3(mx.x(),mi.y(),mi.z());break;
 case (1+2+4): px=btVector3(mx.x(),mx.y(),mx.z());
  pi=btVector3(mi.x(),mi.y(),mi.z());break;
 }
 if((btDot(n,px)+o)<0)  return(-1);
 if((btDot(n,pi)+o)>=0) return(+1);
 return(0);
}

//
DBVT_INLINE btScalar btDbvtAabbMm::ProjectMinimum(const btVector3& v,unsigned signs) const
{
 const btVector3* b[]={&mx,&mi};
 const btVector3  p( b[(signs>>0)&1]->x(),
  b[(signs>>1)&1]->y(),
  b[(signs>>2)&1]->z());
 return(btDot(p,v));
}

//
DBVT_INLINE void  btDbvtAabbMm::AddSpan(const btVector3& d,btScalar& smi,btScalar& smx) const
{
 for(int i=0;i<3;++i)
 {
  if(d[i]<0)
  { smi+=mx[i]*d[i];smx+=mi[i]*d[i]; }
  else
  { smi+=mi[i]*d[i];smx+=mx[i]*d[i]; }
 }
}

//
DBVT_INLINE bool  Intersect( const btDbvtAabbMm& a,
          const btDbvtAabbMm& b)
{
#if DBVT_INT0_IMPL == DBVT_IMPL_SSE
 const __m128 rt(_mm_or_ps( _mm_cmplt_ps(_mm_load_ps(b.mx),_mm_load_ps(a.mi)),
  _mm_cmplt_ps(_mm_load_ps(a.mx),_mm_load_ps(b.mi))));
 const __int32* pu((const __int32*)&rt);
 return((pu[0]|pu[1]|pu[2])==0);
#else
 return( (a.mi.x()<=b.mx.x())&&
  (a.mx.x()>=b.mi.x())&&
  (a.mi.y()<=b.mx.y())&&
  (a.mx.y()>=b.mi.y())&&
  (a.mi.z()<=b.mx.z())&&  
  (a.mx.z()>=b.mi.z()));
#endif
}



//
DBVT_INLINE bool  Intersect( const btDbvtAabbMm& a,
          const btVector3& b)
{
 return( (b.x()>=a.mi.x())&&
  (b.y()>=a.mi.y())&&
  (b.z()>=a.mi.z())&&
  (b.x()<=a.mx.x())&&
  (b.y()<=a.mx.y())&&
  (b.z()<=a.mx.z()));
}





//////////////////////////////////////


//
DBVT_INLINE btScalar Proximity( const btDbvtAabbMm& a,
          const btDbvtAabbMm& b)
{
 const btVector3 d=(a.mi+a.mx)-(b.mi+b.mx);
 return(btFabs(d.x())+btFabs(d.y())+btFabs(d.z()));
}



//
DBVT_INLINE int   Select( const btDbvtAabbMm& o,
          const btDbvtAabbMm& a,
          const btDbvtAabbMm& b)
{
#if DBVT_SELECT_IMPL == DBVT_IMPL_SSE
 static ATTRIBUTE_ALIGNED16(const unsigned __int32) mask[]={0x7fffffff,0x7fffffff,0x7fffffff,0x7fffffff};
 ///@todo: the intrinsic version is 11% slower
#if DBVT_USE_INTRINSIC_SSE

 union btSSEUnion ///NOTE: if we use more intrinsics, move btSSEUnion into the LinearMath directory
 {
    __m128  ssereg;
    float  floats[4];
    int   ints[4];
 };

 __m128 omi(_mm_load_ps(o.mi));
 omi=_mm_add_ps(omi,_mm_load_ps(o.mx));
 __m128 ami(_mm_load_ps(a.mi));
 ami=_mm_add_ps(ami,_mm_load_ps(a.mx));
 ami=_mm_sub_ps(ami,omi);
 ami=_mm_and_ps(ami,_mm_load_ps((const float*)mask));
 __m128 bmi(_mm_load_ps(b.mi));
 bmi=_mm_add_ps(bmi,_mm_load_ps(b.mx));
 bmi=_mm_sub_ps(bmi,omi);
 bmi=_mm_and_ps(bmi,_mm_load_ps((const float*)mask));
 __m128 t0(_mm_movehl_ps(ami,ami));
 ami=_mm_add_ps(ami,t0);
 ami=_mm_add_ss(ami,_mm_shuffle_ps(ami,ami,1));
 __m128 t1(_mm_movehl_ps(bmi,bmi));
 bmi=_mm_add_ps(bmi,t1);
 bmi=_mm_add_ss(bmi,_mm_shuffle_ps(bmi,bmi,1));
 
 btSSEUnion tmp;
 tmp.ssereg = _mm_cmple_ss(bmi,ami);
 return tmp.ints[0]&1;

#else
 ATTRIBUTE_ALIGNED16(__int32 r[1]);
 __asm
 {
  mov  eax,o
   mov  ecx,a
   mov  edx,b
   movaps xmm0,[eax]
  movaps xmm5,mask
   addps xmm0,[eax+16] 
  movaps xmm1,[ecx]
  movaps xmm2,[edx]
  addps xmm1,[ecx+16]
  addps xmm2,[edx+16]
  subps xmm1,xmm0
   subps xmm2,xmm0
   andps xmm1,xmm5
   andps xmm2,xmm5
   movhlps xmm3,xmm1
   movhlps xmm4,xmm2
   addps xmm1,xmm3
   addps xmm2,xmm4
   pshufd xmm3,xmm1,1
   pshufd xmm4,xmm2,1
   addss xmm1,xmm3
   addss xmm2,xmm4
   cmpless xmm2,xmm1
   movss r,xmm2
 }
 return(r[0]&1);
#endif
#else
 return(Proximity(o,a)<Proximity(o,b)?0:1);
#endif
}

//
DBVT_INLINE void  Merge( const btDbvtAabbMm& a,
         const btDbvtAabbMm& b,
         btDbvtAabbMm& r)
{
#if DBVT_MERGE_IMPL==DBVT_IMPL_SSE
 __m128 ami(_mm_load_ps(a.mi));
 __m128 amx(_mm_load_ps(a.mx));
 __m128 bmi(_mm_load_ps(b.mi));
 __m128 bmx(_mm_load_ps(b.mx));
 ami=_mm_min_ps(ami,bmi);
 amx=_mm_max_ps(amx,bmx);
 _mm_store_ps(r.mi,ami);
 _mm_store_ps(r.mx,amx);
#else
 for(int i=0;i<3;++i)
 {
  if(a.mi[i]<b.mi[i]) r.mi[i]=a.mi[i]; else r.mi[i]=b.mi[i];
  if(a.mx[i]>b.mx[i]) r.mx[i]=a.mx[i]; else r.mx[i]=b.mx[i];
 }
#endif
}

//
DBVT_INLINE bool  NotEqual( const btDbvtAabbMm& a,
         const btDbvtAabbMm& b)
{
 return( (a.mi.x()!=b.mi.x())||
  (a.mi.y()!=b.mi.y())||
  (a.mi.z()!=b.mi.z())||
  (a.mx.x()!=b.mx.x())||
  (a.mx.y()!=b.mx.y())||
  (a.mx.z()!=b.mx.z()));
}

//
// Inline's
//

//

inline void  btDbvt::enumNodes( const btDbvtNode* root,
          ICollide& policy)
{
 DBVT_CHECKTYPE
  policy.Process(root);
 if(root->isinternal())
 {
  enumNodes(root->childs[0],policy);
  enumNodes(root->childs[1],policy);
 }
}

//

inline void  btDbvt::enumLeaves( const btDbvtNode* root,
           ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root->isinternal())
  {
   enumLeaves(root->childs[0],policy);
   enumLeaves(root->childs[1],policy);
  }
  else
  {
   policy.Process(root);
  }
}

//

inline void  btDbvt::collideTT( const btDbvtNode* root0,
          const btDbvtNode* root1,
          ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root0&&root1)
  {
   int        depth=1;
   int        treshold=DOUBLE_STACKSIZE-4;
   btAlignedObjectArray<sStkNN> stkStack;
   stkStack.resize(DOUBLE_STACKSIZE);
   stkStack[0]=sStkNN(root0,root1);
   do {  
    sStkNN p=stkStack[--depth];
    if(depth>treshold)
    {
     stkStack.resize(stkStack.size()*2);
     treshold=stkStack.size()-4;
    }
    if(p.a==p.b)
    {
     if(p.a->isinternal())
     {
      stkStack[depth++]=sStkNN(p.a->childs[0],p.a->childs[0]);
      stkStack[depth++]=sStkNN(p.a->childs[1],p.a->childs[1]);
      stkStack[depth++]=sStkNN(p.a->childs[0],p.a->childs[1]);
     }
    }
    else if(Intersect(p.a->volume,p.b->volume))
    {
     if(p.a->isinternal())
     {
      if(p.b->isinternal())
      {
       stkStack[depth++]=sStkNN(p.a->childs[0],p.b->childs[0]);
       stkStack[depth++]=sStkNN(p.a->childs[1],p.b->childs[0]);
       stkStack[depth++]=sStkNN(p.a->childs[0],p.b->childs[1]);
       stkStack[depth++]=sStkNN(p.a->childs[1],p.b->childs[1]);
      }
      else
      {
       stkStack[depth++]=sStkNN(p.a->childs[0],p.b);
       stkStack[depth++]=sStkNN(p.a->childs[1],p.b);
      }
     }
     else
     {
      if(p.b->isinternal())
      {
       stkStack[depth++]=sStkNN(p.a,p.b->childs[0]);
       stkStack[depth++]=sStkNN(p.a,p.b->childs[1]);
      }
      else
      {
       policy.Process(p.a,p.b);
      }
     }
    }
   } while(depth);
  }
}




inline void  btDbvt::collideTTpersistentStack( const btDbvtNode* root0,
          const btDbvtNode* root1,
          ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root0&&root1)
  {
   int        depth=1;
   int        treshold=DOUBLE_STACKSIZE-4;
   
   m_stkStack.resize(DOUBLE_STACKSIZE);
   m_stkStack[0]=sStkNN(root0,root1);
   do {  
    sStkNN p=m_stkStack[--depth];
    if(depth>treshold)
    {
     m_stkStack.resize(m_stkStack.size()*2);
     treshold=m_stkStack.size()-4;
    }
    if(p.a==p.b)
    {
     if(p.a->isinternal())
     {
      m_stkStack[depth++]=sStkNN(p.a->childs[0],p.a->childs[0]);
      m_stkStack[depth++]=sStkNN(p.a->childs[1],p.a->childs[1]);
      m_stkStack[depth++]=sStkNN(p.a->childs[0],p.a->childs[1]);
     }
    }
    else if(Intersect(p.a->volume,p.b->volume))
    {
     if(p.a->isinternal())
     {
      if(p.b->isinternal())
      {
       m_stkStack[depth++]=sStkNN(p.a->childs[0],p.b->childs[0]);
       m_stkStack[depth++]=sStkNN(p.a->childs[1],p.b->childs[0]);
       m_stkStack[depth++]=sStkNN(p.a->childs[0],p.b->childs[1]);
       m_stkStack[depth++]=sStkNN(p.a->childs[1],p.b->childs[1]);
      }
      else
      {
       m_stkStack[depth++]=sStkNN(p.a->childs[0],p.b);
       m_stkStack[depth++]=sStkNN(p.a->childs[1],p.b);
      }
     }
     else
     {
      if(p.b->isinternal())
      {
       m_stkStack[depth++]=sStkNN(p.a,p.b->childs[0]);
       m_stkStack[depth++]=sStkNN(p.a,p.b->childs[1]);
      }
      else
      {
       policy.Process(p.a,p.b);
      }
     }
    }
   } while(depth);
  }
}

 

//

inline void  btDbvt::collideTV( const btDbvtNode* root,
          const btDbvtVolume& vol,
          ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root)
  {
   ATTRIBUTE_ALIGNED16(btDbvtVolume)  volume(vol);
   btAlignedObjectArray<const btDbvtNode*> stack;
   stack.resize(0);
   stack.reserve(SIMPLE_STACKSIZE);
   stack.push_back(root);
   do {
    const btDbvtNode* n=stack[stack.size()-1];
    stack.pop_back();
    if(Intersect(n->volume,volume))
    {
     if(n->isinternal())
     {
      stack.push_back(n->childs[0]);
      stack.push_back(n->childs[1]);
     }
     else
     {
      policy.Process(n);
     }
    }
   } while(stack.size()>0);
  }
}


inline void  btDbvt::rayTestInternal( const btDbvtNode* root,
        const btVector3& rayFrom,
        const btVector3& rayTo,
        const btVector3& rayDirectionInverse,
        unsigned int *signs,
        btScalar lambda_max,
        const btVector3& aabbMin,
        const btVector3& aabbMax,
        ICollide& policy) const
{
        (void) rayTo;
 DBVT_CHECKTYPE
 if(root)
 {
  btVector3 resultNormal;

  int        depth=1;
  int        treshold=DOUBLE_STACKSIZE-2;
  btAlignedObjectArray<const btDbvtNode*> stack;
  stack.resize(DOUBLE_STACKSIZE);
  stack[0]=root;
  btVector3 bounds[2];
  do 
  {
   const btDbvtNode* node=stack[--depth];
   bounds[0] = node->volume.Mins()-aabbMax;
   bounds[1] = node->volume.Maxs()-aabbMin;
   btScalar tmin=1.f,lambda_min=0.f;
   unsigned int result1=false;
   result1 = btRayAabb2(rayFrom,rayDirectionInverse,signs,bounds,tmin,lambda_min,lambda_max);
   if(result1)
   {
    if(node->isinternal())
    {
     if(depth>treshold)
     {
      stack.resize(stack.size()*2);
      treshold=stack.size()-2;
     }
     stack[depth++]=node->childs[0];
     stack[depth++]=node->childs[1];
    }
    else
    {
     policy.Process(node);
    }
   }
  } while(depth);
 }
}

//

inline void  btDbvt::rayTest( const btDbvtNode* root,
        const btVector3& rayFrom,
        const btVector3& rayTo,
        ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root)
  {
   btVector3 rayDir = (rayTo-rayFrom);
   rayDir.normalize ();

   ///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
   btVector3 rayDirectionInverse;
   rayDirectionInverse[0] = rayDir[0] == btScalar(0.0) ? btScalar(BT_LARGE_FLOAT) : btScalar(1.0) / rayDir[0];
   rayDirectionInverse[1] = rayDir[1] == btScalar(0.0) ? btScalar(BT_LARGE_FLOAT) : btScalar(1.0) / rayDir[1];
   rayDirectionInverse[2] = rayDir[2] == btScalar(0.0) ? btScalar(BT_LARGE_FLOAT) : btScalar(1.0) / rayDir[2];
   unsigned int *signs = { rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0};

   btScalar lambda_max = rayDir.dot(rayTo-rayFrom);

   btVector3 resultNormal;

   btAlignedObjectArray<const btDbvtNode*> stack;

   int        depth=1;
   int        treshold=DOUBLE_STACKSIZE-2;

   stack.resize(DOUBLE_STACKSIZE);
   stack[0]=root;
   btVector3 bounds[2];
   do {
    const btDbvtNode* node=stack[--depth];

    bounds[0] = node->volume.Mins();
    bounds[1] = node->volume.Maxs();
    
    btScalar tmin=1.f,lambda_min=0.f;
    unsigned int result1 = btRayAabb2(rayFrom,rayDirectionInverse,signs,bounds,tmin,lambda_min,lambda_max);

#ifdef COMPARE_BTRAY_AABB2
    btScalar param=1.f;
    bool result2 = btRayAabb(rayFrom,rayTo,node->volume.Mins(),node->volume.Maxs(),param,resultNormal);
    btAssert(result1 == result2);
#endif //TEST_BTRAY_AABB2

    if(result1)
    {
     if(node->isinternal())
     {
      if(depth>treshold)
      {
       stack.resize(stack.size()*2);
       treshold=stack.size()-2;
      }
      stack[depth++]=node->childs[0];
      stack[depth++]=node->childs[1];
     }
     else
     {
      policy.Process(node);
     }
    }
   } while(depth);

  }
}

//

inline void  btDbvt::collideKDOP(const btDbvtNode* root,
         const btVector3* normals,
         const btScalar* offsets,
         int count,
         ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root)
  {
   const int      inside=(1<<count)-1;
   btAlignedObjectArray<sStkNP> stack;
   int        signs[sizeof(unsigned)*8];
   btAssert(count<int (sizeof(signs)/sizeof(signs[0])));
   for(int i=0;i<count;++i)
   {
    signs[i]= ((normals[i].x()>=0)?1:0)+
     ((normals[i].y()>=0)?2:0)+
     ((normals[i].z()>=0)?4:0);
   }
   stack.reserve(SIMPLE_STACKSIZE);
   stack.push_back(sStkNP(root,0));
   do {
    sStkNP se=stack[stack.size()-1];
    bool out=false;
    stack.pop_back();
    for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
    {
     if(0==(se.mask&j))
     {
      const int side=se.node->volume.Classify(normals[i],offsets[i],signs[i]);
      switch(side)
      {
      case -1: out=true;break;
      case +1: se.mask|=j;break;
      }
     }
    }
    if(!out)
    {
     if((se.mask!=inside)&&(se.node->isinternal()))
     {
      stack.push_back(sStkNP(se.node->childs[0],se.mask));
      stack.push_back(sStkNP(se.node->childs[1],se.mask));
     }
     else
     {
      if(policy.AllLeaves(se.node)) enumLeaves(se.node,policy);
     }
    }
   } while(stack.size());
  }
}

//

inline void  btDbvt::collideOCL( const btDbvtNode* root,
           const btVector3* normals,
           const btScalar* offsets,
           const btVector3& sortaxis,
           int count,
           ICollide& policy,
           bool fsort)
{
 DBVT_CHECKTYPE
  if(root)
  {
   const unsigned     srtsgns=(sortaxis[0]>=0?1:0)+
    (sortaxis[1]>=0?2:0)+
    (sortaxis[2]>=0?4:0);
   const int      inside=(1<<count)-1;
   btAlignedObjectArray<btDbvt::sStkNPS> stock;
   btAlignedObjectArray<int>  ifree;
   btAlignedObjectArray<int>  stack;
   int        signs[sizeof(unsigned)*8];
   btAssert(count<int (sizeof(signs)/sizeof(signs[0])));
   for(int i=0;i<count;++i)
   {
    signs[i]= ((normals[i].x()>=0)?1:0)+
     ((normals[i].y()>=0)?2:0)+
     ((normals[i].z()>=0)?4:0);
   }
   stock.reserve(SIMPLE_STACKSIZE);
   stack.reserve(SIMPLE_STACKSIZE);
   ifree.reserve(SIMPLE_STACKSIZE);
   stack.push_back(allocate(ifree,stock,btDbvt::sStkNPS(root,0,root->volume.ProjectMinimum(sortaxis,srtsgns))));
   do {
    const int id=stack[stack.size()-1];
    btDbvt::sStkNPS  se=stock[id];
    stack.pop_back();ifree.push_back(id);
    if(se.mask!=inside)
    {
     bool out=false;
     for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
     {
      if(0==(se.mask&j))
      {
       const int side=se.node->volume.Classify(normals[i],offsets[i],signs[i]);
       switch(side)
       {
       case -1: out=true;break;
       case +1: se.mask|=j;break;
       }
      }
     }
     if(out) continue;
    }
    if(policy.Descent(se.node))
    {
     if(se.node->isinternal())
     {
      const btDbvtNode* pns[]={ se.node->childs[0],se.node->childs[1]};
      btDbvt::sStkNPS  nes[]={ btDbvt::sStkNPS(pns[0],se.mask,pns[0]->volume.ProjectMinimum(sortaxis,srtsgns)),
       btDbvt::sStkNPS(pns[1],se.mask,pns[1]->volume.ProjectMinimum(sortaxis,srtsgns))};
      const int q=nes[0].value<nes[1].value?1:0;    
      int   j=stack.size();
      if(fsort&&(j>0))
      {
       /* Insert 0 */ 
       j=nearest(&stack[0],&stock[0],nes[q].value,0,stack.size());
       stack.push_back(0);
#if DBVT_USE_MEMMOVE
       memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
#else
       for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
#endif
       stack[j]=allocate(ifree,stock,nes[q]);
       /* Insert 1 */ 
       j=nearest(&stack[0],&stock[0],nes[1-q].value,j,stack.size());
       stack.push_back(0);
#if DBVT_USE_MEMMOVE
       memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
#else
       for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
#endif
       stack[j]=allocate(ifree,stock,nes[1-q]);
      }
      else
      {
       stack.push_back(allocate(ifree,stock,nes[q]));
       stack.push_back(allocate(ifree,stock,nes[1-q]));
      }
     }
     else
     {
      policy.Process(se.node,se.value);
     }
    }
   } while(stack.size());
  }
}

//

inline void  btDbvt::collideTU( const btDbvtNode* root,
          ICollide& policy)
{
 DBVT_CHECKTYPE
  if(root)
  {
   btAlignedObjectArray<const btDbvtNode*> stack;
   stack.reserve(SIMPLE_STACKSIZE);
   stack.push_back(root);
   do {
    const btDbvtNode* n=stack[stack.size()-1];
    stack.pop_back();
    if(policy.Descent(n))
    {
     if(n->isinternal())
     { stack.push_back(n->childs[0]);stack.push_back(n->childs[1]); }
     else
     { policy.Process(n); }
    }
   } while(stack.size()>0);
  }
}

//
// PP Cleanup
//

#undef DBVT_USE_MEMMOVE
#undef DBVT_USE_TEMPLATE
#undef DBVT_VIRTUAL_DTOR
#undef DBVT_VIRTUAL
#undef 
#undef ICollide& policy
#undef DBVT_CHECKTYPE
#undef DBVT_IMPL_GENERIC
#undef DBVT_IMPL_SSE
#undef DBVT_USE_INTRINSIC_SSE
#undef DBVT_SELECT_IMPL
#undef DBVT_MERGE_IMPL
#undef DBVT_INT0_IMPL

#endif
//// ../src/LinearMath/btQuickprof.h

/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

// Credits: The Clock class was inspired by the Timer classes in 
// Ogre (www.ogre3d.org).



#ifndef BT_QUICK_PROF_H
#define BT_QUICK_PROF_H

//To disable built-in profiling, please comment out next line
//#define BT_NO_PROFILE 1
#ifndef BT_NO_PROFILE
#include <stdio.h>//@todo remove this, backwards compatibility
#include "btScalar.h"
#include "btAlignedAllocator.h"
#include <new>





#define USE_BT_CLOCK 1

#ifdef USE_BT_CLOCK

///The btClock is a portable basic clock that measures accurate time in seconds, use for profiling.
class btClock
{
public:
 btClock();

 btClock(const btClock& other);
 btClock& operator=(const btClock& other);

 ~btClock();

 /// Resets the initial reference time.
 void reset();

 /// Returns the time in ms since the last call to reset or since 
 /// the btClock was created.
 unsigned long int getTimeMilliseconds();

 /// Returns the time in us since the last call to reset or since 
 /// the Clock was created.
 unsigned long int getTimeMicroseconds();
private:
 struct btClockData* m_data;
};

#endif //USE_BT_CLOCK




///A node in the Profile Hierarchy Tree
class CProfileNode {

public:
 CProfileNode( const char * name, CProfileNode * parent );
 ~CProfileNode( void );

 CProfileNode * Get_Sub_Node( const char * name );

 CProfileNode * Get_Parent( void )  { return Parent; }
 CProfileNode * Get_Sibling( void )  { return Sibling; }
 CProfileNode * Get_Child( void )   { return Child; }

 void    CleanupMemory();
 void    Reset( void );
 void    Call( void );
 bool    Return( void );

 const char * Get_Name( void )    { return Name; }
 int    Get_Total_Calls( void )  { return TotalCalls; }
 float    Get_Total_Time( void )  { return TotalTime; }

protected:

 const char * Name;
 int    TotalCalls;
 float    TotalTime;
 unsigned long int   StartTime;
 int    RecursionCounter;

 CProfileNode * Parent;
 CProfileNode * Child;
 CProfileNode * Sibling;
};

///An iterator to navigate through the tree
class CProfileIterator
{
public:
 // Access all the children of the current parent
 void    First(void);
 void    Next(void);
 bool    Is_Done(void);
 bool                Is_Root(void) { return (CurrentParent->Get_Parent() == 0); }

 void    Enter_Child( int index );  // Make the given child the new parent
 void    Enter_Largest_Child( void ); // Make the largest child the new parent
 void    Enter_Parent( void );   // Make the current parent's parent the new parent

 // Access the current child
 const char * Get_Current_Name( void )   { return CurrentChild->Get_Name(); }
 int    Get_Current_Total_Calls( void ) { return CurrentChild->Get_Total_Calls(); }
 float    Get_Current_Total_Time( void ) { return CurrentChild->Get_Total_Time(); }

 // Access the current parent
 const char * Get_Current_Parent_Name( void )   { return CurrentParent->Get_Name(); }
 int    Get_Current_Parent_Total_Calls( void ) { return CurrentParent->Get_Total_Calls(); }
 float    Get_Current_Parent_Total_Time( void ) { return CurrentParent->Get_Total_Time(); }

protected:

 CProfileNode * CurrentParent;
 CProfileNode * CurrentChild;

 CProfileIterator( CProfileNode * start );
 friend class  CProfileManager;
};


///The Manager for the Profile system
class CProfileManager {
public:
 static void      Start_Profile( const char * name );
 static void      Stop_Profile( void );

 static void      CleanupMemory(void)
 {
  Root.CleanupMemory();
 }

 static void      Reset( void );
 static void      Increment_Frame_Counter( void );
 static int      Get_Frame_Count_Since_Reset( void )  { return FrameCounter; }
 static float      Get_Time_Since_Reset( void );

 static CProfileIterator * Get_Iterator( void ) 
 { 
  
  return new CProfileIterator( &Root ); 
 }
 static void      Release_Iterator( CProfileIterator * iterator ) { delete ( iterator); }

 static void dumpRecursive(CProfileIterator* profileIterator, int spacing);

 static void dumpAll();

private:
 static CProfileNode   Root;
 static CProfileNode *   CurrentNode;
 static int      FrameCounter;
 static unsigned long int     ResetTime;
};


///ProfileSampleClass is a simple way to profile a function's scope
///Use the BT_PROFILE macro at the start of scope to time
class CProfileSample {
public:
 CProfileSample( const char * name )
 { 
  CProfileManager::Start_Profile( name ); 
 }

 ~CProfileSample( void )     
 { 
  CProfileManager::Stop_Profile(); 
 }
};


#define BT_PROFILE( name )   CProfileSample __profile( name )

#else

#define BT_PROFILE( name )

#endif //#ifndef BT_NO_PROFILE



#endif //BT_QUICK_PROF_H


//// ../src/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BT_MULTI_SAP_BROADPHASE
#define BT_MULTI_SAP_BROADPHASE

#include "btBroadphaseInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btOverlappingPairCache.h"


class btBroadphaseInterface;
class btSimpleBroadphase;


typedef btAlignedObjectArray<btBroadphaseInterface*> btSapBroadphaseArray;

///The btMultiSapBroadphase is a research project, not recommended to use in production. Use btAxisSweep3 or btDbvtBroadphase instead.
///The btMultiSapBroadphase is a broadphase that contains multiple SAP broadphases.
///The user can add SAP broadphases that cover the world. A btBroadphaseProxy can be in multiple child broadphases at the same time.
///A btQuantizedBvh acceleration structures finds overlapping SAPs for each btBroadphaseProxy.
///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=328
///and http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1329
class btMultiSapBroadphase :public btBroadphaseInterface
{
 btSapBroadphaseArray m_sapBroadphases;
 
 btSimpleBroadphase*  m_simpleBroadphase;

 btOverlappingPairCache* m_overlappingPairs;

 class btQuantizedBvh*   m_optimizedAabbTree;


 bool     m_ownsPairCache;
 
 btOverlapFilterCallback* m_filterCallback;

 int   m_invalidPair;

 struct btBridgeProxy
 {
  btBroadphaseProxy*  m_childProxy;
  btBroadphaseInterface* m_childBroadphase;
 };


public:

 struct btMultiSapProxy : public btBroadphaseProxy
 {

  ///array with all the entries that this proxy belongs to
  btAlignedObjectArray<btBridgeProxy*> m_bridgeProxies;
  btVector3 m_aabbMin;
  btVector3 m_aabbMax;

  int m_shapeType;

/*  void* m_userPtr;
  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
*/
  btMultiSapProxy(const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask)
   :btBroadphaseProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask),
   m_aabbMin(aabbMin),
   m_aabbMax(aabbMax),
   m_shapeType(shapeType)
  {
   m_multiSapParentProxy =this;
  }

  
 };

protected:


 btAlignedObjectArray<btMultiSapProxy*> m_multiSapProxies;

public:

 btMultiSapBroadphase(int maxProxies = 16384,btOverlappingPairCache* pairCache=0);


 btSapBroadphaseArray& getBroadphaseArray()
 {
  return m_sapBroadphases;
 }

 const btSapBroadphaseArray& getBroadphaseArray() const
 {
  return m_sapBroadphases;
 }

 virtual ~btMultiSapBroadphase();

 virtual btBroadphaseProxy* createProxy(  const btVector3& aabbMin,  const btVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, btDispatcher* dispatcher,void* multiSapProxy);
 virtual void destroyProxy(btBroadphaseProxy* proxy,btDispatcher* dispatcher);
 virtual void setAabb(btBroadphaseProxy* proxy,const btVector3& aabbMin,const btVector3& aabbMax, btDispatcher* dispatcher);
 virtual void getAabb(btBroadphaseProxy* proxy,btVector3& aabbMin, btVector3& aabbMax ) const;

 virtual void rayTest(const btVector3& rayFrom,const btVector3& rayTo, btBroadphaseRayCallback& rayCallback,const btVector3& aabbMin,const btVector3& aabbMax);

 void addToChildBroadphase(btMultiSapProxy* parentMultiSapProxy, btBroadphaseProxy* childProxy, btBroadphaseInterface* childBroadphase);

 ///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
 virtual void calculateOverlappingPairs(btDispatcher* dispatcher);

 bool testAabbOverlap(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1);

 virtual btOverlappingPairCache* getOverlappingPairCache()
 {
  return m_overlappingPairs;
 }
 virtual const btOverlappingPairCache* getOverlappingPairCache() const
 {
  return m_overlappingPairs;
 }

 ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
 ///will add some transform later
 virtual void getBroadphaseAabb(btVector3& aabbMin,btVector3& aabbMax) const
 {
  aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
  aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
 }

 void buildTree(const btVector3& bvhAabbMin,const btVector3& bvhAabbMax);

 virtual void printStats();

 void quicksort (btBroadphasePairArray& a, int lo, int hi);

 ///reset broadphase internal structures, to ensure determinism/reproducability
 virtual void resetPool(btDispatcher* dispatcher);

};

#endif //BT_MULTI_SAP_BROADPHASE
//// ../src/LinearMath/btDefaultMotionState.h
#ifndef BT_DEFAULT_MOTION_STATE_H
#define BT_DEFAULT_MOTION_STATE_H

#include "btMotionState.h"

///The btDefaultMotionState provides a common implementation to synchronize world transforms with offsets.
class btDefaultMotionState : public btMotionState
{ public: 
 btTransform m_graphicsWorldTrans;
 btTransform m_centerOfMassOffset;
 btTransform m_startWorldTrans;
 void*  m_userPointer;

 btDefaultMotionState(const btTransform& startTrans = btTransform::getIdentity(),const btTransform& centerOfMassOffset = btTransform::getIdentity())
  : m_graphicsWorldTrans(startTrans),
  m_centerOfMassOffset(centerOfMassOffset),
  m_startWorldTrans(startTrans),
  m_userPointer(0)

 {
 }

 ///synchronizes world transform from user to physics
 virtual void getWorldTransform(btTransform& centerOfMassWorldTrans ) const 
 {
   centerOfMassWorldTrans =  m_centerOfMassOffset.inverse() * m_graphicsWorldTrans ;
 }

 ///synchronizes world transform from physics to user
 ///Bullet only calls the update of worldtransform for active objects
 virtual void setWorldTransform(const btTransform& centerOfMassWorldTrans)
 {
   m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset ;
 }

 

};

#endif //BT_DEFAULT_MOTION_STATE_H
//// ../src/LinearMath/btIDebugDraw.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_IDEBUG_DRAW__H
#define BT_IDEBUG_DRAW__H

#include "btVector3.h"
#include "btTransform.h"


///The btIDebugDraw interface class allows hooking up a debug renderer to visually debug simulations.
///Typical use case: create a debug drawer object, and assign it to a btCollisionWorld or btDynamicsWorld using setDebugDrawer and call debugDrawWorld.
///A class that implements the btIDebugDraw interface has to implement the drawLine method at a minimum.
///For color arguments the X,Y,Z components refer to Red, Green and Blue each in the range [0..1]
class btIDebugDraw
{
 public:

 enum DebugDrawModes
 {
  DBG_NoDebug=0,
  DBG_DrawWireframe = 1,
  DBG_DrawAabb=2,
  DBG_DrawFeaturesText=4,
  DBG_DrawContactPoints=8,
  DBG_NoDeactivation=16,
  DBG_NoHelpText = 32,
  DBG_DrawText=64,
  DBG_ProfileTimings = 128,
  DBG_EnableSatComparison = 256,
  DBG_DisableBulletLCP = 512,
  DBG_EnableCCD = 1024,
  DBG_DrawConstraints = (1 << 11),
  DBG_DrawConstraintLimits = (1 << 12),
  DBG_FastWireframe = (1<<13),
  DBG_MAX_DEBUG_DRAW_MODE
 };

 virtual ~btIDebugDraw() {};

 virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& color)=0;
  
 virtual void    drawLine(const btVector3& from,const btVector3& to, const btVector3& fromColor, const btVector3& toColor)
 {
        (void) toColor;
  drawLine (from, to, fromColor);
 }

 virtual void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color)
 {
  btVector3 start = transform.getOrigin();

  const btVector3 xoffs = transform.getBasis() * btVector3(radius,0,0);
  const btVector3 yoffs = transform.getBasis() * btVector3(0,radius,0);
  const btVector3 zoffs = transform.getBasis() * btVector3(0,0,radius);

  // XY 
  drawLine(start-xoffs, start+yoffs, color);
  drawLine(start+yoffs, start+xoffs, color);
  drawLine(start+xoffs, start-yoffs, color);
  drawLine(start-yoffs, start-xoffs, color);

  // XZ
  drawLine(start-xoffs, start+zoffs, color);
  drawLine(start+zoffs, start+xoffs, color);
  drawLine(start+xoffs, start-zoffs, color);
  drawLine(start-zoffs, start-xoffs, color);

  // YZ
  drawLine(start-yoffs, start+zoffs, color);
  drawLine(start+zoffs, start+yoffs, color);
  drawLine(start+yoffs, start-zoffs, color);
  drawLine(start-zoffs, start-yoffs, color);
 }
 
 virtual void drawSphere (const btVector3& p, btScalar radius, const btVector3& color)
 {
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(p);
  drawSphere(radius,tr,color);
 }
 
 virtual void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& /*n0*/,const btVector3& /*n1*/,const btVector3& /*n2*/,const btVector3& color, btScalar alpha)
 {
  drawTriangle(v0,v1,v2,color,alpha);
 }
 virtual void drawTriangle(const btVector3& v0,const btVector3& v1,const btVector3& v2,const btVector3& color, btScalar /*alpha*/)
 {
  drawLine(v0,v1,color);
  drawLine(v1,v2,color);
  drawLine(v2,v0,color);
 }

 virtual void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)=0;

 virtual void reportErrorWarning(const char* warningString) = 0;

 virtual void draw3dText(const btVector3& location,const char* textString) = 0;
 
 virtual void setDebugMode(int debugMode) =0;
 
 virtual int  getDebugMode() const = 0;

 virtual void drawAabb(const btVector3& from,const btVector3& to,const btVector3& color)
 {

  btVector3 halfExtents = (to-from)* 0.5f;
  btVector3 center = (to+from) *0.5f;
  int i,j;

  btVector3 edgecoord(1.f,1.f,1.f),pa,pb;
  for (i=0;i<4;i++)
  {
   for (j=0;j<3;j++)
   {
    pa = btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],  
     edgecoord[2]*halfExtents[2]);
    pa+=center;

    int othercoord = j%3;
    edgecoord[othercoord]*=-1.f;
    pb = btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1], 
     edgecoord[2]*halfExtents[2]);
    pb+=center;

    drawLine(pa,pb,color);
   }
   edgecoord = btVector3(-1.f,-1.f,-1.f);
   if (i<3)
    edgecoord[i]*=-1.f;
  }
 }
 virtual void drawTransform(const btTransform& transform, btScalar orthoLen)
 {
  btVector3 start = transform.getOrigin();
  drawLine(start, start+transform.getBasis() * btVector3(orthoLen, 0, 0), btVector3(0.7f,0,0));
  drawLine(start, start+transform.getBasis() * btVector3(0, orthoLen, 0), btVector3(0,0.7f,0));
  drawLine(start, start+transform.getBasis() * btVector3(0, 0, orthoLen), btVector3(0,0,0.7f));
 }

 virtual void drawArc(const btVector3& center, const btVector3& normal, const btVector3& axis, btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle, 
    const btVector3& color, bool drawSect, btScalar stepDegrees = btScalar(10.f))
 {
  const btVector3& vx = axis;
  btVector3 vy = normal.cross(axis);
  btScalar step = stepDegrees * SIMD_RADS_PER_DEG;
  int nSteps = (int)((maxAngle - minAngle) / step);
  if(!nSteps) nSteps = 1;
  btVector3 prev = center + radiusA * vx * btCos(minAngle) + radiusB * vy * btSin(minAngle);
  if(drawSect)
  {
   drawLine(center, prev, color);
  }
  for(int i = 1; i <= nSteps; i++)
  {
   btScalar angle = minAngle + (maxAngle - minAngle) * btScalar(i) / btScalar(nSteps);
   btVector3 next = center + radiusA * vx * btCos(angle) + radiusB * vy * btSin(angle);
   drawLine(prev, next, color);
   prev = next;
  }
  if(drawSect)
  {
   drawLine(center, prev, color);
  }
 }
 virtual void drawSpherePatch(const btVector3& center, const btVector3& up, const btVector3& axis, btScalar radius, 
  btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3& color, btScalar stepDegrees = btScalar(10.f))
 {
  btVector3 vA[74];
  btVector3 vB[74];
  btVector3 *pvA = vA, *pvB = vB, *pT;
  btVector3 npole = center + up * radius;
  btVector3 spole = center - up * radius;
  btVector3 arcStart;
  btScalar step = stepDegrees * SIMD_RADS_PER_DEG;
  const btVector3& kv = up;
  const btVector3& iv = axis;
  btVector3 jv = kv.cross(iv);
  bool drawN = false;
  bool drawS = false;
  if(minTh <= -SIMD_HALF_PI)
  {
   minTh = -SIMD_HALF_PI + step;
   drawN = true;
  }
  if(maxTh >= SIMD_HALF_PI)
  {
   maxTh = SIMD_HALF_PI - step;
   drawS = true;
  }
  if(minTh > maxTh)
  {
   minTh = -SIMD_HALF_PI + step;
   maxTh =  SIMD_HALF_PI - step;
   drawN = drawS = true;
  }
  int n_hor = (int)((maxTh - minTh) / step) + 1;
  if(n_hor < 2) n_hor = 2;
  btScalar step_h = (maxTh - minTh) / btScalar(n_hor - 1);
  bool isClosed = false;
  if(minPs > maxPs)
  {
   minPs = -SIMD_PI + step;
   maxPs =  SIMD_PI;
   isClosed = true;
  }
  else if((maxPs - minPs) >= SIMD_PI * btScalar(2.f))
  {
   isClosed = true;
  }
  else
  {
   isClosed = false;
  }
  int n_vert = (int)((maxPs - minPs) / step) + 1;
  if(n_vert < 2) n_vert = 2;
  btScalar step_v = (maxPs - minPs) / btScalar(n_vert - 1);
  for(int i = 0; i < n_hor; i++)
  {
   btScalar th = minTh + btScalar(i) * step_h;
   btScalar sth = radius * btSin(th);
   btScalar cth = radius * btCos(th);
   for(int j = 0; j < n_vert; j++)
   {
    btScalar psi = minPs + btScalar(j) * step_v;
    btScalar sps = btSin(psi);
    btScalar cps = btCos(psi);
    pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
    if(i)
    {
     drawLine(pvA[j], pvB[j], color);
    }
    else if(drawS)
    {
     drawLine(spole, pvB[j], color);
    }
    if(j)
    {
     drawLine(pvB[j-1], pvB[j], color);
    }
    else
    {
     arcStart = pvB[j];
    }
    if((i == (n_hor - 1)) && drawN)
    {
     drawLine(npole, pvB[j], color);
    }
    if(isClosed)
    {
     if(j == (n_vert-1))
     {
      drawLine(arcStart, pvB[j], color);
     }
    }
    else
    {
     if(((!i) || (i == (n_hor-1))) && ((!j) || (j == (n_vert-1))))
     {
      drawLine(center, pvB[j], color);
     }
    }
   }
   pT = pvA; pvA = pvB; pvB = pT;
  }
 }
 
 virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btVector3& color)
 {
  drawLine(btVector3(bbMin[0], bbMin[1], bbMin[2]), btVector3(bbMax[0], bbMin[1], bbMin[2]), color);
  drawLine(btVector3(bbMax[0], bbMin[1], bbMin[2]), btVector3(bbMax[0], bbMax[1], bbMin[2]), color);
  drawLine(btVector3(bbMax[0], bbMax[1], bbMin[2]), btVector3(bbMin[0], bbMax[1], bbMin[2]), color);
  drawLine(btVector3(bbMin[0], bbMax[1], bbMin[2]), btVector3(bbMin[0], bbMin[1], bbMin[2]), color);
  drawLine(btVector3(bbMin[0], bbMin[1], bbMin[2]), btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
  drawLine(btVector3(bbMax[0], bbMin[1], bbMin[2]), btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(btVector3(bbMax[0], bbMax[1], bbMin[2]), btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3(bbMin[0], bbMax[1], bbMin[2]), btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3(bbMin[0], bbMin[1], bbMax[2]), btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(btVector3(bbMax[0], bbMin[1], bbMax[2]), btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3(bbMax[0], bbMax[1], bbMax[2]), btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(btVector3(bbMin[0], bbMax[1], bbMax[2]), btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
 }
 virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btTransform& trans, const btVector3& color)
 {
  drawLine(trans * btVector3(bbMin[0], bbMin[1], bbMin[2]), trans * btVector3(bbMax[0], bbMin[1], bbMin[2]), color);
  drawLine(trans * btVector3(bbMax[0], bbMin[1], bbMin[2]), trans * btVector3(bbMax[0], bbMax[1], bbMin[2]), color);
  drawLine(trans * btVector3(bbMax[0], bbMax[1], bbMin[2]), trans * btVector3(bbMin[0], bbMax[1], bbMin[2]), color);
  drawLine(trans * btVector3(bbMin[0], bbMax[1], bbMin[2]), trans * btVector3(bbMin[0], bbMin[1], bbMin[2]), color);
  drawLine(trans * btVector3(bbMin[0], bbMin[1], bbMin[2]), trans * btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMax[0], bbMin[1], bbMin[2]), trans * btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMax[0], bbMax[1], bbMin[2]), trans * btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMin[0], bbMax[1], bbMin[2]), trans * btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMin[0], bbMin[1], bbMax[2]), trans * btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMax[0], bbMin[1], bbMax[2]), trans * btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMax[0], bbMax[1], bbMax[2]), trans * btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
  drawLine(trans * btVector3(bbMin[0], bbMax[1], bbMax[2]), trans * btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
 }

 virtual void drawCapsule(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color)
 {
  btVector3 capStart(0.f,0.f,0.f);
  capStart[upAxis] = -halfHeight;

  btVector3 capEnd(0.f,0.f,0.f);
  capEnd[upAxis] = halfHeight;

  // Draw the ends
  {

   btTransform childTransform = transform;
   childTransform.getOrigin() = transform * capStart;
   drawSphere(radius, childTransform, color);
  }

  {
   btTransform childTransform = transform;
   childTransform.getOrigin() = transform * capEnd;
   drawSphere(radius, childTransform, color);
  }

  // Draw some additional lines
  btVector3 start = transform.getOrigin();

  capStart[(upAxis+1)%3] = radius;
  capEnd[(upAxis+1)%3] = radius;
  drawLine(start+transform.getBasis() * capStart,start+transform.getBasis() * capEnd, color);
  capStart[(upAxis+1)%3] = -radius;
  capEnd[(upAxis+1)%3] = -radius;
  drawLine(start+transform.getBasis() * capStart,start+transform.getBasis() * capEnd, color);

  capStart[(upAxis+1)%3] = 0.f;
  capEnd[(upAxis+1)%3] = 0.f;

  capStart[(upAxis+2)%3] = radius;
  capEnd[(upAxis+2)%3] = radius;
  drawLine(start+transform.getBasis() * capStart,start+transform.getBasis() * capEnd, color);
  capStart[(upAxis+2)%3] = -radius;
  capEnd[(upAxis+2)%3] = -radius;
  drawLine(start+transform.getBasis() * capStart,start+transform.getBasis() * capEnd, color);
 }

 virtual void drawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color)
 {
  btVector3 start = transform.getOrigin();
  btVector3 offsetHeight(0,0,0);
  offsetHeight[upAxis] = halfHeight;
  btVector3 offsetRadius(0,0,0);
  offsetRadius[(upAxis+1)%3] = radius;
  drawLine(start+transform.getBasis() * (offsetHeight+offsetRadius),start+transform.getBasis() * (-offsetHeight+offsetRadius),color);
  drawLine(start+transform.getBasis() * (offsetHeight-offsetRadius),start+transform.getBasis() * (-offsetHeight-offsetRadius),color);

  // Drawing top and bottom caps of the cylinder
  btVector3 yaxis(0,0,0);
  yaxis[upAxis] = btScalar(1.0);
  btVector3 xaxis(0,0,0);
  xaxis[(upAxis+1)%3] = btScalar(1.0);
  drawArc(start-transform.getBasis()*(offsetHeight),transform.getBasis()*yaxis,transform.getBasis()*xaxis,radius,radius,0,SIMD_2_PI,color,false,btScalar(10.0));
  drawArc(start+transform.getBasis()*(offsetHeight),transform.getBasis()*yaxis,transform.getBasis()*xaxis,radius,radius,0,SIMD_2_PI,color,false,btScalar(10.0));
 }

 virtual void drawCone(btScalar radius, btScalar height, int upAxis, const btTransform& transform, const btVector3& color)
 {

  btVector3 start = transform.getOrigin();

  btVector3 offsetHeight(0,0,0);
  offsetHeight[upAxis] = height * btScalar(0.5);
  btVector3 offsetRadius(0,0,0);
  offsetRadius[(upAxis+1)%3] = radius;
  btVector3 offset2Radius(0,0,0);
  offset2Radius[(upAxis+2)%3] = radius;

  drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight+offsetRadius),color);
  drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight-offsetRadius),color);
  drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight+offset2Radius),color);
  drawLine(start+transform.getBasis() * (offsetHeight),start+transform.getBasis() * (-offsetHeight-offset2Radius),color);

  // Drawing the base of the cone
  btVector3 yaxis(0,0,0);
  yaxis[upAxis] = btScalar(1.0);
  btVector3 xaxis(0,0,0);
  xaxis[(upAxis+1)%3] = btScalar(1.0);
  drawArc(start-transform.getBasis()*(offsetHeight),transform.getBasis()*yaxis,transform.getBasis()*xaxis,radius,radius,0,SIMD_2_PI,color,false,10.0);
 }

 virtual void drawPlane(const btVector3& planeNormal, btScalar planeConst, const btTransform& transform, const btVector3& color)
 {
  btVector3 planeOrigin = planeNormal * planeConst;
  btVector3 vec0,vec1;
  btPlaneSpace1(planeNormal,vec0,vec1);
  btScalar vecLen = 100.f;
  btVector3 pt0 = planeOrigin + vec0*vecLen;
  btVector3 pt1 = planeOrigin - vec0*vecLen;
  btVector3 pt2 = planeOrigin + vec1*vecLen;
  btVector3 pt3 = planeOrigin - vec1*vecLen;
  drawLine(transform*pt0,transform*pt1,color);
  drawLine(transform*pt2,transform*pt3,color);
 }
};


#endif //BT_IDEBUG_DRAW__H

//// ../src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_DISCRETE_DYNAMICS_WORLD_H
#define BT_DISCRETE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;
class btSimulationIslandManager;
class btTypedConstraint;
class btActionInterface;

class btIDebugDraw;
#include "LinearMath/btAlignedObjectArray.h"


///btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
class btDiscreteDynamicsWorld : public btDynamicsWorld
{
protected:

 btConstraintSolver* m_constraintSolver;

 btSimulationIslandManager* m_islandManager;

 btAlignedObjectArray<btTypedConstraint*> m_constraints;

 btAlignedObjectArray<btRigidBody*> m_nonStaticRigidBodies;

 btVector3 m_gravity;

 //for variable timesteps
 btScalar m_localTime;
 //for variable timesteps

 bool m_ownsIslandManager;
 bool m_ownsConstraintSolver;
 bool m_synchronizeAllMotionStates;

 btAlignedObjectArray<btActionInterface*> m_actions;
 
 int m_profileTimings;

 virtual void predictUnconstraintMotion(btScalar timeStep);
 
 virtual void integrateTransforms(btScalar timeStep);
  
 virtual void addSpeculativeContacts(btScalar timeStep);

 virtual void calculateSimulationIslands();

 virtual void solveConstraints(btContactSolverInfo& solverInfo);
 
 void updateActivationState(btScalar timeStep);

 void updateActions(btScalar timeStep);

 void startProfiling(btScalar timeStep);

 virtual void internalSingleStepSimulation( btScalar timeStep);


 virtual void saveKinematicState(btScalar timeStep);

 void serializeRigidBodies(btSerializer* serializer);

public:


 ///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
 btDiscreteDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration);

 virtual ~btDiscreteDynamicsWorld();

 ///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
 virtual int stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));


 virtual void synchronizeMotionStates();

 ///this can be useful to synchronize a single rigid body -> graphics object
 void synchronizeSingleMotionState(btRigidBody* body);

 virtual void addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false);

 virtual void removeConstraint(btTypedConstraint* constraint);

 virtual void addAction(btActionInterface*);

 virtual void removeAction(btActionInterface*);
 
 btSimulationIslandManager* getSimulationIslandManager()
 {
  return m_islandManager;
 }

 const btSimulationIslandManager* getSimulationIslandManager() const 
 {
  return m_islandManager;
 }

 btCollisionWorld* getCollisionWorld()
 {
  return this;
 }

 virtual void setGravity(const btVector3& gravity);

 virtual btVector3 getGravity () const;

 virtual void addCollisionObject(btCollisionObject* collisionObject,short int collisionFilterGroup=btBroadphaseProxy::StaticFilter,short int collisionFilterMask=btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

 virtual void addRigidBody(btRigidBody* body);

 virtual void addRigidBody(btRigidBody* body, short group, short mask);

 virtual void removeRigidBody(btRigidBody* body);

 ///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
 virtual void removeCollisionObject(btCollisionObject* collisionObject);


 void debugDrawConstraint(btTypedConstraint* constraint);

 virtual void debugDrawWorld();

 virtual void setConstraintSolver(btConstraintSolver* solver);

 virtual btConstraintSolver* getConstraintSolver();
 
 virtual int  getNumConstraints() const;

 virtual btTypedConstraint* getConstraint(int index) ;

 virtual const btTypedConstraint* getConstraint(int index) const;

 
 virtual btDynamicsWorldType getWorldType() const
 {
  return BT_DISCRETE_DYNAMICS_WORLD;
 }
 
 ///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
 virtual void clearForces();

 ///apply gravity, call this once per timestep
 virtual void applyGravity();

 virtual void setNumTasks(int numTasks)
 {
        (void) numTasks;
 }

 ///obsolete, use updateActions instead
 virtual void updateVehicles(btScalar timeStep)
 {
  updateActions(timeStep);
 }

 ///obsolete, use addAction instead
 virtual void addVehicle(btActionInterface* vehicle);
 ///obsolete, use removeAction instead
 virtual void removeVehicle(btActionInterface* vehicle);
 ///obsolete, use addAction instead
 virtual void addCharacter(btActionInterface* character);
 ///obsolete, use removeAction instead
 virtual void removeCharacter(btActionInterface* character);

 void setSynchronizeAllMotionStates(bool synchronizeAll)
 {
  m_synchronizeAllMotionStates = synchronizeAll;
 }
 bool getSynchronizeAllMotionStates() const
 {
  return m_synchronizeAllMotionStates;
 }

 ///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (see Bullet/Demos/SerializeDemo)
 virtual void serialize(btSerializer* serializer);

};

#endif //BT_DISCRETE_DYNAMICS_WORLD_H
//// ../src/BulletDynamics/Dynamics/btDynamicsWorld.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_DYNAMICS_WORLD_H
#define BT_DYNAMICS_WORLD_H

#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

class btTypedConstraint;
class btActionInterface;
class btConstraintSolver;
class btDynamicsWorld;


/// Type for the callback for each tick
typedef void (*btInternalTickCallback)(btDynamicsWorld *world, btScalar timeStep);

enum btDynamicsWorldType
{
 BT_SIMPLE_DYNAMICS_WORLD=1,
 BT_DISCRETE_DYNAMICS_WORLD=2,
 BT_CONTINUOUS_DYNAMICS_WORLD=3,
 BT_SOFT_RIGID_DYNAMICS_WORLD=4
};

///The btDynamicsWorld is the interface class for several dynamics implementation, basic, discrete, parallel, and continuous etc.
class btDynamicsWorld : public btCollisionWorld
{

protected:
  btInternalTickCallback m_internalTickCallback;
  btInternalTickCallback m_internalPreTickCallback;
  void* m_worldUserInfo;

  btContactSolverInfo m_solverInfo;

public:
  

  btDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* broadphase,btCollisionConfiguration* collisionConfiguration)
  :btCollisionWorld(dispatcher,broadphase,collisionConfiguration), m_internalTickCallback(0),m_internalPreTickCallback(0), m_worldUserInfo(0)
  {
  }

  virtual ~btDynamicsWorld()
  {
  }
  
  ///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
  ///By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
  ///in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
  ///You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
  virtual int  stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.))=0;
   
  virtual void debugDrawWorld() = 0;
    
  virtual void addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false) 
  { 
   (void)constraint; (void)disableCollisionsBetweenLinkedBodies;
  }

  virtual void removeConstraint(btTypedConstraint* constraint) {(void)constraint;}

  virtual void addAction(btActionInterface* action) = 0;

  virtual void removeAction(btActionInterface* action) = 0;

  //once a rigidbody is added to the dynamics world, it will get this gravity assigned
  //existing rigidbodies in the world get gravity assigned too, during this method
  virtual void setGravity(const btVector3& gravity) = 0;
  virtual btVector3 getGravity () const = 0;

  virtual void synchronizeMotionStates() = 0;

  virtual void addRigidBody(btRigidBody* body) = 0;

  virtual void addRigidBody(btRigidBody* body, short group, short mask) = 0;

  virtual void removeRigidBody(btRigidBody* body) = 0;

  virtual void setConstraintSolver(btConstraintSolver* solver) = 0;

  virtual btConstraintSolver* getConstraintSolver() = 0;
  
  virtual int  getNumConstraints() const { return 0;  }
  
  virtual btTypedConstraint* getConstraint(int index)  { (void)index;  return 0;  }
  
  virtual const btTypedConstraint* getConstraint(int index) const { (void)index; return 0; }

  virtual btDynamicsWorldType getWorldType() const=0;

  virtual void clearForces() = 0;

  /// Set the callback for when an internal tick (simulation substep) happens, optional user info
  void setInternalTickCallback(btInternalTickCallback cb, void* worldUserInfo=0,bool isPreTick=false) 
  { 
   if (isPreTick)
   {
    m_internalPreTickCallback = cb;
   } else
   {
    m_internalTickCallback = cb; 
   }
   m_worldUserInfo = worldUserInfo;
  }

  void setWorldUserInfo(void* worldUserInfo)
  {
   m_worldUserInfo = worldUserInfo;
  }

  void* getWorldUserInfo() const
  {
   return m_worldUserInfo;
  }

  btContactSolverInfo& getSolverInfo()
  {
   return m_solverInfo;
  }


  ///obsolete, use addAction instead.
  virtual void addVehicle(btActionInterface* vehicle) {(void)vehicle;}
  ///obsolete, use removeAction instead
  virtual void removeVehicle(btActionInterface* vehicle) {(void)vehicle;}
  ///obsolete, use addAction instead.
  virtual void addCharacter(btActionInterface* character) {(void)character;}
  ///obsolete, use removeAction instead
  virtual void removeCharacter(btActionInterface* character) {(void)character;}


};

#endif //BT_DYNAMICS_WORLD_H


//// ../src/BulletDynamics/ConstraintSolver/btContactSolverInfo.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONTACT_SOLVER_INFO
#define BT_CONTACT_SOLVER_INFO

enum btSolverMode
{
 SOLVER_RANDMIZE_ORDER = 1,
 SOLVER_FRICTION_SEPARATE = 2,
 SOLVER_USE_WARMSTARTING = 4,
 SOLVER_USE_FRICTION_WARMSTARTING = 8,
 SOLVER_USE_2_FRICTION_DIRECTIONS = 16,
 SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32,
 SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
 SOLVER_CACHE_FRIENDLY = 128,
 SOLVER_SIMD = 256, //enabled for Windows, the solver innerloop is branchless SIMD, 40% faster than FPU/scalar version
 SOLVER_CUDA = 512 //will be open sourced during Game Developers Conference 2009. Much faster.
};

class btContactSolverInfoData
{ public: 
 

 btScalar m_tau;
 btScalar m_damping;//global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
 btScalar m_friction;
 btScalar m_timeStep;
 btScalar m_restitution;
 int  m_numIterations;
 btScalar m_maxErrorReduction;
 btScalar m_sor;
 btScalar m_erp;//used as Baumgarte factor
 btScalar m_erp2;//used in Split Impulse
 btScalar m_globalCfm;//constraint force mixing
 int   m_splitImpulse;
 btScalar m_splitImpulsePenetrationThreshold;
 btScalar m_linearSlop;
 btScalar m_warmstartingFactor;

 int   m_solverMode;
 int m_restingContactRestitutionThreshold;
 int   m_minimumSolverBatchSize;


};

class btContactSolverInfo : public btContactSolverInfoData
{ public: 

 

 inline btContactSolverInfo()
 {
  m_tau = btScalar(0.6);
  m_damping = btScalar(1.0);
  m_friction = btScalar(0.3);
  m_restitution = btScalar(0.);
  m_maxErrorReduction = btScalar(20.);
  m_numIterations = 10;
  m_erp = btScalar(0.2);
  m_erp2 = btScalar(0.1);
  m_globalCfm = btScalar(0.);
  m_sor = btScalar(1.);
  m_splitImpulse = false;
  m_splitImpulsePenetrationThreshold = -0.02f;
  m_linearSlop = btScalar(0.0);
  m_warmstartingFactor=btScalar(0.85);
  m_solverMode = SOLVER_USE_WARMSTARTING | SOLVER_SIMD;// | SOLVER_RANDMIZE_ORDER;
  m_restingContactRestitutionThreshold = 2;//resting contact lifetime threshold to disable restitution
  m_minimumSolverBatchSize = 128; //try to combine islands until the amount of constraints reaches this limit
 }
};

#endif //BT_CONTACT_SOLVER_INFO
//// ../src/BulletDynamics/Dynamics/btContinuousDynamicsWorld.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONTINUOUS_DYNAMICS_WORLD_H
#define BT_CONTINUOUS_DYNAMICS_WORLD_H

#include "btDiscreteDynamicsWorld.h"

///btContinuousDynamicsWorld adds optional (per object) continuous collision detection for fast moving objects to the btDiscreteDynamicsWorld.
///This copes with fast moving objects that otherwise would tunnel/miss collisions.
///Under construction, don't use yet! Please use btDiscreteDynamicsWorld instead.
class btContinuousDynamicsWorld : public btDiscreteDynamicsWorld
{

 void updateTemporalAabbs(btScalar timeStep);

 public:

  btContinuousDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration);
  virtual ~btContinuousDynamicsWorld();
  
  ///time stepping with calculation of time of impact for selected fast moving objects
  virtual void internalSingleStepSimulation( btScalar timeStep);

  virtual void calculateTimeOfImpacts(btScalar timeStep);

  virtual btDynamicsWorldType getWorldType() const
  {
   return BT_CONTINUOUS_DYNAMICS_WORLD;
  }

};

#endif //BT_CONTINUOUS_DYNAMICS_WORLD_H
//// ../src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SIMPLE_DYNAMICS_WORLD_H
#define BT_SIMPLE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;

///The btSimpleDynamicsWorld serves as unit-test and to verify more complicated and optimized dynamics worlds.
///Please use btDiscreteDynamicsWorld instead (or btContinuousDynamicsWorld once it is finished).
class btSimpleDynamicsWorld : public btDynamicsWorld
{
protected:

 btConstraintSolver* m_constraintSolver;

 bool m_ownsConstraintSolver;

 void predictUnconstraintMotion(btScalar timeStep);
 
 void integrateTransforms(btScalar timeStep);
  
 btVector3 m_gravity;
 
public:



 ///this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
 btSimpleDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration);

 virtual ~btSimpleDynamicsWorld();
  
 ///maxSubSteps/fixedTimeStep for interpolation is currently ignored for btSimpleDynamicsWorld, use btDiscreteDynamicsWorld instead
 virtual int stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));

 virtual void setGravity(const btVector3& gravity);

 virtual btVector3 getGravity () const;

 virtual void addRigidBody(btRigidBody* body);

 virtual void addRigidBody(btRigidBody* body, short group, short mask);

 virtual void removeRigidBody(btRigidBody* body);

 virtual void debugDrawWorld();
    
 virtual void addAction(btActionInterface* action);

 virtual void removeAction(btActionInterface* action);

 ///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
 virtual void removeCollisionObject(btCollisionObject* collisionObject);
 
 virtual void updateAabbs();

 virtual void synchronizeMotionStates();

 virtual void setConstraintSolver(btConstraintSolver* solver);

 virtual btConstraintSolver* getConstraintSolver();

 virtual btDynamicsWorldType getWorldType() const
 {
  return BT_SIMPLE_DYNAMICS_WORLD;
 }

 virtual void clearForces();

};

#endif //BT_SIMPLE_DYNAMICS_WORLD_H
//// ../src/BulletDynamics/Dynamics/btRigidBody.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_RIGIDBODY_H
#define BT_RIGIDBODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class btCollisionShape;
class btMotionState;
class btTypedConstraint;


extern btScalar gDeactivationTime;
extern bool gDisableDeactivation;

#ifdef BT_USE_DOUBLE_PRECISION
#define btRigidBodyData btRigidBodyDoubleData
#define btRigidBodyDataName "btRigidBodyDoubleData"
#else
#define btRigidBodyData btRigidBodyFloatData
#define btRigidBodyDataName "btRigidBodyFloatData"
#endif //BT_USE_DOUBLE_PRECISION


enum btRigidBodyFlags
{
 BT_DISABLE_WORLD_GRAVITY = 1
};


///The btRigidBody is the main class for rigid body objects. It is derived from btCollisionObject, so it keeps a pointer to a btCollisionShape.
///It is recommended for performance and memory use to share btCollisionShape objects whenever possible.
///There are 3 types of rigid bodies: 
///- A) Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
///- B) Fixed objects with zero mass. They are not moving (basically collision objects)
///- C) Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction, and Bullet calculates a velocity based on the timestep and previous and current world transform.
///Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
///Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to allow active objects to activate/wake up sleeping objects)
class btRigidBody  : public btCollisionObject
{

 btMatrix3x3 m_invInertiaTensorWorld;
 btVector3  m_linearVelocity;
 btVector3  m_angularVelocity;
 btScalar  m_inverseMass;
 btVector3  m_linearFactor;

 btVector3  m_gravity; 
 btVector3  m_gravity_acceleration;
 btVector3  m_invInertiaLocal;
 btVector3  m_totalForce;
 btVector3  m_totalTorque;
 
 btScalar  m_linearDamping;
 btScalar  m_angularDamping;

 bool   m_additionalDamping;
 btScalar  m_additionalDampingFactor;
 btScalar  m_additionalLinearDampingThresholdSqr;
 btScalar  m_additionalAngularDampingThresholdSqr;
 btScalar  m_additionalAngularDampingFactor;


 btScalar  m_linearSleepingThreshold;
 btScalar  m_angularSleepingThreshold;

 //m_optionalMotionState allows to automatic synchronize the world transform for active objects
 btMotionState* m_optionalMotionState;

 //keep track of typed constraints referencing this rigid body
 btAlignedObjectArray<btTypedConstraint*> m_constraintRefs;

 int    m_rigidbodyFlags;
 
 int    m_debugBodyId;
 

protected:

 ATTRIBUTE_ALIGNED64(btVector3  m_deltaLinearVelocity);
 btVector3  m_deltaAngularVelocity;
 btVector3  m_angularFactor;
 btVector3  m_invMass;
 btVector3  m_pushVelocity;
 btVector3  m_turnVelocity;


public:


 ///The btRigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
 ///For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
 ///You can use the motion state to synchronize the world transform between physics and graphics objects. 
 ///And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state,
 ///m_startWorldTransform is only used when you don't provide a motion state.
 struct btRigidBodyConstructionInfo
 {
  btScalar   m_mass;

  ///When a motionState is provided, the rigid body will initialize its world transform from the motion state
  ///In this case, m_startWorldTransform is ignored.
  btMotionState*  m_motionState;
  btTransform m_startWorldTransform;

  btCollisionShape* m_collisionShape;
  btVector3   m_localInertia;
  btScalar   m_linearDamping;
  btScalar   m_angularDamping;

  ///best simulation results when friction is non-zero
  btScalar   m_friction;
  ///best simulation results using zero restitution.
  btScalar   m_restitution;

  btScalar   m_linearSleepingThreshold;
  btScalar   m_angularSleepingThreshold;

  //Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
  //Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
  bool    m_additionalDamping;
  btScalar   m_additionalDampingFactor;
  btScalar   m_additionalLinearDampingThresholdSqr;
  btScalar   m_additionalAngularDampingThresholdSqr;
  btScalar   m_additionalAngularDampingFactor;

  btRigidBodyConstructionInfo( btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia):
  m_mass(mass),
   m_motionState(motionState),
   m_collisionShape(collisionShape),
   m_localInertia(localInertia),
   m_linearDamping(btScalar(0.)),
   m_angularDamping(btScalar(0.)),
   m_friction(btScalar(0.5)),
   m_restitution(btScalar(0.)),
   m_linearSleepingThreshold(btScalar(0.8)),
   m_angularSleepingThreshold(btScalar(1.f)),
   m_additionalDamping(false),
   m_additionalDampingFactor(btScalar(0.005)),
   m_additionalLinearDampingThresholdSqr(btScalar(0.01)),
   m_additionalAngularDampingThresholdSqr(btScalar(0.01)),
   m_additionalAngularDampingFactor(btScalar(0.01))
  {
   m_startWorldTransform.setIdentity();
  }
 };

 ///btRigidBody constructor using construction info
 btRigidBody( const btRigidBodyConstructionInfo& constructionInfo);

 ///btRigidBody constructor for backwards compatibility. 
 ///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
 btRigidBody( btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia);


 virtual ~btRigidBody()
        { 
                //No constraints should point to this rigidbody
  //Remove constraints from the dynamics world before you delete the related rigidbodies. 
                btAssert(m_constraintRefs.size()==0); 
        }

protected:

 ///setupRigidBody is only used internally by the constructor
 void setupRigidBody(const btRigidBodyConstructionInfo& constructionInfo);

public:

 void   proceedToTransform(const btTransform& newTrans); 
 
 ///to keep collision detection and dynamics separate we don't store a rigidbody pointer
 ///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
 static const btRigidBody* upcast(const btCollisionObject* colObj)
 {
  if (colObj->getInternalType()&btCollisionObject::CO_RIGID_BODY)
   return (const btRigidBody*)colObj;
  return 0;
 }
 static btRigidBody* upcast(btCollisionObject* colObj)
 {
  if (colObj->getInternalType()&btCollisionObject::CO_RIGID_BODY)
   return (btRigidBody*)colObj;
  return 0;
 }
 
 /// continuous collision detection needs prediction
 void   predictIntegratedTransform(btScalar step, btTransform& predictedTransform) ;
 
 void   saveKinematicState(btScalar step);
 
 void   applyGravity();
 
 void   setGravity(const btVector3& acceleration);  

 const btVector3& getGravity() const
 {
  return m_gravity_acceleration;
 }

 void   setDamping(btScalar lin_damping, btScalar ang_damping);

 btScalar getLinearDamping() const
 {
  return m_linearDamping;
 }

 btScalar getAngularDamping() const
 {
  return m_angularDamping;
 }

 btScalar getLinearSleepingThreshold() const
 {
  return m_linearSleepingThreshold;
 }

 btScalar getAngularSleepingThreshold() const
 {
  return m_angularSleepingThreshold;
 }

 void   applyDamping(btScalar timeStep);

  const btCollisionShape* getCollisionShape() const {
  return m_collisionShape;
 }

  btCollisionShape* getCollisionShape() {
   return m_collisionShape;
 }
 
 void   setMassProps(btScalar mass, const btVector3& inertia);
 
 const btVector3& getLinearFactor() const
 {
  return m_linearFactor;
 }
 void setLinearFactor(const btVector3& linearFactor)
 {
  m_linearFactor = linearFactor;
  m_invMass = m_linearFactor*m_inverseMass;
 }
 btScalar  getInvMass() const { return m_inverseMass; }
 const btMatrix3x3& getInvInertiaTensorWorld() const { 
  return m_invInertiaTensorWorld; 
 }
  
 void   integrateVelocities(btScalar step);

 void   setCenterOfMassTransform(const btTransform& xform);

 void   applyCentralForce(const btVector3& force)
 {
  m_totalForce += force*m_linearFactor;
 }

 const btVector3& getTotalForce() const
 {
  return m_totalForce;
 };

 const btVector3& getTotalTorque() const
 {
  return m_totalTorque;
 };
    
 const btVector3& getInvInertiaDiagLocal() const
 {
  return m_invInertiaLocal;
 };

 void setInvInertiaDiagLocal(const btVector3& diagInvInertia)
 {
  m_invInertiaLocal = diagInvInertia;
 }

 void setSleepingThresholds(btScalar linear,btScalar angular)
 {
  m_linearSleepingThreshold = linear;
  m_angularSleepingThreshold = angular;
 }

 void applyTorque(const btVector3& torque)
 {
  m_totalTorque += torque*m_angularFactor;
 }
 
 void applyForce(const btVector3& force, const btVector3& rel_pos) 
 {
  applyCentralForce(force);
  applyTorque(rel_pos.cross(force*m_linearFactor));
 }
 
 void applyCentralImpulse(const btVector3& impulse)
 {
  m_linearVelocity += impulse *m_linearFactor * m_inverseMass;
 }
 
   void applyTorqueImpulse(const btVector3& torque)
 {
   m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
 }
 
 void applyImpulse(const btVector3& impulse, const btVector3& rel_pos) 
 {
  if (m_inverseMass != btScalar(0.))
  {
   applyCentralImpulse(impulse);
   if (m_angularFactor)
   {
    applyTorqueImpulse(rel_pos.cross(impulse*m_linearFactor));
   }
  }
 }

 void clearForces() 
 {
  m_totalForce.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
  m_totalTorque.setValue(btScalar(0.0), btScalar(0.0), btScalar(0.0));
 }
 
 void updateInertiaTensor();    
 
 const btVector3&     getCenterOfMassPosition() const { 
  return m_worldTransform.getOrigin(); 
 }
 btQuaternion getOrientation() const;
 
 const btTransform&  getCenterOfMassTransform() const { 
  return m_worldTransform; 
 }
 const btVector3&   getLinearVelocity() const { 
  return m_linearVelocity; 
 }
 const btVector3&    getAngularVelocity() const { 
  return m_angularVelocity; 
 }
 

 inline void setLinearVelocity(const btVector3& lin_vel)
 { 
  m_linearVelocity = lin_vel; 
 }

 inline void setAngularVelocity(const btVector3& ang_vel) 
 { 
  m_angularVelocity = ang_vel; 
 }

 btVector3 getVelocityInLocalPoint(const btVector3& rel_pos) const
 {
  //we also calculate lin/ang velocity for kinematic objects
  return m_linearVelocity + m_angularVelocity.cross(rel_pos);

  //for kinematic objects, we could also use use:
  //  return  (m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
 }

 void translate(const btVector3& v) 
 {
  m_worldTransform.getOrigin() += v; 
 }

 
 void getAabb(btVector3& aabbMin,btVector3& aabbMax) const;




 
  btScalar computeImpulseDenominator(const btVector3& pos, const btVector3& normal) const
 {
  btVector3 r0 = pos - getCenterOfMassPosition();

  btVector3 c0 = (r0).cross(normal);

  btVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);

  return m_inverseMass + normal.dot(vec);

 }

  btScalar computeAngularImpulseDenominator(const btVector3& axis) const
 {
  btVector3 vec = axis * getInvInertiaTensorWorld();
  return axis.dot(vec);
 }

  void updateDeactivation(btScalar timeStep)
 {
  if ( (getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION))
   return;

  if ((getLinearVelocity().length2() < m_linearSleepingThreshold*m_linearSleepingThreshold) &&
   (getAngularVelocity().length2() < m_angularSleepingThreshold*m_angularSleepingThreshold))
  {
   m_deactivationTime += timeStep;
  } else
  {
   m_deactivationTime=btScalar(0.);
   setActivationState(0);
  }

 }

  bool wantsSleeping()
 {

  if (getActivationState() == DISABLE_DEACTIVATION)
   return false;

  //disable deactivation
  if (gDisableDeactivation || (gDeactivationTime == btScalar(0.)))
   return false;

  if ( (getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION))
   return true;

  if (m_deactivationTime> gDeactivationTime)
  {
   return true;
  }
  return false;
 }


 
 const btBroadphaseProxy* getBroadphaseProxy() const
 {
  return m_broadphaseHandle;
 }
 btBroadphaseProxy* getBroadphaseProxy() 
 {
  return m_broadphaseHandle;
 }
 void setNewBroadphaseProxy(btBroadphaseProxy* broadphaseProxy)
 {
  m_broadphaseHandle = broadphaseProxy;
 }

 //btMotionState allows to automatic synchronize the world transform for active objects
 btMotionState* getMotionState()
 {
  return m_optionalMotionState;
 }
 const btMotionState* getMotionState() const
 {
  return m_optionalMotionState;
 }
 void setMotionState(btMotionState* motionState)
 {
  m_optionalMotionState = motionState;
  if (m_optionalMotionState)
   motionState->getWorldTransform(m_worldTransform);
 }

 //for experimental overriding of friction/contact solver func
 int m_contactSolverType;
 int m_frictionSolverType;

 void setAngularFactor(const btVector3& angFac)
 {
  m_angularFactor = angFac;
 }

 void setAngularFactor(btScalar angFac)
 {
  m_angularFactor.setValue(angFac,angFac,angFac);
 }
 const btVector3& getAngularFactor() const
 {
  return m_angularFactor;
 }

 //is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
 bool isInWorld() const
 {
  return (getBroadphaseProxy() != 0);
 }

 virtual bool checkCollideWithOverride(btCollisionObject* co);

 void addConstraintRef(btTypedConstraint* c);
 void removeConstraintRef(btTypedConstraint* c);

 btTypedConstraint* getConstraintRef(int index)
 {
  return m_constraintRefs[index];
 }

 int getNumConstraintRefs() const
 {
  return m_constraintRefs.size();
 }

 void setFlags(int flags)
 {
  m_rigidbodyFlags = flags;
 }

 int getFlags() const
 {
  return m_rigidbodyFlags;
 }

 const btVector3& getDeltaLinearVelocity() const
 {
  return m_deltaLinearVelocity;
 }

 const btVector3& getDeltaAngularVelocity() const
 {
  return m_deltaAngularVelocity;
 }

 const btVector3& getPushVelocity() const 
 {
  return m_pushVelocity;
 }

 const btVector3& getTurnVelocity() const 
 {
  return m_turnVelocity;
 }


 ////////////////////////////////////////////////
 ///some internal methods, don't use them
  
 btVector3& internalGetDeltaLinearVelocity()
 {
  return m_deltaLinearVelocity;
 }

 btVector3& internalGetDeltaAngularVelocity()
 {
  return m_deltaAngularVelocity;
 }

 const btVector3& internalGetAngularFactor() const
 {
  return m_angularFactor;
 }

 const btVector3& internalGetInvMass() const
 {
  return m_invMass;
 }
 
 btVector3& internalGetPushVelocity()
 {
  return m_pushVelocity;
 }

 btVector3& internalGetTurnVelocity()
 {
  return m_turnVelocity;
 }

  void internalGetVelocityInLocalPointObsolete(const btVector3& rel_pos, btVector3& velocity ) const
 {
  velocity = getLinearVelocity()+m_deltaLinearVelocity + (getAngularVelocity()+m_deltaAngularVelocity).cross(rel_pos);
 }

  void internalGetAngularVelocity(btVector3& angVel) const
 {
  angVel = getAngularVelocity()+m_deltaAngularVelocity;
 }


 //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
  void internalApplyImpulse(const btVector3& linearComponent, const btVector3& angularComponent,const btScalar impulseMagnitude)
 {
  if (m_inverseMass)
  {
   m_deltaLinearVelocity += linearComponent*impulseMagnitude;
   m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
  }
 }

  void internalApplyPushImpulse(const btVector3& linearComponent, const btVector3& angularComponent,btScalar impulseMagnitude)
 {
  if (m_inverseMass)
  {
   m_pushVelocity += linearComponent*impulseMagnitude;
   m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
  }
 }
 
 void internalWritebackVelocity()
 {
  if (m_inverseMass)
  {
   setLinearVelocity(getLinearVelocity()+ m_deltaLinearVelocity);
   setAngularVelocity(getAngularVelocity()+m_deltaAngularVelocity);
   //m_deltaLinearVelocity.setZero();
   //m_deltaAngularVelocity .setZero();
   //m_originalBody->setCompanionId(-1);
  }
 }


 void internalWritebackVelocity(btScalar timeStep);

 

 ///////////////////////////////////////////////

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer,  class btSerializer* serializer) const;

 virtual void serializeSingleObject(class btSerializer* serializer) const;

};

//@todo add m_optionalMotionState and m_constraintRefs to btRigidBodyData
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btRigidBodyFloatData
{ public: 
 btCollisionObjectFloatData m_collisionObjectData;
 btMatrix3x3FloatData  m_invInertiaTensorWorld;
 btVector3FloatData  m_linearVelocity;
 btVector3FloatData  m_angularVelocity;
 btVector3FloatData  m_angularFactor;
 btVector3FloatData  m_linearFactor;
 btVector3FloatData  m_gravity; 
 btVector3FloatData  m_gravity_acceleration;
 btVector3FloatData  m_invInertiaLocal;
 btVector3FloatData  m_totalForce;
 btVector3FloatData  m_totalTorque;
 float     m_inverseMass;
 float     m_linearDamping;
 float     m_angularDamping;
 float     m_additionalDampingFactor;
 float     m_additionalLinearDampingThresholdSqr;
 float     m_additionalAngularDampingThresholdSqr;
 float     m_additionalAngularDampingFactor;
 float     m_linearSleepingThreshold;
 float     m_angularSleepingThreshold;
 int      m_additionalDamping;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btRigidBodyDoubleData
{ public: 
 btCollisionObjectDoubleData m_collisionObjectData;
 btMatrix3x3DoubleData  m_invInertiaTensorWorld;
 btVector3DoubleData  m_linearVelocity;
 btVector3DoubleData  m_angularVelocity;
 btVector3DoubleData  m_angularFactor;
 btVector3DoubleData  m_linearFactor;
 btVector3DoubleData  m_gravity; 
 btVector3DoubleData  m_gravity_acceleration;
 btVector3DoubleData  m_invInertiaLocal;
 btVector3DoubleData  m_totalForce;
 btVector3DoubleData  m_totalTorque;
 double     m_inverseMass;
 double     m_linearDamping;
 double     m_angularDamping;
 double     m_additionalDampingFactor;
 double     m_additionalLinearDampingThresholdSqr;
 double     m_additionalAngularDampingThresholdSqr;
 double     m_additionalAngularDampingFactor;
 double     m_linearSleepingThreshold;
 double     m_angularSleepingThreshold;
 int      m_additionalDamping;
 char m_padding[4];
};



#endif //BT_RIGIDBODY_H

//// ../src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_POINT2POINTCONSTRAINT_H
#define BT_POINT2POINTCONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btRigidBody;


#ifdef BT_USE_DOUBLE_PRECISION
#define btPoint2PointConstraintData btPoint2PointConstraintDoubleData
#define btPoint2PointConstraintDataName "btPoint2PointConstraintDoubleData"
#else
#define btPoint2PointConstraintData btPoint2PointConstraintFloatData
#define btPoint2PointConstraintDataName "btPoint2PointConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION

class btConstraintSetting
{ public: 
 btConstraintSetting() :
  m_tau(btScalar(0.3)),
  m_damping(btScalar(1.)),
  m_impulseClamp(btScalar(0.))
 {
 }
 btScalar  m_tau;
 btScalar  m_damping;
 btScalar  m_impulseClamp;
};

enum btPoint2PointFlags
{
 BT_P2P_FLAGS_ERP = 1,
 BT_P2P_FLAGS_CFM = 2
};

/// point to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
class btPoint2PointConstraint : public btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
 btJacobianEntry m_jac[3]; //3 orthogonal linear constraints
 
 btVector3 m_pivotInA;
 btVector3 m_pivotInB;
 
 int   m_flags;
 btScalar m_erp;
 btScalar m_cfm;
 
public:

 ///for backwards compatibility during the transition to 'getInfo/getInfo2'
 bool  m_useSolveConstraintObsolete;

 btConstraintSetting m_setting;

 btPoint2PointConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB);

 btPoint2PointConstraint(btRigidBody& rbA,const btVector3& pivotInA);


 virtual void buildJacobian();

 virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info);

 void getInfo1NonVirtual (btTypedConstraint::btConstraintInfo1* info);

 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);

 void getInfo2NonVirtual (btTypedConstraint::btConstraintInfo2* info, const btTransform& body0_trans, const btTransform& body1_trans);

 void updateRHS(btScalar timeStep);

 void setPivotA(const btVector3& pivotA)
 {
  m_pivotInA = pivotA;
 }

 void setPivotB(const btVector3& pivotB)
 {
  m_pivotInB = pivotB;
 }

 const btVector3& getPivotInA() const
 {
  return m_pivotInA;
 }

 const btVector3& getPivotInB() const
 {
  return m_pivotInB;
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 virtual void setParam(int num, btScalar value, int axis = -1);
 ///return the local value of parameter
 virtual btScalar getParam(int num, int axis = -1) const;

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btPoint2PointConstraintFloatData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btVector3FloatData m_pivotInA;
 btVector3FloatData m_pivotInB;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btPoint2PointConstraintDoubleData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btVector3DoubleData m_pivotInA;
 btVector3DoubleData m_pivotInB;
};


 int btPoint2PointConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btPoint2PointConstraintData);

}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btPoint2PointConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btPoint2PointConstraintData* p2pData = (btPoint2PointConstraintData*)dataBuffer;

 btTypedConstraint::serialize(&p2pData->m_typeConstraintData,serializer);
 m_pivotInA.serialize(p2pData->m_pivotInA);
 m_pivotInB.serialize(p2pData->m_pivotInB);

 return btPoint2PointConstraintDataName;
}

#endif //BT_POINT2POINTCONSTRAINT_H
//// ../src/BulletDynamics/ConstraintSolver/btJacobianEntry.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_JACOBIAN_ENTRY_H
#define BT_JACOBIAN_ENTRY_H

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"


//notes:
// Another memory optimization would be to store m_1MinvJt in the remaining 3 w components
// which makes the btJacobianEntry memory layout 16 bytes
// if you only are interested in angular part, just feed massInvA and massInvB zero

/// Jacobian entry is an abstraction that allows to describe constraints
/// it can be used in combination with a constraint solver
/// Can be used to relate the effect of an impulse to the constraint error
class btJacobianEntry
{
public:
 btJacobianEntry() {};
 //constraint between two different rigidbodies
 btJacobianEntry(
  const btMatrix3x3& world2A,
  const btMatrix3x3& world2B,
  const btVector3& rel_pos1,const btVector3& rel_pos2,
  const btVector3& jointAxis,
  const btVector3& inertiaInvA, 
  const btScalar massInvA,
  const btVector3& inertiaInvB,
  const btScalar massInvB)
  :m_linearJointAxis(jointAxis)
 {
  m_aJ = world2A*(rel_pos1.cross(m_linearJointAxis));
  m_bJ = world2B*(rel_pos2.cross(-m_linearJointAxis));
  m_0MinvJt = inertiaInvA * m_aJ;
  m_1MinvJt = inertiaInvB * m_bJ;
  m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);

  btAssert(m_Adiag > btScalar(0.0));
 }

 //angular constraint between two different rigidbodies
 btJacobianEntry(const btVector3& jointAxis,
  const btMatrix3x3& world2A,
  const btMatrix3x3& world2B,
  const btVector3& inertiaInvA,
  const btVector3& inertiaInvB)
  :m_linearJointAxis(btVector3(btScalar(0.),btScalar(0.),btScalar(0.)))
 {
  m_aJ= world2A*jointAxis;
  m_bJ = world2B*-jointAxis;
  m_0MinvJt = inertiaInvA * m_aJ;
  m_1MinvJt = inertiaInvB * m_bJ;
  m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

  btAssert(m_Adiag > btScalar(0.0));
 }

 //angular constraint between two different rigidbodies
 btJacobianEntry(const btVector3& axisInA,
  const btVector3& axisInB,
  const btVector3& inertiaInvA,
  const btVector3& inertiaInvB)
  : m_linearJointAxis(btVector3(btScalar(0.),btScalar(0.),btScalar(0.)))
  , m_aJ(axisInA)
  , m_bJ(-axisInB)
 {
  m_0MinvJt = inertiaInvA * m_aJ;
  m_1MinvJt = inertiaInvB * m_bJ;
  m_Adiag =  m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

  btAssert(m_Adiag > btScalar(0.0));
 }

 //constraint on one rigidbody
 btJacobianEntry(
  const btMatrix3x3& world2A,
  const btVector3& rel_pos1,const btVector3& rel_pos2,
  const btVector3& jointAxis,
  const btVector3& inertiaInvA, 
  const btScalar massInvA)
  :m_linearJointAxis(jointAxis)
 {
  m_aJ= world2A*(rel_pos1.cross(jointAxis));
  m_bJ = world2A*(rel_pos2.cross(-jointAxis));
  m_0MinvJt = inertiaInvA * m_aJ;
  m_1MinvJt = btVector3(btScalar(0.),btScalar(0.),btScalar(0.));
  m_Adiag = massInvA + m_0MinvJt.dot(m_aJ);

  btAssert(m_Adiag > btScalar(0.0));
 }

 btScalar getDiagonal() const { return m_Adiag; }

 // for two constraints on the same rigidbody (for example vehicle friction)
 btScalar getNonDiagonal(const btJacobianEntry& jacB, const btScalar massInvA) const
 {
  const btJacobianEntry& jacA = *this;
  btScalar lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
  btScalar ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
  return lin + ang;
 }

 

 // for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
 btScalar getNonDiagonal(const btJacobianEntry& jacB,const btScalar massInvA,const btScalar massInvB) const
 {
  const btJacobianEntry& jacA = *this;
  btVector3 lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
  btVector3 ang0 = jacA.m_0MinvJt * jacB.m_aJ;
  btVector3 ang1 = jacA.m_1MinvJt * jacB.m_bJ;
  btVector3 lin0 = massInvA * lin ;
  btVector3 lin1 = massInvB * lin;
  btVector3 sum = ang0+ang1+lin0+lin1;
  return sum[0]+sum[1]+sum[2];
 }

 btScalar getRelativeVelocity(const btVector3& linvelA,const btVector3& angvelA,const btVector3& linvelB,const btVector3& angvelB)
 {
  btVector3 linrel = linvelA - linvelB;
  btVector3 angvela  = angvelA * m_aJ;
  btVector3 angvelb  = angvelB * m_bJ;
  linrel *= m_linearJointAxis;
  angvela += angvelb;
  angvela += linrel;
  btScalar rel_vel2 = angvela[0]+angvela[1]+angvela[2];
  return rel_vel2 + SIMD_EPSILON;
 }
//private:

 btVector3 m_linearJointAxis;
 btVector3 m_aJ;
 btVector3 m_bJ;
 btVector3 m_0MinvJt;
 btVector3 m_1MinvJt;
 //Optimization: can be stored in the w/last component of one of the vectors
 btScalar m_Adiag;

};

#endif //BT_JACOBIAN_ENTRY_H
//// ../src/BulletDynamics/ConstraintSolver/btTypedConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TYPED_CONSTRAINT_H
#define BT_TYPED_CONSTRAINT_H

class btRigidBody;
#include "LinearMath/btScalar.h"
#include "btSolverConstraint.h"

class btSerializer;

//Don't change any of the existing enum values, so add enum types at the end for serialization compatibility
enum btTypedConstraintType
{
 POINT2POINT_CONSTRAINT_TYPE=3,
 HINGE_CONSTRAINT_TYPE,
 CONETWIST_CONSTRAINT_TYPE,
 D6_CONSTRAINT_TYPE,
 SLIDER_CONSTRAINT_TYPE,
 CONTACT_CONSTRAINT_TYPE,
 D6_SPRING_CONSTRAINT_TYPE,
 MAX_CONSTRAINT_TYPE
};


enum btConstraintParams
{
 BT_CONSTRAINT_ERP=1,
 BT_CONSTRAINT_STOP_ERP,
 BT_CONSTRAINT_CFM,
 BT_CONSTRAINT_STOP_CFM
};

#if 1
 #define btAssertConstrParams(_par) btAssert(_par) 
#else
 #define btAssertConstrParams(_par)
#endif


///TypedConstraint is the baseclass for Bullet constraints and vehicles
class btTypedConstraint : public btTypedObject
{
 int m_userConstraintType;

 union
 {
  int m_userConstraintId;
  void* m_userConstraintPtr;
 };

 btScalar m_breakingImpulseThreshold;
 bool  m_isEnabled;


 bool m_needsFeedback;

 btTypedConstraint& operator=(btTypedConstraint& other)
 {
  btAssert(0);
  (void) other;
  return *this;
 }

protected:
 btRigidBody& m_rbA;
 btRigidBody& m_rbB;
 btScalar m_appliedImpulse;
 btScalar m_dbgDrawSize;

 ///internal method used by the constraint solver, don't use them directly
 btScalar getMotorFactor(btScalar pos, btScalar lowLim, btScalar uppLim, btScalar vel, btScalar timeFact);
 
 static btRigidBody& getFixedBody();

public:

 virtual ~btTypedConstraint() {};
 btTypedConstraint(btTypedConstraintType type, btRigidBody& rbA);
 btTypedConstraint(btTypedConstraintType type, btRigidBody& rbA,btRigidBody& rbB);

 struct btConstraintInfo1 {
  int m_numConstraintRows,nub;
 };

 struct btConstraintInfo2 {
  // integrator parameters: frames per second (1/stepsize), default error
  // reduction parameter (0..1).
  btScalar fps,erp;

  // for the first and second body, pointers to two (linear and angular)
  // n*3 jacobian sub matrices, stored by rows. these matrices will have
  // been initialized to 0 on entry. if the second body is zero then the
  // J2xx pointers may be 0.
  btScalar *m_J1linearAxis,*m_J1angularAxis,*m_J2linearAxis,*m_J2angularAxis;

  // elements to jump from one row to the next in J's
  int rowskip;

  // right hand sides of the equation J*v = c + cfm * lambda. cfm is the
  // "constraint force mixing" vector. c is set to zero on entry, cfm is
  // set to a constant value (typically very small or zero) value on entry.
  btScalar *m_constraintError,*cfm;

  // lo and hi limits for variables (set to -/+ infinity on entry).
  btScalar *m_lowerLimit,*m_upperLimit;

  // findex vector for variables. see the LCP solver interface for a
  // description of what this does. this is set to -1 on entry.
  // note that the returned indexes are relative to the first index of
  // the constraint.
  int *findex;
  // number of solver iterations
  int m_numIterations;

  //damping of the velocity
  btScalar m_damping;
 };

 ///internal method used by the constraint solver, don't use them directly
 virtual void buildJacobian() {};

 ///internal method used by the constraint solver, don't use them directly
 virtual void setupSolverConstraint(btConstraintArray& ca, int solverBodyA,int solverBodyB, btScalar timeStep)
 {
        (void)ca;
        (void)solverBodyA;
        (void)solverBodyB;
        (void)timeStep;
 }
 
 ///internal method used by the constraint solver, don't use them directly
 virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info)=0;

 ///internal method used by the constraint solver, don't use them directly
 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info)=0;

 ///internal method used by the constraint solver, don't use them directly
 void internalSetAppliedImpulse(btScalar appliedImpulse)
 {
  m_appliedImpulse = appliedImpulse;
 }
 ///internal method used by the constraint solver, don't use them directly
 btScalar internalGetAppliedImpulse()
 {
  return m_appliedImpulse;
 }


 btScalar getBreakingImpulseThreshold() const
 {
  return  m_breakingImpulseThreshold;
 }

 void setBreakingImpulseThreshold(btScalar threshold)
 {
  m_breakingImpulseThreshold = threshold;
 }

 bool isEnabled() const
 {
  return m_isEnabled;
 }

 void setEnabled(bool enabled)
 {
  m_isEnabled=enabled;
 }


 ///internal method used by the constraint solver, don't use them directly
 // solveConstraintObsolete(btRigidBody& /*bodyA*/,btRigidBody& /*bodyB*/,btScalar /*timeStep*/) {};

 
 const btRigidBody& getRigidBodyA() const
 {
  return m_rbA;
 }
 const btRigidBody& getRigidBodyB() const
 {
  return m_rbB;
 }

 btRigidBody& getRigidBodyA() 
 {
  return m_rbA;
 }
 btRigidBody& getRigidBodyB()
 {
  return m_rbB;
 }

 int getUserConstraintType() const
 {
  return m_userConstraintType ;
 }

 void setUserConstraintType(int userConstraintType)
 {
  m_userConstraintType = userConstraintType;
 };

 void setUserConstraintId(int uid)
 {
  m_userConstraintId = uid;
 }

 int getUserConstraintId() const
 {
  return m_userConstraintId;
 }

 void setUserConstraintPtr(void* ptr)
 {
  m_userConstraintPtr = ptr;
 }

 void* getUserConstraintPtr()
 {
  return m_userConstraintPtr;
 }

 int getUid() const
 {
  return m_userConstraintId;   
 } 

 bool needsFeedback() const
 {
  return m_needsFeedback;
 }

 ///enableFeedback will allow to read the applied linear and angular impulse
 ///use getAppliedImpulse, getAppliedLinearImpulse and getAppliedAngularImpulse to read feedback information
 void enableFeedback(bool needsFeedback)
 {
  m_needsFeedback = needsFeedback;
 }

 ///getAppliedImpulse is an estimated total applied impulse. 
 ///This feedback could be used to determine breaking constraints or playing sounds.
 btScalar getAppliedImpulse() const
 {
  btAssert(m_needsFeedback);
  return m_appliedImpulse;
 }

 btTypedConstraintType getConstraintType () const
 {
  return btTypedConstraintType(m_objectType);
 }
 
 void setDbgDrawSize(btScalar dbgDrawSize)
 {
  m_dbgDrawSize = dbgDrawSize;
 }
 btScalar getDbgDrawSize()
 {
  return m_dbgDrawSize;
 }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 virtual void setParam(int num, btScalar value, int axis = -1) = 0;

 ///return the local value of parameter
 virtual btScalar getParam(int num, int axis = -1) const = 0;
 
 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

};

// returns angle in range [-SIMD_2_PI, SIMD_2_PI], closest to one of the limits 
// all arguments should be normalized angles (i.e. in range [-SIMD_PI, SIMD_PI])
 btScalar btAdjustAngleToLimits(btScalar angleInRadians, btScalar angleLowerLimitInRadians, btScalar angleUpperLimitInRadians)
{
 if(angleLowerLimitInRadians >= angleUpperLimitInRadians)
 {
  return angleInRadians;
 }
 else if(angleInRadians < angleLowerLimitInRadians)
 {
  btScalar diffLo = btFabs(btNormalizeAngle(angleLowerLimitInRadians - angleInRadians));
  btScalar diffHi = btFabs(btNormalizeAngle(angleUpperLimitInRadians - angleInRadians));
  return (diffLo < diffHi) ? angleInRadians : (angleInRadians + SIMD_2_PI);
 }
 else if(angleInRadians > angleUpperLimitInRadians)
 {
  btScalar diffHi = btFabs(btNormalizeAngle(angleInRadians - angleUpperLimitInRadians));
  btScalar diffLo = btFabs(btNormalizeAngle(angleInRadians - angleLowerLimitInRadians));
  return (diffLo < diffHi) ? (angleInRadians - SIMD_2_PI) : angleInRadians;
 }
 else
 {
  return angleInRadians;
 }
}

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btTypedConstraintData
{ public: 
 btRigidBodyData  *m_rbA;
 btRigidBodyData  *m_rbB;
 char *m_name;

 int m_objectType;
 int m_userConstraintType;
 int m_userConstraintId;
 int m_needsFeedback;

 float m_appliedImpulse;
 float m_dbgDrawSize;

 int m_disableCollisionsBetweenLinkedBodies;
 char m_pad4[4];
 
};

 int btTypedConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btTypedConstraintData);
}



class btAngularLimit
{
private:
 btScalar 
  m_center,
  m_halfRange,
  m_softness,
  m_biasFactor,
  m_relaxationFactor,
  m_correction,
  m_sign;

 bool
  m_solveLimit;

public:
 /// Default constructor initializes limit as inactive, allowing free constraint movement
 btAngularLimit()
  :m_center(0.0f),
  m_halfRange(-1.0f),
  m_softness(0.9f),
  m_biasFactor(0.3f),
  m_relaxationFactor(1.0f),
  m_correction(0.0f),
  m_sign(0.0f),
  m_solveLimit(false)
 {}

 /// Sets all limit's parameters.
 /// When low > high limit becomes inactive.
 /// When high - low > 2PI limit is ineffective too becouse no angle can exceed the limit
 void set(btScalar low, btScalar high, btScalar _softness = 0.9f, btScalar _biasFactor = 0.3f, btScalar _relaxationFactor = 1.0f);

 /// Checks conastaint angle against limit. If limit is active and the angle violates the limit
 /// correction is calculated.
 void test(const btScalar angle);

 /// Returns limit's softness
 inline btScalar getSoftness() const
 {
  return m_softness;
 }

 /// Returns limit's bias factor
 inline btScalar getBiasFactor() const
 {
  return m_biasFactor;
 }

 /// Returns limit's relaxation factor
 inline btScalar getRelaxationFactor() const
 {
  return m_relaxationFactor;
 }

 /// Returns correction value evaluated when test() was invoked 
 inline btScalar getCorrection() const
 {
  return m_correction;
 }

 /// Returns sign value evaluated when test() was invoked 
 inline btScalar getSign() const
 {
  return m_sign;
 }

 /// Gives half of the distance between min and max limit angle
 inline btScalar getHalfRange() const
 {
  return m_halfRange;
 }

 /// Returns true when the last test() invocation recognized limit violation
 inline bool isLimit() const
 {
  return m_solveLimit;
 }

 /// Checks given angle against limit. If limit is active and angle doesn't fit it, the angle
 /// returned is modified so it equals to the limit closest to given angle.
 void fit(btScalar& angle) const;

 /// Returns correction value multiplied by sign value
 btScalar getError() const;

 btScalar getLow() const;

 btScalar getHigh() const;

};



#endif //BT_TYPED_CONSTRAINT_H
//// ../src/BulletDynamics/ConstraintSolver/btSolverConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SOLVER_CONSTRAINT_H
#define BT_SOLVER_CONSTRAINT_H

class btRigidBody;
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "btJacobianEntry.h"

//#define NO_FRICTION_TANGENTIALS 1
#include "btSolverBody.h"


///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
ATTRIBUTE_ALIGNED64 (struct) btSolverConstraint
{
 

 btVector3  m_relpos1CrossNormal;
 btVector3  m_contactNormal;

 btVector3  m_relpos2CrossNormal;
 //btVector3  m_contactNormal2;//usually m_contactNormal2 == -m_contactNormal

 btVector3  m_angularComponentA;
 btVector3  m_angularComponentB;
 
 mutable btSimdScalar m_appliedPushImpulse;
 mutable btSimdScalar m_appliedImpulse;
 
 
 btScalar m_friction;
 btScalar m_jacDiagABInv;
 union
 {
  int m_numConsecutiveRowsPerKernel;
  btScalar m_unusedPadding0;
 };

 union
 {
  int   m_frictionIndex;
  btScalar m_unusedPadding1;
 };
 union
 {
  btRigidBody* m_solverBodyA;
  int    m_companionIdA;
 };
 union
 {
  btRigidBody* m_solverBodyB;
  int    m_companionIdB;
 };
 
 union
 {
  void*  m_originalContactPoint;
  btScalar m_unusedPadding4;
 };

 btScalar  m_rhs;
 btScalar  m_cfm;
 btScalar  m_lowerLimit;
 btScalar  m_upperLimit;

 btScalar  m_rhsPenetration;

 enum  btSolverConstraintType
 {
  BT_SOLVER_CONTACT_1D = 0,
  BT_SOLVER_FRICTION_1D
 };
};

typedef btAlignedObjectArray<btSolverConstraint> btConstraintArray;


#endif //BT_SOLVER_CONSTRAINT_H



//// ../src/BulletDynamics/ConstraintSolver/btSolverBody.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SOLVER_BODY_H
#define BT_SOLVER_BODY_H

class btRigidBody;
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btTransformUtil.h"

///Until we get other contributions, only use SIMD on Windows, when using Visual Studio 2008 or later, and not double precision
#if 0
#define USE_SIMD 1
#endif //


#if 0

struct class_btSimdScalar
{ public: 
  btSimdScalar()
 {

 }

  btSimdScalar(float fl)
 :m_vec128 (_mm_set1_ps(fl))
 {
 }

  btSimdScalar(__m128 v128)
  :m_vec128(v128)
 {
 }
 union
 {
  __m128  m_vec128;
  float  m_floats[4];
  int   m_ints[4];
  btScalar m_unusedPadding;
 };
  __m128 get128()
 {
  return m_vec128;
 }

  const __m128 get128() const
 {
  return m_vec128;
 }

  void set128(__m128 v128)
 {
  m_vec128 = v128;
 }

  operator       __m128()       
 { 
  return m_vec128; 
 }
  operator const __m128() const 
 { 
  return m_vec128; 
 }
 
  operator float() const 
 { 
  return m_floats[0]; 
 }

};

///@brief Return the elementwise product of two btSimdScalar
 btSimdScalar 
operator*(const btSimdScalar& v1, const btSimdScalar& v2) 
{
 return btSimdScalar(_mm_mul_ps(v1.get128(),v2.get128()));
}

///@brief Return the elementwise product of two btSimdScalar
 btSimdScalar 
operator+(const btSimdScalar& v1, const btSimdScalar& v2) 
{
 return btSimdScalar(_mm_add_ps(v1.get128(),v2.get128()));
}


#else
#define btSimdScalar btScalar
#endif

///The btSolverBody is an internal datastructure for the constraint solver. Only necessary data is packed to increase cache coherence/performance.
ATTRIBUTE_ALIGNED64 (struct) btSolverBodyObsolete
{
 
 btVector3  m_deltaLinearVelocity;
 btVector3  m_deltaAngularVelocity;
 btVector3  m_angularFactor;
 btVector3  m_invMass;
 btRigidBody* m_originalBody;
 btVector3  m_pushVelocity;
 btVector3  m_turnVelocity;

 
  void getVelocityInLocalPointObsolete(const btVector3& rel_pos, btVector3& velocity ) const
 {
  if (m_originalBody)
   velocity = m_originalBody->getLinearVelocity()+m_deltaLinearVelocity + (m_originalBody->getAngularVelocity()+m_deltaAngularVelocity).cross(rel_pos);
  else
   velocity.setValue(0,0,0);
 }

  void getAngularVelocity(btVector3& angVel) const
 {
  if (m_originalBody)
   angVel = m_originalBody->getAngularVelocity()+m_deltaAngularVelocity;
  else
   angVel.setValue(0,0,0);
 }


 //Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position
  void applyImpulse(const btVector3& linearComponent, const btVector3& angularComponent,const btScalar impulseMagnitude)
 {
  //if (m_invMass)
  {
   m_deltaLinearVelocity += linearComponent*impulseMagnitude;
   m_deltaAngularVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
  }
 }

  void internalApplyPushImpulse(const btVector3& linearComponent, const btVector3& angularComponent,btScalar impulseMagnitude)
 {
  if (m_originalBody)
  {
   m_pushVelocity += linearComponent*impulseMagnitude;
   m_turnVelocity += angularComponent*(impulseMagnitude*m_angularFactor);
  }
 }
 
 void writebackVelocity()
 {
  if (m_originalBody)
  {
   m_originalBody->setLinearVelocity(m_originalBody->getLinearVelocity()+ m_deltaLinearVelocity);
   m_originalBody->setAngularVelocity(m_originalBody->getAngularVelocity()+m_deltaAngularVelocity);
   
   //m_originalBody->setCompanionId(-1);
  }
 }


 void writebackVelocity(btScalar timeStep)
 {
        (void) timeStep;
  if (m_originalBody)
  {
   m_originalBody->setLinearVelocity(m_originalBody->getLinearVelocity()+ m_deltaLinearVelocity);
   m_originalBody->setAngularVelocity(m_originalBody->getAngularVelocity()+m_deltaAngularVelocity);
   
   //correct the position/orientation based on push/turn recovery
   btTransform newTransform;
   btTransformUtil::integrateTransform(m_originalBody->getWorldTransform(),m_pushVelocity,m_turnVelocity,timeStep,newTransform);
   m_originalBody->setWorldTransform(newTransform);
   
   //m_originalBody->setCompanionId(-1);
  }
 }
 


};

#endif //BT_SOLVER_BODY_H


//// ../src/BulletDynamics/ConstraintSolver/btHingeConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

#ifndef BT_HINGECONSTRAINT_H
#define BT_HINGECONSTRAINT_H

#define _BT_USE_CENTER_LIMIT_ 1


#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btRigidBody;

#ifdef BT_USE_DOUBLE_PRECISION
#define btHingeConstraintData btHingeConstraintDoubleData
#define btHingeConstraintDataName "btHingeConstraintDoubleData"
#else
#define btHingeConstraintData btHingeConstraintFloatData
#define btHingeConstraintDataName "btHingeConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION



enum btHingeFlags
{
 BT_HINGE_FLAGS_CFM_STOP = 1,
 BT_HINGE_FLAGS_ERP_STOP = 2,
 BT_HINGE_FLAGS_CFM_NORM = 4
};


/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
class btHingeConstraint : public btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
 btJacobianEntry m_jac[3]; //3 orthogonal linear constraints
 btJacobianEntry m_jacAng[3]; //2 orthogonal angular constraints+ 1 for limit/motor

 btTransform m_rbAFrame; // constraint axii. Assumes z is hinge axis.
 btTransform m_rbBFrame;

 btScalar m_motorTargetVelocity;
 btScalar m_maxMotorImpulse;


#ifdef _BT_USE_CENTER_LIMIT_
 btAngularLimit m_limit;
#else
 btScalar m_lowerLimit; 
 btScalar m_upperLimit; 
 btScalar m_limitSign;
 btScalar m_correction;

 btScalar m_limitSoftness; 
 btScalar m_biasFactor; 
 btScalar m_relaxationFactor; 

 bool  m_solveLimit;
#endif

 btScalar m_kHinge;


 btScalar m_accLimitImpulse;
 btScalar m_hingeAngle;
 btScalar m_referenceSign;

 bool  m_angularOnly;
 bool  m_enableAngularMotor;
 bool  m_useSolveConstraintObsolete;
 bool  m_useOffsetForConstraintFrame;
 bool  m_useReferenceFrameA;

 btScalar m_accMotorImpulse;

 int   m_flags;
 btScalar m_normalCFM;
 btScalar m_stopCFM;
 btScalar m_stopERP;

 
public:

 btHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btVector3& pivotInA,const btVector3& pivotInB, const btVector3& axisInA,const btVector3& axisInB, bool useReferenceFrameA = false);

 btHingeConstraint(btRigidBody& rbA,const btVector3& pivotInA,const btVector3& axisInA, bool useReferenceFrameA = false);
 
 btHingeConstraint(btRigidBody& rbA,btRigidBody& rbB, const btTransform& rbAFrame, const btTransform& rbBFrame, bool useReferenceFrameA = false);

 btHingeConstraint(btRigidBody& rbA,const btTransform& rbAFrame, bool useReferenceFrameA = false);


 virtual void buildJacobian();

 virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info);

 void getInfo1NonVirtual(btTypedConstraint::btConstraintInfo1* info);

 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);

 void getInfo2NonVirtual(btTypedConstraint::btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btVector3& angVelA,const btVector3& angVelB);

 void getInfo2Internal(btTypedConstraint::btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btVector3& angVelA,const btVector3& angVelB);
 void getInfo2InternalUsingFrameOffset(btTypedConstraint::btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btVector3& angVelA,const btVector3& angVelB);
  

 void updateRHS(btScalar timeStep);

 const btRigidBody& getRigidBodyA() const
 {
  return m_rbA;
 }
 const btRigidBody& getRigidBodyB() const
 {
  return m_rbB;
 }

 btRigidBody& getRigidBodyA() 
 {  
  return m_rbA; 
 } 

 btRigidBody& getRigidBodyB() 
 {  
  return m_rbB; 
 }

 btTransform& getFrameOffsetA()
 {
 return m_rbAFrame;
 }

 btTransform& getFrameOffsetB()
 {
  return m_rbBFrame;
 }

 void setFrames(const btTransform& frameA, const btTransform& frameB);
 
 void setAngularOnly(bool angularOnly)
 {
  m_angularOnly = angularOnly;
 }

 void enableAngularMotor(bool enableMotor,btScalar targetVelocity,btScalar maxMotorImpulse)
 {
  m_enableAngularMotor  = enableMotor;
  m_motorTargetVelocity = targetVelocity;
  m_maxMotorImpulse = maxMotorImpulse;
 }

 // extra motor API, including ability to set a target rotation (as opposed to angular velocity)
 // note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
 //       maintain a given angular target.
 void enableMotor(bool enableMotor)  { m_enableAngularMotor = enableMotor; }
 void setMaxMotorImpulse(btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; }
 void setMotorTarget(const btQuaternion& qAinB, btScalar dt); // qAinB is rotation of body A wrt body B.
 void setMotorTarget(btScalar targetAngle, btScalar dt);


 void setLimit(btScalar low,btScalar high,btScalar _softness = 0.9f, btScalar _biasFactor = 0.3f, btScalar _relaxationFactor = 1.0f)
 {
#ifdef _BT_USE_CENTER_LIMIT_
  m_limit.set(low, high, _softness, _biasFactor, _relaxationFactor);
#else
  m_lowerLimit = btNormalizeAngle(low);
  m_upperLimit = btNormalizeAngle(high);
  m_limitSoftness =  _softness;
  m_biasFactor = _biasFactor;
  m_relaxationFactor = _relaxationFactor;
#endif
 }

 void setAxis(btVector3& axisInA)
 {
  btVector3 rbAxisA1, rbAxisA2;
  btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
  btVector3 pivotInA = m_rbAFrame.getOrigin();
//  m_rbAFrame.getOrigin() = pivotInA;
  m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
          rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
          rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

  btVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

  btQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
  btVector3 rbAxisB1 =  quatRotate(rotationArc,rbAxisA1);
  btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

  m_rbBFrame.getOrigin() = m_rbB.getCenterOfMassTransform().inverse()(m_rbA.getCenterOfMassTransform()(pivotInA));

  m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
          rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
          rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
  m_rbBFrame.getBasis() = m_rbB.getCenterOfMassTransform().getBasis().inverse() * m_rbBFrame.getBasis();

 }

 btScalar getLowerLimit() const
 {
#ifdef _BT_USE_CENTER_LIMIT_
 return m_limit.getLow();
#else
 return m_lowerLimit;
#endif
 }

 btScalar getUpperLimit() const
 {
#ifdef _BT_USE_CENTER_LIMIT_
 return m_limit.getHigh();
#else  
 return m_upperLimit;
#endif
 }


 btScalar getHingeAngle();

 btScalar getHingeAngle(const btTransform& transA,const btTransform& transB);

 void testLimit(const btTransform& transA,const btTransform& transB);


 const btTransform& getAFrame() const { return m_rbAFrame; }; 
 const btTransform& getBFrame() const { return m_rbBFrame; };

 btTransform& getAFrame() { return m_rbAFrame; }; 
 btTransform& getBFrame() { return m_rbBFrame; };

 inline int getSolveLimit()
 {
#ifdef _BT_USE_CENTER_LIMIT_
 return m_limit.isLimit();
#else
 return m_solveLimit;
#endif
 }

 inline btScalar getLimitSign()
 {
#ifdef _BT_USE_CENTER_LIMIT_
 return m_limit.getSign();
#else
  return m_limitSign;
#endif
 }

 inline bool getAngularOnly() 
 { 
  return m_angularOnly; 
 }
 inline bool getEnableAngularMotor() 
 { 
  return m_enableAngularMotor; 
 }
 inline btScalar getMotorTargetVelosity() 
 { 
  return m_motorTargetVelocity; 
 }
 inline btScalar getMaxMotorImpulse() 
 { 
  return m_maxMotorImpulse; 
 }
 // access for UseFrameOffset
 bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
 void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }


 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 virtual void setParam(int num, btScalar value, int axis = -1);
 ///return the local value of parameter
 virtual btScalar getParam(int num, int axis = -1) const;

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btHingeConstraintDoubleData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
 btTransformDoubleData m_rbBFrame;
 int   m_useReferenceFrameA;
 int   m_angularOnly;
 int   m_enableAngularMotor;
 float m_motorTargetVelocity;
 float m_maxMotorImpulse;

 float m_lowerLimit;
 float m_upperLimit;
 float m_limitSoftness;
 float m_biasFactor;
 float m_relaxationFactor;

};
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btHingeConstraintFloatData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
 btTransformFloatData m_rbBFrame;
 int   m_useReferenceFrameA;
 int   m_angularOnly;
 
 int   m_enableAngularMotor;
 float m_motorTargetVelocity;
 float m_maxMotorImpulse;

 float m_lowerLimit;
 float m_upperLimit;
 float m_limitSoftness;
 float m_biasFactor;
 float m_relaxationFactor;

};



 int btHingeConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btHingeConstraintData);
}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btHingeConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btHingeConstraintData* hingeData = (btHingeConstraintData*)dataBuffer;
 btTypedConstraint::serialize(&hingeData->m_typeConstraintData,serializer);

 m_rbAFrame.serialize(hingeData->m_rbAFrame);
 m_rbBFrame.serialize(hingeData->m_rbBFrame);

 hingeData->m_angularOnly = m_angularOnly;
 hingeData->m_enableAngularMotor = m_enableAngularMotor;
 hingeData->m_maxMotorImpulse = float(m_maxMotorImpulse);
 hingeData->m_motorTargetVelocity = float(m_motorTargetVelocity);
 hingeData->m_useReferenceFrameA = m_useReferenceFrameA;
#ifdef _BT_USE_CENTER_LIMIT_
 hingeData->m_lowerLimit = float(m_limit.getLow());
 hingeData->m_upperLimit = float(m_limit.getHigh());
 hingeData->m_limitSoftness = float(m_limit.getSoftness());
 hingeData->m_biasFactor = float(m_limit.getBiasFactor());
 hingeData->m_relaxationFactor = float(m_limit.getRelaxationFactor());
#else
 hingeData->m_lowerLimit = float(m_lowerLimit);
 hingeData->m_upperLimit = float(m_upperLimit);
 hingeData->m_limitSoftness = float(m_limitSoftness);
 hingeData->m_biasFactor = float(m_biasFactor);
 hingeData->m_relaxationFactor = float(m_relaxationFactor);
#endif

 return btHingeConstraintDataName;
}

#endif //BT_HINGECONSTRAINT_H
//// ../src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marcus Hennix
*/



/*
Overview:

btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
It is a fixed translation, 3 degree-of-freedom (DOF) rotational "joint".
It divides the 3 rotational DOFs into swing (movement within a cone) and twist.
Swing is divided into swing1 and swing2 which can have different limits, giving an elliptical shape.
(Note: the cone's base isn't flat, so this ellipse is "embedded" on the surface of a sphere.)

In the contraint's frame of reference:
twist is along the x-axis,
and swing 1 and 2 are along the z and y axes respectively.
*/



#ifndef BT_CONETWISTCONSTRAINT_H
#define BT_CONETWISTCONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btRigidBody;

enum btConeTwistFlags
{
 BT_CONETWIST_FLAGS_LIN_CFM = 1,
 BT_CONETWIST_FLAGS_LIN_ERP = 2,
 BT_CONETWIST_FLAGS_ANG_CFM = 4
};

///btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc)
class btConeTwistConstraint : public btTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
 btJacobianEntry m_jac[3]; //3 orthogonal linear constraints

 btTransform m_rbAFrame; 
 btTransform m_rbBFrame;

 btScalar m_limitSoftness;
 btScalar m_biasFactor;
 btScalar m_relaxationFactor;

 btScalar m_damping;

 btScalar m_swingSpan1;
 btScalar m_swingSpan2;
 btScalar m_twistSpan;

 btScalar m_fixThresh;

 btVector3   m_swingAxis;
 btVector3 m_twistAxis;

 btScalar m_kSwing;
 btScalar m_kTwist;

 btScalar m_twistLimitSign;
 btScalar m_swingCorrection;
 btScalar m_twistCorrection;

 btScalar m_twistAngle;

 btScalar m_accSwingLimitImpulse;
 btScalar m_accTwistLimitImpulse;

 bool  m_angularOnly;
 bool  m_solveTwistLimit;
 bool  m_solveSwingLimit;

 bool m_useSolveConstraintObsolete;

 // not yet used...
 btScalar m_swingLimitRatio;
 btScalar m_twistLimitRatio;
 btVector3   m_twistAxisA;

 // motor
 bool   m_bMotorEnabled;
 bool   m_bNormalizedMotorStrength;
 btQuaternion m_qTarget;
 btScalar  m_maxMotorImpulse;
 btVector3  m_accMotorImpulse;
 
 // parameters
 int   m_flags;
 btScalar m_linCFM;
 btScalar m_linERP;
 btScalar m_angCFM;
 
protected:

 void init();

 void computeConeLimitInfo(const btQuaternion& qCone, // in
  btScalar& swingAngle, btVector3& vSwingAxis, btScalar& swingLimit); // all outs

 void computeTwistLimitInfo(const btQuaternion& qTwist, // in
  btScalar& twistAngle, btVector3& vTwistAxis); // all outs

 void adjustSwingAxisToUseEllipseNormal(btVector3& vSwingAxis) const;


public:

 btConeTwistConstraint(btRigidBody& rbA,btRigidBody& rbB,const btTransform& rbAFrame, const btTransform& rbBFrame);
 
 btConeTwistConstraint(btRigidBody& rbA,const btTransform& rbAFrame);

 virtual void buildJacobian();

 virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info);

 void getInfo1NonVirtual(btTypedConstraint::btConstraintInfo1* info);
 
 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);
 
 void getInfo2NonVirtual(btTypedConstraint::btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btMatrix3x3& invInertiaWorldA,const btMatrix3x3& invInertiaWorldB);

 // solveConstraintObsolete(btRigidBody& bodyA,btRigidBody& bodyB,btScalar timeStep);

 void updateRHS(btScalar timeStep);


 const btRigidBody& getRigidBodyA() const
 {
  return m_rbA;
 }
 const btRigidBody& getRigidBodyB() const
 {
  return m_rbB;
 }

 void setAngularOnly(bool angularOnly)
 {
  m_angularOnly = angularOnly;
 }

 void setLimit(int limitIndex,btScalar limitValue)
 {
  switch (limitIndex)
  {
  case 3:
   {
    m_twistSpan = limitValue;
    break;
   }
  case 4:
   {
    m_swingSpan2 = limitValue;
    break;
   }
  case 5:
   {
    m_swingSpan1 = limitValue;
    break;
   }
  default:
   {
   }
  };
 }

 // setLimit(), a few notes:
 // _softness:
 //  0->1, recommend ~0.8->1.
 //  describes % of limits where movement is free.
 //  beyond this softness %, the limit is gradually enforced until the "hard" (1.0) limit is reached.
 // _biasFactor:
 //  0->1?, recommend 0.3 +/-0.3 or so.
 //  strength with which constraint resists zeroth order (angular, not angular velocity) limit violation.
 // __relaxationFactor:
 //  0->1, recommend to stay near 1.
 //  the lower the value, the less the constraint will fight velocities which violate the angular limits.
 void setLimit(btScalar _swingSpan1,btScalar _swingSpan2,btScalar _twistSpan, btScalar _softness = 1.f, btScalar _biasFactor = 0.3f, btScalar _relaxationFactor = 1.0f)
 {
  m_swingSpan1 = _swingSpan1;
  m_swingSpan2 = _swingSpan2;
  m_twistSpan  = _twistSpan;

  m_limitSoftness =  _softness;
  m_biasFactor = _biasFactor;
  m_relaxationFactor = _relaxationFactor;
 }

 const btTransform& getAFrame() { return m_rbAFrame; }; 
 const btTransform& getBFrame() { return m_rbBFrame; };

 inline int getSolveTwistLimit()
 {
  return m_solveTwistLimit;
 }

 inline int getSolveSwingLimit()
 {
  return m_solveTwistLimit;
 }

 inline btScalar getTwistLimitSign()
 {
  return m_twistLimitSign;
 }

 void calcAngleInfo();
 void calcAngleInfo2(const btTransform& transA, const btTransform& transB,const btMatrix3x3& invInertiaWorldA,const btMatrix3x3& invInertiaWorldB);

 inline btScalar getSwingSpan1()
 {
  return m_swingSpan1;
 }
 inline btScalar getSwingSpan2()
 {
  return m_swingSpan2;
 }
 inline btScalar getTwistSpan()
 {
  return m_twistSpan;
 }
 inline btScalar getTwistAngle()
 {
  return m_twistAngle;
 }
 bool isPastSwingLimit() { return m_solveSwingLimit; }

 void setDamping(btScalar damping) { m_damping = damping; }

 void enableMotor(bool b) { m_bMotorEnabled = b; }
 void setMaxMotorImpulse(btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = false; }
 void setMaxMotorImpulseNormalized(btScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = true; }

 btScalar getFixThresh() { return m_fixThresh; }
 void setFixThresh(btScalar fixThresh) { m_fixThresh = fixThresh; }

 // setMotorTarget:
 // q: the desired rotation of bodyA wrt bodyB.
 // note: if q violates the joint limits, the internal target is clamped to avoid conflicting impulses (very bad for stability)
 // note: don't forget to enableMotor()
 void setMotorTarget(const btQuaternion &q);

 // same as above, but q is the desired rotation of frameA wrt frameB in constraint space
 void setMotorTargetInConstraintSpace(const btQuaternion &q);

 btVector3 GetPointForAngle(btScalar fAngleInRadians, btScalar fLength) const;

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 virtual void setParam(int num, btScalar value, int axis = -1);

 virtual void setFrames(const btTransform& frameA, const btTransform& frameB);

 const btTransform& getFrameOffsetA() const
 {
  return m_rbAFrame;
 }

 const btTransform& getFrameOffsetB() const
 {
  return m_rbBFrame;
 }


 ///return the local value of parameter
 virtual btScalar getParam(int num, int axis = -1) const;

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btConeTwistConstraintData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btTransformFloatData m_rbAFrame;
 btTransformFloatData m_rbBFrame;

 //limits
 float m_swingSpan1;
 float m_swingSpan2;
 float m_twistSpan;
 float m_limitSoftness;
 float m_biasFactor;
 float m_relaxationFactor;

 float m_damping;
  
 char m_pad[4];

};
 


 int btConeTwistConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btConeTwistConstraintData);

}


 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btConeTwistConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btConeTwistConstraintData* cone = (btConeTwistConstraintData*) dataBuffer;
 btTypedConstraint::serialize(&cone->m_typeConstraintData,serializer);

 m_rbAFrame.serializeFloat(cone->m_rbAFrame);
 m_rbBFrame.serializeFloat(cone->m_rbBFrame);
 
 cone->m_swingSpan1 = float(m_swingSpan1);
 cone->m_swingSpan2 = float(m_swingSpan2);
 cone->m_twistSpan = float(m_twistSpan);
 cone->m_limitSoftness = float(m_limitSoftness);
 cone->m_biasFactor = float(m_biasFactor);
 cone->m_relaxationFactor = float(m_relaxationFactor);
 cone->m_damping = float(m_damping);

 return "btConeTwistConstraintData";
}


#endif //BT_CONETWISTCONSTRAINT_H
//// ../src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/


#ifndef BT_GENERIC_6DOF_CONSTRAINT_H
#define BT_GENERIC_6DOF_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btRigidBody;




//! Rotation Limit structure for generic joints
class btRotationalLimitMotor
{
public:
    //! limit_parameters
    //!@{
    btScalar m_loLimit;//!< joint limit
    btScalar m_hiLimit;//!< joint limit
    btScalar m_targetVelocity;//!< target motor velocity
    btScalar m_maxMotorForce;//!< max force on motor
    btScalar m_maxLimitForce;//!< max force on limit
    btScalar m_damping;//!< Damping.
    btScalar m_limitSoftness;//! Relaxation factor
    btScalar m_normalCFM;//!< Constraint force mixing factor
    btScalar m_stopERP;//!< Error tolerance factor when joint is at limit
    btScalar m_stopCFM;//!< Constraint force mixing factor when joint is at limit
    btScalar m_bounce;//!< restitution factor
    bool m_enableMotor;

    //!@}

    //! temp_variables
    //!@{
    btScalar m_currentLimitError;//!  How much is violated this limit
    btScalar m_currentPosition;     //!  current value of angle 
    int m_currentLimit;//!< 0=free, 1=at lo limit, 2=at hi limit
    btScalar m_accumulatedImpulse;
    //!@}

    btRotationalLimitMotor()
    {
     m_accumulatedImpulse = 0.f;
        m_targetVelocity = 0;
        m_maxMotorForce = 0.1f;
        m_maxLimitForce = 300.0f;
        m_loLimit = 1.0f;
        m_hiLimit = -1.0f;
  m_normalCFM = 0.f;
  m_stopERP = 0.2f;
  m_stopCFM = 0.f;
        m_bounce = 0.0f;
        m_damping = 1.0f;
        m_limitSoftness = 0.5f;
        m_currentLimit = 0;
        m_currentLimitError = 0;
        m_enableMotor = false;
    }

    btRotationalLimitMotor(const btRotationalLimitMotor & limot)
    {
        m_targetVelocity = limot.m_targetVelocity;
        m_maxMotorForce = limot.m_maxMotorForce;
        m_limitSoftness = limot.m_limitSoftness;
        m_loLimit = limot.m_loLimit;
        m_hiLimit = limot.m_hiLimit;
  m_normalCFM = limot.m_normalCFM;
  m_stopERP = limot.m_stopERP;
  m_stopCFM = limot.m_stopCFM;
        m_bounce = limot.m_bounce;
        m_currentLimit = limot.m_currentLimit;
        m_currentLimitError = limot.m_currentLimitError;
        m_enableMotor = limot.m_enableMotor;
    }



 //! Is limited
    bool isLimited()
    {
     if(m_loLimit > m_hiLimit) return false;
     return true;
    }

 //! Need apply correction
    bool needApplyTorques()
    {
     if(m_currentLimit == 0 && m_enableMotor == false) return false;
     return true;
    }

 //! calculates  error
 /*!
 calculates m_currentLimit and m_currentLimitError.
 */
 int testLimitValue(btScalar test_value);

 //! apply the correction impulses for two bodies
    btScalar solveAngularLimits(btScalar timeStep,btVector3& axis, btScalar jacDiagABInv,btRigidBody * body0, btRigidBody * body1);

};



class btTranslationalLimitMotor
{
public:
 btVector3 m_lowerLimit;//!< the constraint lower limits
    btVector3 m_upperLimit;//!< the constraint upper limits
    btVector3 m_accumulatedImpulse;
    //! Linear_Limit_parameters
    //!@{
    btScalar m_limitSoftness;//!< Softness for linear limit
    btScalar m_damping;//!< Damping for linear limit
    btScalar m_restitution;//! Bounce parameter for linear limit
 btVector3 m_normalCFM;//!< Constraint force mixing factor
    btVector3 m_stopERP;//!< Error tolerance factor when joint is at limit
 btVector3 m_stopCFM;//!< Constraint force mixing factor when joint is at limit
    //!@}
 bool  m_enableMotor[3];
    btVector3 m_targetVelocity;//!< target motor velocity
    btVector3 m_maxMotorForce;//!< max force on motor
    btVector3 m_currentLimitError;//!  How much is violated this limit
    btVector3 m_currentLinearDiff;//!  Current relative offset of constraint frames
    int   m_currentLimit[3];//!< 0=free, 1=at lower limit, 2=at upper limit

    btTranslationalLimitMotor()
    {
     m_lowerLimit.setValue(0.f,0.f,0.f);
     m_upperLimit.setValue(0.f,0.f,0.f);
     m_accumulatedImpulse.setValue(0.f,0.f,0.f);
  m_normalCFM.setValue(0.f, 0.f, 0.f);
  m_stopERP.setValue(0.2f, 0.2f, 0.2f);
  m_stopCFM.setValue(0.f, 0.f, 0.f);

     m_limitSoftness = 0.7f;
     m_damping = btScalar(1.0f);
     m_restitution = btScalar(0.5f);
  for(int i=0; i < 3; i++) 
  {
   m_enableMotor[i] = false;
   m_targetVelocity[i] = btScalar(0.f);
   m_maxMotorForce[i] = btScalar(0.f);
  }
    }

    btTranslationalLimitMotor(const btTranslationalLimitMotor & other )
    {
     m_lowerLimit = other.m_lowerLimit;
     m_upperLimit = other.m_upperLimit;
     m_accumulatedImpulse = other.m_accumulatedImpulse;

     m_limitSoftness = other.m_limitSoftness ;
     m_damping = other.m_damping;
     m_restitution = other.m_restitution;
  m_normalCFM = other.m_normalCFM;
  m_stopERP = other.m_stopERP;
  m_stopCFM = other.m_stopCFM;

  for(int i=0; i < 3; i++) 
  {
   m_enableMotor[i] = other.m_enableMotor[i];
   m_targetVelocity[i] = other.m_targetVelocity[i];
   m_maxMotorForce[i] = other.m_maxMotorForce[i];
  }
    }

    //! Test limit
 /*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
    */
    inline bool isLimited(int limitIndex)
    {
       return (m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex]);
    }
    inline bool needApplyForce(int limitIndex)
    {
     if(m_currentLimit[limitIndex] == 0 && m_enableMotor[limitIndex] == false) return false;
     return true;
    }
 int testLimitValue(int limitIndex, btScalar test_value);


    btScalar solveLinearAxis(
     btScalar timeStep,
        btScalar jacDiagABInv,
        btRigidBody& body1,const btVector3 &pointInA,
        btRigidBody& body2,const btVector3 &pointInB,
        int limit_index,
        const btVector3 & axis_normal_on_a,
  const btVector3 & anchorPos);


};

enum bt6DofFlags
{
 BT_6DOF_FLAGS_CFM_NORM = 1,
 BT_6DOF_FLAGS_CFM_STOP = 2,
 BT_6DOF_FLAGS_ERP_STOP = 4
};
#define BT_6DOF_FLAGS_AXIS_SHIFT 3 // bits per axis


/// btGeneric6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/*!
btGeneric6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'.
currently this limit supports rotational motors<br>
<ul>
<li> For Linear limits, use btGeneric6DofConstraint.setLinearUpperLimit, btGeneric6DofConstraint.setLinearLowerLimit. You can set the parameters with the btTranslationalLimitMotor structure accsesible through the btGeneric6DofConstraint.getTranslationalLimitMotor method.
At this moment translational motors are not supported. May be in the future. </li>

<li> For Angular limits, use the btRotationalLimitMotor structure for configuring the limit.
This is accessible through btGeneric6DofConstraint.getLimitMotor method,
This brings support for limit parameters and motors. </li>

<li> Angulars limits have these possible ranges:
<table border=1 >
<tr>
 <td><b>AXIS</b></td>
 <td><b>MIN ANGLE</b></td>
 <td><b>MAX ANGLE</b></td>
</tr><tr>
 <td>X</td>
 <td>-PI</td>
 <td>PI</td>
</tr><tr>
 <td>Y</td>
 <td>-PI/2</td>
 <td>PI/2</td>
</tr><tr>
 <td>Z</td>
 <td>-PI</td>
 <td>PI</td>
</tr>
</table>
</li>
</ul>

*/
class btGeneric6DofConstraint : public btTypedConstraint
{
protected:

 //! relative_frames
    //!@{
 btTransform m_frameInA;//!< the constraint space w.r.t body A
    btTransform m_frameInB;//!< the constraint space w.r.t body B
    //!@}

    //! Jacobians
    //!@{
    btJacobianEntry m_jacLinear[3];//!< 3 orthogonal linear constraints
    btJacobianEntry m_jacAng[3];//!< 3 orthogonal angular constraints
    //!@}

 //! Linear_Limit_parameters
    //!@{
    btTranslationalLimitMotor m_linearLimits;
    //!@}


    //! hinge_parameters
    //!@{
    btRotationalLimitMotor m_angularLimits[3];
 //!@}


protected:
    //! temporal variables
    //!@{
    btScalar m_timeStep;
    btTransform m_calculatedTransformA;
    btTransform m_calculatedTransformB;
    btVector3 m_calculatedAxisAngleDiff;
    btVector3 m_calculatedAxis[3];
    btVector3 m_calculatedLinearDiff;
 btScalar m_factA;
 btScalar m_factB;
 bool  m_hasStaticBody;
    
 btVector3 m_AnchorPos; // point betwen pivots of bodies A and B to solve linear axes

    bool m_useLinearReferenceFrameA;
 bool m_useOffsetForConstraintFrame;
    
 int  m_flags;

    //!@}

    btGeneric6DofConstraint& operator=(btGeneric6DofConstraint& other)
    {
        btAssert(0);
        (void) other;
        return *this;
    }


 int setAngularLimits(btTypedConstraint::btConstraintInfo2 *info, int row_offset,const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB);

 int setLinearLimits(btTypedConstraint::btConstraintInfo2 *info, int row, const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB);

    void buildLinearJacobian(
        btJacobianEntry & jacLinear,const btVector3 & normalWorld,
        const btVector3 & pivotAInW,const btVector3 & pivotBInW);

    void buildAngularJacobian(btJacobianEntry & jacAngular,const btVector3 & jointAxisW);

 // tests linear limits
 void calculateLinearInfo();

 //! calcs the euler angles between the two bodies.
    void calculateAngleInfo();



public:

 ///for backwards compatibility during the transition to 'getInfo/getInfo2'
 bool  m_useSolveConstraintObsolete;

    btGeneric6DofConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    btGeneric6DofConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB);
    
 //! Calcs global transform of the offsets
 /*!
 Calcs the global transform for the joint offset for body A an B, and also calcs the agle differences between the bodies.
 \sa btGeneric6DofConstraint.getCalculatedTransformA , btGeneric6DofConstraint.getCalculatedTransformB, btGeneric6DofConstraint.calculateAngleInfo
 */
    void calculateTransforms(const btTransform& transA,const btTransform& transB);

 void calculateTransforms();

 //! Gets the global transform of the offset for body A
    /*!
    \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
    */
    const btTransform & getCalculatedTransformA() const
    {
     return m_calculatedTransformA;
    }

    //! Gets the global transform of the offset for body B
    /*!
    \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
    */
    const btTransform & getCalculatedTransformB() const
    {
     return m_calculatedTransformB;
    }

    const btTransform & getFrameOffsetA() const
    {
     return m_frameInA;
    }

    const btTransform & getFrameOffsetB() const
    {
     return m_frameInB;
    }


    btTransform & getFrameOffsetA()
    {
     return m_frameInA;
    }

    btTransform & getFrameOffsetB()
    {
     return m_frameInB;
    }


 //! performs Jacobian calculation, and also calculates angle differences and axis
    virtual void buildJacobian();

 virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info);

 void getInfo1NonVirtual (btTypedConstraint::btConstraintInfo1* info);

 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);

 void getInfo2NonVirtual (btTypedConstraint::btConstraintInfo2* info,const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB);


    void updateRHS(btScalar timeStep);

 //! Get the rotation axis in global coordinates
 /*!
 \pre btGeneric6DofConstraint.buildJacobian must be called previously.
 */
    btVector3 getAxis(int axis_index) const;

    //! Get the relative Euler angle
    /*!
 \pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
 */
    btScalar getAngle(int axis_index) const;

 //! Get the relative position of the constraint pivot
    /*!
 \pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
 */
 btScalar getRelativePivotPosition(int axis_index) const;

 void setFrames(const btTransform & frameA, const btTransform & frameB);

 //! Test angular limit.
 /*!
 Calculates angular correction and returns true if limit needs to be corrected.
 \pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
 */
    bool testAngularLimitMotor(int axis_index);

    void setLinearLowerLimit(const btVector3& linearLower)
    {
     m_linearLimits.m_lowerLimit = linearLower;
    }

 void getLinearLowerLimit(btVector3& linearLower)
 {
  linearLower = m_linearLimits.m_lowerLimit;
 }

 void setLinearUpperLimit(const btVector3& linearUpper)
 {
  m_linearLimits.m_upperLimit = linearUpper;
 }

 void getLinearUpperLimit(btVector3& linearUpper)
 {
  linearUpper = m_linearLimits.m_upperLimit;
 }

    void setAngularLowerLimit(const btVector3& angularLower)
    {
  for(int i = 0; i < 3; i++) 
   m_angularLimits[i].m_loLimit = btNormalizeAngle(angularLower[i]);
    }

 void getAngularLowerLimit(btVector3& angularLower)
 {
  for(int i = 0; i < 3; i++) 
   angularLower[i] = m_angularLimits[i].m_loLimit;
 }

    void setAngularUpperLimit(const btVector3& angularUpper)
    {
  for(int i = 0; i < 3; i++)
   m_angularLimits[i].m_hiLimit = btNormalizeAngle(angularUpper[i]);
    }

 void getAngularUpperLimit(btVector3& angularUpper)
 {
  for(int i = 0; i < 3; i++)
   angularUpper[i] = m_angularLimits[i].m_hiLimit;
 }

 //! Retrieves the angular limit informacion
    btRotationalLimitMotor * getRotationalLimitMotor(int index)
    {
     return &m_angularLimits[index];
    }

    //! Retrieves the  limit informacion
    btTranslationalLimitMotor * getTranslationalLimitMotor()
    {
     return &m_linearLimits;
    }

    //first 3 are linear, next 3 are angular
    void setLimit(int axis, btScalar lo, btScalar hi)
    {
     if(axis<3)
     {
      m_linearLimits.m_lowerLimit[axis] = lo;
      m_linearLimits.m_upperLimit[axis] = hi;
     }
     else
     {
   lo = btNormalizeAngle(lo);
   hi = btNormalizeAngle(hi);
      m_angularLimits[axis-3].m_loLimit = lo;
      m_angularLimits[axis-3].m_hiLimit = hi;
     }
    }

 //! Test limit
 /*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
    */
    bool isLimited(int limitIndex)
    {
     if(limitIndex<3)
     {
   return m_linearLimits.isLimited(limitIndex);

     }
        return m_angularLimits[limitIndex-3].isLimited();
    }

 virtual void calcAnchorPos(void); // overridable

 int get_limit_motor_info2( btRotationalLimitMotor * limot,
        const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB,
        btTypedConstraint::btConstraintInfo2 *info, int row, btVector3& ax1, int rotational, int rotAllowed = false);

 // access for UseFrameOffset
 bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
 void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 virtual void setParam(int num, btScalar value, int axis = -1);
 ///return the local value of parameter
 virtual btScalar getParam(int num, int axis = -1) const;

 void setAxis( const btVector3& axis1, const btVector3& axis2);


 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

 
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btGeneric6DofConstraintData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
 btTransformFloatData m_rbBFrame;
 
 btVector3FloatData m_linearUpperLimit;
 btVector3FloatData m_linearLowerLimit;

 btVector3FloatData m_angularUpperLimit;
 btVector3FloatData m_angularLowerLimit;
 
 int m_useLinearReferenceFrameA;
 int m_useOffsetForConstraintFrame;
};

 int btGeneric6DofConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btGeneric6DofConstraintData);
}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btGeneric6DofConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{

 btGeneric6DofConstraintData* dof = (btGeneric6DofConstraintData*)dataBuffer;
 btTypedConstraint::serialize(&dof->m_typeConstraintData,serializer);

 m_frameInA.serializeFloat(dof->m_rbAFrame);
 m_frameInB.serializeFloat(dof->m_rbBFrame);

  
 int i;
 for (i=0;i<3;i++)
 {
  dof->m_angularLowerLimit.m_floats[i] =  float(m_angularLimits[i].m_loLimit);
  dof->m_angularUpperLimit.m_floats[i] =  float(m_angularLimits[i].m_hiLimit);
  dof->m_linearLowerLimit.m_floats[i] = float(m_linearLimits.m_lowerLimit[i]);
  dof->m_linearUpperLimit.m_floats[i] = float(m_linearLimits.m_upperLimit[i]);
 }
 
 dof->m_useLinearReferenceFrameA = m_useLinearReferenceFrameA? 1 : 0;
 dof->m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame ? 1 : 0;

 return "btGeneric6DofConstraintData";
}





#endif //BT_GENERIC_6DOF_CONSTRAINT_H
//// ../src/BulletDynamics/ConstraintSolver/btSliderConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

#ifndef BT_SLIDER_CONSTRAINT_H
#define BT_SLIDER_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"



class btRigidBody;



#define SLIDER_CONSTRAINT_DEF_SOFTNESS  (btScalar(1.0))
#define SLIDER_CONSTRAINT_DEF_DAMPING  (btScalar(1.0))
#define SLIDER_CONSTRAINT_DEF_RESTITUTION (btScalar(0.7))
#define SLIDER_CONSTRAINT_DEF_CFM   (btScalar(0.f))


enum btSliderFlags
{
 BT_SLIDER_FLAGS_CFM_DIRLIN = (1 << 0),
 BT_SLIDER_FLAGS_ERP_DIRLIN = (1 << 1),
 BT_SLIDER_FLAGS_CFM_DIRANG = (1 << 2),
 BT_SLIDER_FLAGS_ERP_DIRANG = (1 << 3),
 BT_SLIDER_FLAGS_CFM_ORTLIN = (1 << 4),
 BT_SLIDER_FLAGS_ERP_ORTLIN = (1 << 5),
 BT_SLIDER_FLAGS_CFM_ORTANG = (1 << 6),
 BT_SLIDER_FLAGS_ERP_ORTANG = (1 << 7),
 BT_SLIDER_FLAGS_CFM_LIMLIN = (1 << 8),
 BT_SLIDER_FLAGS_ERP_LIMLIN = (1 << 9),
 BT_SLIDER_FLAGS_CFM_LIMANG = (1 << 10),
 BT_SLIDER_FLAGS_ERP_LIMANG = (1 << 11)
};


class btSliderConstraint : public btTypedConstraint
{
protected:
 ///for backwards compatibility during the transition to 'getInfo/getInfo2'
 bool  m_useSolveConstraintObsolete;
 bool  m_useOffsetForConstraintFrame;
 btTransform m_frameInA;
    btTransform m_frameInB;
 // use frameA fo define limits, if true
 bool m_useLinearReferenceFrameA;
 // linear limits
 btScalar m_lowerLinLimit;
 btScalar m_upperLinLimit;
 // angular limits
 btScalar m_lowerAngLimit;
 btScalar m_upperAngLimit;
 // softness, restitution and damping for different cases
 // DirLin - moving inside linear limits
 // LimLin - hitting linear limit
 // DirAng - moving inside angular limits
 // LimAng - hitting angular limit
 // OrthoLin, OrthoAng - against constraint axis
 btScalar m_softnessDirLin;
 btScalar m_restitutionDirLin;
 btScalar m_dampingDirLin;
 btScalar m_cfmDirLin;

 btScalar m_softnessDirAng;
 btScalar m_restitutionDirAng;
 btScalar m_dampingDirAng;
 btScalar m_cfmDirAng;

 btScalar m_softnessLimLin;
 btScalar m_restitutionLimLin;
 btScalar m_dampingLimLin;
 btScalar m_cfmLimLin;

 btScalar m_softnessLimAng;
 btScalar m_restitutionLimAng;
 btScalar m_dampingLimAng;
 btScalar m_cfmLimAng;

 btScalar m_softnessOrthoLin;
 btScalar m_restitutionOrthoLin;
 btScalar m_dampingOrthoLin;
 btScalar m_cfmOrthoLin;

 btScalar m_softnessOrthoAng;
 btScalar m_restitutionOrthoAng;
 btScalar m_dampingOrthoAng;
 btScalar m_cfmOrthoAng;
 
 // for interlal use
 bool m_solveLinLim;
 bool m_solveAngLim;

 int m_flags;

 btJacobianEntry m_jacLin[3];
 btScalar  m_jacLinDiagABInv[3];

    btJacobianEntry m_jacAng[3];

 btScalar m_timeStep;
    btTransform m_calculatedTransformA;
    btTransform m_calculatedTransformB;

 btVector3 m_sliderAxis;
 btVector3 m_realPivotAInW;
 btVector3 m_realPivotBInW;
 btVector3 m_projPivotInW;
 btVector3 m_delta;
 btVector3 m_depth;
 btVector3 m_relPosA;
 btVector3 m_relPosB;

 btScalar m_linPos;
 btScalar m_angPos;

 btScalar m_angDepth;
 btScalar m_kAngle;

 bool  m_poweredLinMotor;
    btScalar m_targetLinMotorVelocity;
    btScalar m_maxLinMotorForce;
    btScalar m_accumulatedLinMotorImpulse;
 
 bool  m_poweredAngMotor;
    btScalar m_targetAngMotorVelocity;
    btScalar m_maxAngMotorForce;
    btScalar m_accumulatedAngMotorImpulse;

 //------------------------    
 void initParams();
public:
 // constructors
    btSliderConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    btSliderConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameA);

 // overrides

    virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info);

 void getInfo1NonVirtual(btTypedConstraint::btConstraintInfo1* info);
 
 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);

 void getInfo2NonVirtual(btTypedConstraint::btConstraintInfo2* info, const btTransform& transA, const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB, btScalar rbAinvMass,btScalar rbBinvMass);


 // access
    const btRigidBody& getRigidBodyA() const { return m_rbA; }
    const btRigidBody& getRigidBodyB() const { return m_rbB; }
    const btTransform & getCalculatedTransformA() const { return m_calculatedTransformA; }
    const btTransform & getCalculatedTransformB() const { return m_calculatedTransformB; }
    const btTransform & getFrameOffsetA() const { return m_frameInA; }
    const btTransform & getFrameOffsetB() const { return m_frameInB; }
    btTransform & getFrameOffsetA() { return m_frameInA; }
    btTransform & getFrameOffsetB() { return m_frameInB; }
    btScalar getLowerLinLimit() { return m_lowerLinLimit; }
    void setLowerLinLimit(btScalar lowerLimit) { m_lowerLinLimit = lowerLimit; }
    btScalar getUpperLinLimit() { return m_upperLinLimit; }
    void setUpperLinLimit(btScalar upperLimit) { m_upperLinLimit = upperLimit; }
    btScalar getLowerAngLimit() { return m_lowerAngLimit; }
    void setLowerAngLimit(btScalar lowerLimit) { m_lowerAngLimit = btNormalizeAngle(lowerLimit); }
    btScalar getUpperAngLimit() { return m_upperAngLimit; }
    void setUpperAngLimit(btScalar upperLimit) { m_upperAngLimit = btNormalizeAngle(upperLimit); }
 bool getUseLinearReferenceFrameA() { return m_useLinearReferenceFrameA; }
 btScalar getSoftnessDirLin() { return m_softnessDirLin; }
 btScalar getRestitutionDirLin() { return m_restitutionDirLin; }
 btScalar getDampingDirLin() { return m_dampingDirLin ; }
 btScalar getSoftnessDirAng() { return m_softnessDirAng; }
 btScalar getRestitutionDirAng() { return m_restitutionDirAng; }
 btScalar getDampingDirAng() { return m_dampingDirAng; }
 btScalar getSoftnessLimLin() { return m_softnessLimLin; }
 btScalar getRestitutionLimLin() { return m_restitutionLimLin; }
 btScalar getDampingLimLin() { return m_dampingLimLin; }
 btScalar getSoftnessLimAng() { return m_softnessLimAng; }
 btScalar getRestitutionLimAng() { return m_restitutionLimAng; }
 btScalar getDampingLimAng() { return m_dampingLimAng; }
 btScalar getSoftnessOrthoLin() { return m_softnessOrthoLin; }
 btScalar getRestitutionOrthoLin() { return m_restitutionOrthoLin; }
 btScalar getDampingOrthoLin() { return m_dampingOrthoLin; }
 btScalar getSoftnessOrthoAng() { return m_softnessOrthoAng; }
 btScalar getRestitutionOrthoAng() { return m_restitutionOrthoAng; }
 btScalar getDampingOrthoAng() { return m_dampingOrthoAng; }
 void setSoftnessDirLin(btScalar softnessDirLin) { m_softnessDirLin = softnessDirLin; }
 void setRestitutionDirLin(btScalar restitutionDirLin) { m_restitutionDirLin = restitutionDirLin; }
 void setDampingDirLin(btScalar dampingDirLin) { m_dampingDirLin = dampingDirLin; }
 void setSoftnessDirAng(btScalar softnessDirAng) { m_softnessDirAng = softnessDirAng; }
 void setRestitutionDirAng(btScalar restitutionDirAng) { m_restitutionDirAng = restitutionDirAng; }
 void setDampingDirAng(btScalar dampingDirAng) { m_dampingDirAng = dampingDirAng; }
 void setSoftnessLimLin(btScalar softnessLimLin) { m_softnessLimLin = softnessLimLin; }
 void setRestitutionLimLin(btScalar restitutionLimLin) { m_restitutionLimLin = restitutionLimLin; }
 void setDampingLimLin(btScalar dampingLimLin) { m_dampingLimLin = dampingLimLin; }
 void setSoftnessLimAng(btScalar softnessLimAng) { m_softnessLimAng = softnessLimAng; }
 void setRestitutionLimAng(btScalar restitutionLimAng) { m_restitutionLimAng = restitutionLimAng; }
 void setDampingLimAng(btScalar dampingLimAng) { m_dampingLimAng = dampingLimAng; }
 void setSoftnessOrthoLin(btScalar softnessOrthoLin) { m_softnessOrthoLin = softnessOrthoLin; }
 void setRestitutionOrthoLin(btScalar restitutionOrthoLin) { m_restitutionOrthoLin = restitutionOrthoLin; }
 void setDampingOrthoLin(btScalar dampingOrthoLin) { m_dampingOrthoLin = dampingOrthoLin; }
 void setSoftnessOrthoAng(btScalar softnessOrthoAng) { m_softnessOrthoAng = softnessOrthoAng; }
 void setRestitutionOrthoAng(btScalar restitutionOrthoAng) { m_restitutionOrthoAng = restitutionOrthoAng; }
 void setDampingOrthoAng(btScalar dampingOrthoAng) { m_dampingOrthoAng = dampingOrthoAng; }
 void setPoweredLinMotor(bool onOff) { m_poweredLinMotor = onOff; }
 bool getPoweredLinMotor() { return m_poweredLinMotor; }
 void setTargetLinMotorVelocity(btScalar targetLinMotorVelocity) { m_targetLinMotorVelocity = targetLinMotorVelocity; }
 btScalar getTargetLinMotorVelocity() { return m_targetLinMotorVelocity; }
 void setMaxLinMotorForce(btScalar maxLinMotorForce) { m_maxLinMotorForce = maxLinMotorForce; }
 btScalar getMaxLinMotorForce() { return m_maxLinMotorForce; }
 void setPoweredAngMotor(bool onOff) { m_poweredAngMotor = onOff; }
 bool getPoweredAngMotor() { return m_poweredAngMotor; }
 void setTargetAngMotorVelocity(btScalar targetAngMotorVelocity) { m_targetAngMotorVelocity = targetAngMotorVelocity; }
 btScalar getTargetAngMotorVelocity() { return m_targetAngMotorVelocity; }
 void setMaxAngMotorForce(btScalar maxAngMotorForce) { m_maxAngMotorForce = maxAngMotorForce; }
 btScalar getMaxAngMotorForce() { return m_maxAngMotorForce; }

 btScalar getLinearPos() const { return m_linPos; }
 btScalar getAngularPos() const { return m_angPos; }
 
 

 // access for ODE solver
 bool getSolveLinLimit() { return m_solveLinLim; }
 btScalar getLinDepth() { return m_depth[0]; }
 bool getSolveAngLimit() { return m_solveAngLim; }
 btScalar getAngDepth() { return m_angDepth; }
 // shared code used by ODE solver
 void calculateTransforms(const btTransform& transA,const btTransform& transB);
 void testLinLimits();
 void testAngLimits();
 // access for PE Solver
 btVector3 getAncorInA();
 btVector3 getAncorInB();
 // access for UseFrameOffset
 bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
 void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

 void setFrames(const btTransform& frameA, const btTransform& frameB) 
 { 
  m_frameInA=frameA; 
  m_frameInB=frameB;
  calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
  buildJacobian();
 } 


 ///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
 ///If no axis is provided, it uses the default axis for this constraint.
 virtual void setParam(int num, btScalar value, int axis = -1);
 ///return the local value of parameter
 virtual btScalar getParam(int num, int axis = -1) const;

 virtual int calculateSerializeBufferSize() const;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btSliderConstraintData
{ public: 
 btTypedConstraintData m_typeConstraintData;
 btTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
 btTransformFloatData m_rbBFrame;
 
 float m_linearUpperLimit;
 float m_linearLowerLimit;

 float m_angularUpperLimit;
 float m_angularLowerLimit;

 int m_useLinearReferenceFrameA;
 int m_useOffsetForConstraintFrame;

};


  int btSliderConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btSliderConstraintData);
}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btSliderConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{

 btSliderConstraintData* sliderData = (btSliderConstraintData*) dataBuffer;
 btTypedConstraint::serialize(&sliderData->m_typeConstraintData,serializer);

 m_frameInA.serializeFloat(sliderData->m_rbAFrame);
 m_frameInB.serializeFloat(sliderData->m_rbBFrame);

 sliderData->m_linearUpperLimit = float(m_upperLinLimit);
 sliderData->m_linearLowerLimit = float(m_lowerLinLimit);

 sliderData->m_angularUpperLimit = float(m_upperAngLimit);
 sliderData->m_angularLowerLimit = float(m_lowerAngLimit);

 sliderData->m_useLinearReferenceFrameA = m_useLinearReferenceFrameA;
 sliderData->m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame;

 return "btSliderConstraintData";
}



#endif //BT_SLIDER_CONSTRAINT_H

//// ../src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_GENERIC_6DOF_SPRING_CONSTRAINT_H
#define BT_GENERIC_6DOF_SPRING_CONSTRAINT_H


#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofConstraint.h"


/// Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF

/// DOF index used in enableSpring() and setStiffness() means:
/// 0 : translation X
/// 1 : translation Y
/// 2 : translation Z
/// 3 : rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
/// 4 : rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
/// 5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )

class btGeneric6DofSpringConstraint : public btGeneric6DofConstraint
{
protected:
 bool  m_springEnabled[6];
 btScalar m_equilibriumPoint[6];
 btScalar m_springStiffness[6];
 btScalar m_springDamping[6]; // between 0 and 1 (1 == no damping)
 void internalUpdateSprings(btTypedConstraint::btConstraintInfo2* info);
public: 
    btGeneric6DofSpringConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
 void enableSpring(int index, bool onOff);
 void setStiffness(int index, btScalar stiffness);
 void setDamping(int index, btScalar damping);
 void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
 void setEquilibriumPoint(int index);  // set the current constraint position/orientation as an equilibrium point for given DOF
 void setEquilibriumPoint(int index, btScalar val);

 virtual void setAxis( const btVector3& axis1, const btVector3& axis2);

 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);

 virtual int calculateSerializeBufferSize() const;
 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

};


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
class btGeneric6DofSpringConstraintData
{ public: 
 btGeneric6DofConstraintData m_6dofData;
 
 int   m_springEnabled[6];
 float  m_equilibriumPoint[6];
 float  m_springStiffness[6];
 float  m_springDamping[6];
};

 int btGeneric6DofSpringConstraint::calculateSerializeBufferSize() const
{
 return sizeof(btGeneric6DofSpringConstraintData);
}

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 const char* btGeneric6DofSpringConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
 btGeneric6DofSpringConstraintData* dof = (btGeneric6DofSpringConstraintData*)dataBuffer;
 btGeneric6DofConstraint::serialize(&dof->m_6dofData,serializer);

 int i;
 for (i=0;i<6;i++)
 {
  dof->m_equilibriumPoint[i] = m_equilibriumPoint[i];
  dof->m_springDamping[i] = m_springDamping[i];
  dof->m_springEnabled[i] = m_springEnabled[i]? 1 : 0;
  dof->m_springStiffness[i] = m_springStiffness[i];
 }
 return "btGeneric6DofConstraintData";
}

#endif // BT_GENERIC_6DOF_SPRING_CONSTRAINT_H

//// ../src/BulletDynamics/ConstraintSolver/btUniversalConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_UNIVERSAL_CONSTRAINT_H
#define BT_UNIVERSAL_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofConstraint.h"



/// Constraint similar to ODE Universal Joint
/// has 2 rotatioonal degrees of freedom, similar to Euler rotations around Z (axis 1)
/// and Y (axis 2)
/// Description from ODE manual : 
/// "Given axis 1 on body 1, and axis 2 on body 2 that is perpendicular to axis 1, it keeps them perpendicular. 
/// In other words, rotation of the two bodies about the direction perpendicular to the two axes will be equal."

class btUniversalConstraint : public btGeneric6DofConstraint
{
protected:
 btVector3 m_anchor;
 btVector3 m_axis1;
 btVector3 m_axis2;
public:
 // constructor
 // anchor, axis1 and axis2 are in world coordinate system
 // axis1 must be orthogonal to axis2
    btUniversalConstraint(btRigidBody& rbA, btRigidBody& rbB, btVector3& anchor, btVector3& axis1, btVector3& axis2);
 // access
 const btVector3& getAnchor() { return m_calculatedTransformA.getOrigin(); }
 const btVector3& getAnchor2() { return m_calculatedTransformB.getOrigin(); }
 const btVector3& getAxis1() { return m_axis1; }
 const btVector3& getAxis2() { return m_axis2; }
 btScalar getAngle1() { return getAngle(2); }
 btScalar getAngle2() { return getAngle(1); }
 // limits
 void setUpperLimit(btScalar ang1max, btScalar ang2max) { setAngularUpperLimit(btVector3(0.f, ang1max, ang2max)); }
 void setLowerLimit(btScalar ang1min, btScalar ang2min) { setAngularLowerLimit(btVector3(0.f, ang1min, ang2min)); }

 void setAxis( const btVector3& axis1, const btVector3& axis2);
};



#endif // BT_UNIVERSAL_CONSTRAINT_H

//// ../src/BulletDynamics/ConstraintSolver/btHinge2Constraint.h
/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_HINGE2_CONSTRAINT_H
#define BT_HINGE2_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofSpringConstraint.h"



// Constraint similar to ODE Hinge2 Joint
// has 3 degrees of frredom:
// 2 rotational degrees of freedom, similar to Euler rotations around Z (axis 1) and X (axis 2)
// 1 translational (along axis Z) with suspension spring

class btHinge2Constraint : public btGeneric6DofSpringConstraint
{
protected:
 btVector3 m_anchor;
 btVector3 m_axis1;
 btVector3 m_axis2;
public:
 // constructor
 // anchor, axis1 and axis2 are in world coordinate system
 // axis1 must be orthogonal to axis2
    btHinge2Constraint(btRigidBody& rbA, btRigidBody& rbB, btVector3& anchor, btVector3& axis1, btVector3& axis2);
 // access
 const btVector3& getAnchor() { return m_calculatedTransformA.getOrigin(); }
 const btVector3& getAnchor2() { return m_calculatedTransformB.getOrigin(); }
 const btVector3& getAxis1() { return m_axis1; }
 const btVector3& getAxis2() { return m_axis2; }
 btScalar getAngle1() { return getAngle(2); }
 btScalar getAngle2() { return getAngle(0); }
 // limits
 void setUpperLimit(btScalar ang1max) { setAngularUpperLimit(btVector3(-1.f, 0.f, ang1max)); }
 void setLowerLimit(btScalar ang1min) { setAngularLowerLimit(btVector3( 1.f, 0.f, ang1min)); }
};



#endif // BT_HINGE2_CONSTRAINT_H

//// ../src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H
#define BT_SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

#include "btConstraintSolver.h"
class btIDebugDraw;
#include "btContactConstraint.h"
#include "btSolverBody.h"
#include "btSolverConstraint.h"
#include "btTypedConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"

///The btSequentialImpulseConstraintSolver is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.
class btSequentialImpulseConstraintSolver : public btConstraintSolver
{
protected:

 btConstraintArray   m_tmpSolverContactConstraintPool;
 btConstraintArray   m_tmpSolverNonContactConstraintPool;
 btConstraintArray   m_tmpSolverContactFrictionConstraintPool;
 btAlignedObjectArray<int> m_orderTmpConstraintPool;
 btAlignedObjectArray<int> m_orderFrictionConstraintPool;
 btAlignedObjectArray<btTypedConstraint::btConstraintInfo1> m_tmpConstraintSizesPool;

 void setupFrictionConstraint( btSolverConstraint& solverConstraint, const btVector3& normalAxis,btRigidBody* solverBodyA,btRigidBody* solverBodyIdB,
         btManifoldPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,
         btCollisionObject* colObj0,btCollisionObject* colObj1, btScalar relaxation, 
         btScalar desiredVelocity=0., btScalar cfmSlip=0.);

 btSolverConstraint& addFrictionConstraint(const btVector3& normalAxis,btRigidBody* solverBodyA,btRigidBody* solverBodyB,int frictionIndex,btManifoldPoint& cp,const btVector3& rel_pos1,const btVector3& rel_pos2,btCollisionObject* colObj0,btCollisionObject* colObj1, btScalar relaxation, btScalar desiredVelocity=0., btScalar cfmSlip=0.);
 
 void setupContactConstraint(btSolverConstraint& solverConstraint, btCollisionObject* colObj0, btCollisionObject* colObj1, btManifoldPoint& cp, 
        const btContactSolverInfo& infoGlobal, btVector3& vel, btScalar& rel_vel, btScalar& relaxation, 
        btVector3& rel_pos1, btVector3& rel_pos2);

 void setFrictionConstraintImpulse( btSolverConstraint& solverConstraint, btRigidBody* rb0, btRigidBody* rb1, 
           btManifoldPoint& cp, const btContactSolverInfo& infoGlobal);

 ///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
 unsigned long m_btSeed2;

// void initSolverBody(btSolverBody* solverBody, btCollisionObject* collisionObject);
 btScalar restitutionCurve(btScalar rel_vel, btScalar restitution);

 void convertContact(btPersistentManifold* manifold,const btContactSolverInfo& infoGlobal);


 void resolveSplitPenetrationSIMD(
        btRigidBody& body1,
        btRigidBody& body2,
        const btSolverConstraint& contactConstraint);

 void resolveSplitPenetrationImpulseCacheFriendly(
        btRigidBody& body1,
        btRigidBody& body2,
        const btSolverConstraint& contactConstraint);

 //internal method
 int getOrInitSolverBody(btCollisionObject& body);

 void resolveSingleConstraintRowGeneric(btRigidBody& body1,btRigidBody& body2,const btSolverConstraint& contactConstraint);

 void resolveSingleConstraintRowGenericSIMD(btRigidBody& body1,btRigidBody& body2,const btSolverConstraint& contactConstraint);
 
 void resolveSingleConstraintRowLowerLimit(btRigidBody& body1,btRigidBody& body2,const btSolverConstraint& contactConstraint);
 
 void resolveSingleConstraintRowLowerLimitSIMD(btRigidBody& body1,btRigidBody& body2,const btSolverConstraint& contactConstraint);
  
protected:
 static btRigidBody& getFixedBody();
 
 virtual void solveGroupCacheFriendlySplitImpulseIterations(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc);
 virtual btScalar solveGroupCacheFriendlyFinish(btCollisionObject** bodies ,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc);
 btScalar solveSingleIteration(int iteration, btCollisionObject** bodies ,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc);

 virtual btScalar solveGroupCacheFriendlySetup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc);
 virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer,btStackAlloc* stackAlloc);


public:

 
 btSequentialImpulseConstraintSolver();
 virtual ~btSequentialImpulseConstraintSolver();

 virtual btScalar solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher);
 

 
 ///clear internal cached data and reset random seed
 virtual void reset();
 
 unsigned long btRand2();

 int btRandInt2 (int n);

 void setRandSeed(unsigned long seed)
 {
  m_btSeed2 = seed;
 }
 unsigned long getRandSeed() const
 {
  return m_btSeed2;
 }

};

#ifndef BT_PREFER_SIMD
typedef btSequentialImpulseConstraintSolver btSequentialImpulseConstraintSolverPrefered;
#endif


#endif //BT_SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

//// ../src/BulletDynamics/ConstraintSolver/btConstraintSolver.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONSTRAINT_SOLVER_H
#define BT_CONSTRAINT_SOLVER_H

#include "LinearMath/btScalar.h"

class btPersistentManifold;
class btRigidBody;
class btCollisionObject;
class btTypedConstraint;
struct btContactSolverInfo;
struct btBroadphaseProxy;
class btIDebugDraw;
class btStackAlloc;
class btDispatcher;
/// btConstraintSolver provides solver interface
class btConstraintSolver
{

public:

 virtual ~btConstraintSolver() {}
 
 virtual void prepareSolve (int numBodies, int numManifolds) {;}

 ///solve a group of constraints
 virtual btScalar solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints, const btContactSolverInfo& info,class btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher) = 0;

 virtual void allSolved (const btContactSolverInfo& /* info */,class btIDebugDraw* /* debugDrawer */, btStackAlloc* /* stackAlloc */) {;}

 ///clear internal cached data and reset random seed
 virtual void reset() = 0;
};




#endif //BT_CONSTRAINT_SOLVER_H
//// ../src/BulletDynamics/ConstraintSolver/btContactConstraint.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONTACT_CONSTRAINT_H
#define BT_CONTACT_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

///btContactConstraint can be automatically created to solve contact constraints using the unified btTypedConstraint interface
class btContactConstraint : public btTypedConstraint
{
protected:

 btPersistentManifold m_contactManifold;

public:


 btContactConstraint(btPersistentManifold* contactManifold,btRigidBody& rbA,btRigidBody& rbB);

 void setContactManifold(btPersistentManifold* contactManifold);

 btPersistentManifold* getContactManifold()
 {
  return &m_contactManifold;
 }

 const btPersistentManifold* getContactManifold() const
 {
  return &m_contactManifold;
 }

 virtual ~btContactConstraint();

 virtual void getInfo1 (btTypedConstraint::btConstraintInfo1* info);

 virtual void getInfo2 (btTypedConstraint::btConstraintInfo2* info);

 ///obsolete methods
 virtual void buildJacobian();


};

///very basic collision resolution without friction
btScalar resolveSingleCollision(btRigidBody* body1, class btCollisionObject* colObj2, const btVector3& contactPositionWorld,const btVector3& contactNormalOnB, const struct btContactSolverInfo& solverInfo,btScalar distance);


///resolveSingleBilateral is an obsolete methods used for vehicle friction between two dynamic objects
void resolveSingleBilateral(btRigidBody& body1, const btVector3& pos1,
                      btRigidBody& body2, const btVector3& pos2,
                      btScalar distance, const btVector3& normal,btScalar& impulse ,btScalar timeStep);



#endif //BT_CONTACT_CONSTRAINT_H
//// ../src/BulletDynamics/Vehicle/btRaycastVehicle.h
/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class btVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : public btActionInterface
{

  btAlignedObjectArray<btVector3> m_forwardWS;
  btAlignedObjectArray<btVector3> m_axle;
  btAlignedObjectArray<btScalar> m_forwardImpulse;
  btAlignedObjectArray<btScalar> m_sideImpulse;
 
  ///backwards compatibility
  int m_userConstraintType;
  int m_userConstraintId;

public:
 class btVehicleTuning
  {
   public:

   btVehicleTuning()
    :m_suspensionStiffness(btScalar(5.88)),
    m_suspensionCompression(btScalar(0.83)),
    m_suspensionDamping(btScalar(0.88)),
    m_maxSuspensionTravelCm(btScalar(500.)),
    m_frictionSlip(btScalar(10.5)),
    m_maxSuspensionForce(btScalar(6000.))
   {
   }
   btScalar m_suspensionStiffness;
   btScalar m_suspensionCompression;
   btScalar m_suspensionDamping;
   btScalar m_maxSuspensionTravelCm;
   btScalar m_frictionSlip;
   btScalar m_maxSuspensionForce;

  };
private:

 btScalar m_tau;
 btScalar m_damping;
 btVehicleRaycaster* m_vehicleRaycaster;
 btScalar  m_pitchControl;
 btScalar m_steeringValue; 
 btScalar m_currentVehicleSpeedKmHour;

 btRigidBody* m_chassisBody;

 int m_indexRightAxis;
 int m_indexUpAxis;
 int m_indexForwardAxis;

 void defaultInit(const btRaycastVehicle::btVehicleTuning& tuning);

public:

 //constructor to create a car from an existing rigidbody
 btRaycastVehicle(const btRaycastVehicle::btVehicleTuning& tuning,btRigidBody* chassis, btVehicleRaycaster* raycaster );

 virtual ~btRaycastVehicle() ;


 ///btActionInterface interface
 virtual void updateAction( btCollisionWorld* collisionWorld, btScalar step)
 {
        (void) collisionWorld;
  updateVehicle(step);
 }
 

 ///btActionInterface interface
 void debugDraw(btIDebugDraw* debugDrawer);
   
 const btTransform& getChassisWorldTransform() const;
 
 btScalar rayCast(btWheelInfo& wheel);

 virtual void updateVehicle(btScalar step);
 
 
 void resetSuspension();

 btScalar getSteeringValue(int wheel) const;

 void setSteeringValue(btScalar steering,int wheel);


 void applyEngineForce(btScalar force, int wheel);

 const btTransform& getWheelTransformWS( int wheelIndex ) const;

 void updateWheelTransform( int wheelIndex, bool interpolatedTransform = true );
 
// void setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

 btWheelInfo& addWheel( const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS,btScalar suspensionRestLength,btScalar wheelRadius,const btRaycastVehicle::btVehicleTuning& tuning, bool isFrontWheel);

 inline int  getNumWheels() const {
  return int (m_wheelInfo.size());
 }
 
 btAlignedObjectArray<btWheelInfo> m_wheelInfo;


 const btWheelInfo& getWheelInfo(int index) const;

 btWheelInfo& getWheelInfo(int index);

 void updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform = true);

 
 void setBrake(btScalar brake,int wheelIndex);

 void setPitchControl(btScalar pitch)
 {
  m_pitchControl = pitch;
 }
 
 void updateSuspension(btScalar deltaTime);

 virtual void updateFriction(btScalar timeStep);



 inline btRigidBody* getRigidBody()
 {
  return m_chassisBody;
 }

 const btRigidBody* getRigidBody() const
 {
  return m_chassisBody;
 }

 inline int getRightAxis() const
 {
  return m_indexRightAxis;
 }
 inline int getUpAxis() const
 {
  return m_indexUpAxis;
 }

 inline int getForwardAxis() const
 {
  return m_indexForwardAxis;
 }

 
 ///Worldspace forward vector
 btVector3 getForwardVector() const
 {
  const btTransform& chassisTrans = getChassisWorldTransform(); 

  btVector3 forwardW ( 
     chassisTrans.getBasis()[0][m_indexForwardAxis], 
     chassisTrans.getBasis()[1][m_indexForwardAxis], 
     chassisTrans.getBasis()[2][m_indexForwardAxis]); 

  return forwardW;
 }

 ///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
 btScalar getCurrentSpeedKmHour() const
 {
  return m_currentVehicleSpeedKmHour;
 }

 virtual void setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
 {
  m_indexRightAxis = rightIndex;
  m_indexUpAxis = upIndex;
  m_indexForwardAxis = forwardIndex;
 }


 ///backwards compatibility
 int getUserConstraintType() const
 {
  return m_userConstraintType ;
 }

 void setUserConstraintType(int userConstraintType)
 {
  m_userConstraintType = userConstraintType;
 };

 void setUserConstraintId(int uid)
 {
  m_userConstraintId = uid;
 }

 int getUserConstraintId() const
 {
  return m_userConstraintId;
 }

};

class btDefaultVehicleRaycaster : public btVehicleRaycaster
{
 btDynamicsWorld* m_dynamicsWorld;
public:
 btDefaultVehicleRaycaster(btDynamicsWorld* world)
  :m_dynamicsWorld(world)
 {
 }

 virtual void* castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);

};


#endif //BT_RAYCASTVEHICLE_H

//// ../src/BulletDynamics/Vehicle/btVehicleRaycaster.h
/*
 * Copyright (c) 2005 Erwin Coumans http://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_VEHICLE_RAYCASTER_H
#define BT_VEHICLE_RAYCASTER_H

#include "LinearMath/btVector3.h"

/// btVehicleRaycaster is provides interface for between vehicle simulation and raycasting
class btVehicleRaycaster
{ public: 
virtual ~btVehicleRaycaster()
{
}
 struct btVehicleRaycasterResult
 {
  btVehicleRaycasterResult() :m_distFraction(btScalar(-1.)){};
  btVector3 m_hitPointInWorld;
  btVector3 m_hitNormalInWorld;
  btScalar m_distFraction;
 };

 virtual void* castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result) = 0;

};

#endif //BT_VEHICLE_RAYCASTER_H

//// ../src/BulletDynamics/Vehicle/btWheelInfo.h
/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_WHEEL_INFO_H
#define BT_WHEEL_INFO_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

class btRigidBody;

class btWheelInfoConstructionInfo
{ public: 
 btVector3 m_chassisConnectionCS;
 btVector3 m_wheelDirectionCS;
 btVector3 m_wheelAxleCS;
 btScalar m_suspensionRestLength;
 btScalar m_maxSuspensionTravelCm;
 btScalar m_wheelRadius;
 
 btScalar  m_suspensionStiffness;
 btScalar  m_wheelsDampingCompression;
 btScalar  m_wheelsDampingRelaxation;
 btScalar  m_frictionSlip;
 btScalar  m_maxSuspensionForce;
 bool m_bIsFrontWheel;
 
};

/// btWheelInfo contains information per wheel about friction and suspension.
class btWheelInfo
{ public: 
 struct RaycastInfo
 {
  //set by raycaster
  btVector3 m_contactNormalWS;//contactnormal
  btVector3 m_contactPointWS;//raycast hitpoint
  btScalar m_suspensionLength;
  btVector3 m_hardPointWS;//raycast starting point
  btVector3 m_wheelDirectionWS; //direction in worldspace
  btVector3 m_wheelAxleWS; // axle in worldspace
  bool  m_isInContact;
  void*  m_groundObject; //could be general void* ptr
 };

 RaycastInfo m_raycastInfo;

 btTransform m_worldTransform;
 
 btVector3 m_chassisConnectionPointCS; //const
 btVector3 m_wheelDirectionCS;//const
 btVector3 m_wheelAxleCS; // const or modified by steering
 btScalar m_suspensionRestLength1;//const
 btScalar m_maxSuspensionTravelCm;
 btScalar getSuspensionRestLength() const;
 btScalar m_wheelsRadius;//const
 btScalar m_suspensionStiffness;//const
 btScalar m_wheelsDampingCompression;//const
 btScalar m_wheelsDampingRelaxation;//const
 btScalar m_frictionSlip;
 btScalar m_steering;
 btScalar m_rotation;
 btScalar m_deltaRotation;
 btScalar m_rollInfluence;
 btScalar m_maxSuspensionForce;

 btScalar m_engineForce;

 btScalar m_brake;
 
 bool m_bIsFrontWheel;
 
 void*  m_clientInfo;//can be used to store pointer to sync transforms...

 btWheelInfo(btWheelInfoConstructionInfo& ci)

 {

  m_suspensionRestLength1 = ci.m_suspensionRestLength;
  m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

  m_wheelsRadius = ci.m_wheelRadius;
  m_suspensionStiffness = ci.m_suspensionStiffness;
  m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
  m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
  m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
  m_wheelDirectionCS = ci.m_wheelDirectionCS;
  m_wheelAxleCS = ci.m_wheelAxleCS;
  m_frictionSlip = ci.m_frictionSlip;
  m_steering = btScalar(0.);
  m_engineForce = btScalar(0.);
  m_rotation = btScalar(0.);
  m_deltaRotation = btScalar(0.);
  m_brake = btScalar(0.);
  m_rollInfluence = btScalar(0.1);
  m_bIsFrontWheel = ci.m_bIsFrontWheel;
  m_maxSuspensionForce = ci.m_maxSuspensionForce;

 }

 void updateWheel(const btRigidBody& chassis,RaycastInfo& raycastInfo);

 btScalar m_clippedInvContactDotSuspension;
 btScalar m_suspensionRelativeVelocity;
 //calculated by suspension
 btScalar m_wheelsSuspensionForce;
 btScalar m_skidInfo;

};

#endif //BT_WHEEL_INFO_H

//// ../src/BulletDynamics/Dynamics/btActionInterface.h
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _BT_ACTION_INTERFACE_H
#define _BT_ACTION_INTERFACE_H

class btIDebugDraw;
class btCollisionWorld;

#include "LinearMath/btScalar.h"
#include "btRigidBody.h"

///Basic interface to allow actions such as vehicles and characters to be updated inside a btDynamicsWorld
class btActionInterface
{
protected:

 static btRigidBody& getFixedBody();
 
 
public:

 virtual ~btActionInterface()
 {
 }

 virtual void updateAction( btCollisionWorld* collisionWorld, btScalar deltaTimeStep)=0;

 virtual void debugDraw(btIDebugDraw* debugDrawer) = 0;

};

#endif //_BT_ACTION_INTERFACE_H

