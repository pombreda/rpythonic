# 80 "/usr/local/include/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.h" 2

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

 __const__ btBroadphaseInterface* getBroadphase() __const__
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

 __const__ btDispatcher* getDispatcher() __const__
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

 virtual void debugDrawObject(__const__ btTransform& worldTransform, __const__ btCollisionShape* shape, __const__ btVector3& color);


 ///LocalShapeInfo gives extra information for complex shapes
 ///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
 struct LocalShapeInfo
 {
  int m_shapePart;
  int m_triangleIndex;

  //const btCollisionShape*	m_shapeTemp;
  //const btTransform*	m_shapeLocalTransform;
 };

 struct LocalRayResult
 {
  LocalRayResult(btCollisionObject* collisionObject,
   LocalShapeInfo* localShapeInfo,
   __const__ btVector3& hitNormalLocal,
   btScalar hitFraction)
  :m_collisionObject(collisionObject),
  m_localShapeInfo(localShapeInfo),
  m_hitNormalLocal(hitNormalLocal),
  m_hitFraction(hitFraction)
  {
  }

  btCollisionObject* m_collisionObject;
  LocalShapeInfo* m_localShapeInfo;
  btVector3 m_hitNormalLocal;
  btScalar m_hitFraction;

 };

 ///RayResultCallback is used to report new raycast results
 struct RayResultCallback
 {
  btScalar m_closestHitFraction;
  btCollisionObject* m_collisionObject;
  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
      //@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback
      unsigned int m_flags;

  virtual ~RayResultCallback()
  {
  }
  bool hasHit() __const__
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

  virtual bool needsCollision(btBroadphaseProxy* proxy0) __const__
  {
   bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
   collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   return collides;
  }


  virtual btScalar addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace) = 0;
 };

 struct ClosestRayResultCallback : public RayResultCallback
 {
  ClosestRayResultCallback(__const__ btVector3& rayFromWorld,__const__ btVector3& rayToWorld)
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
   ;

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
  AllHitsRayResultCallback(__const__ btVector3& rayFromWorld,__const__ btVector3& rayToWorld)
  :m_rayFromWorld(rayFromWorld),
  m_rayToWorld(rayToWorld)
  {
  }

  btAlignedObjectArray<btCollisionObject*> m_collisionObjects;

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
   __const__ btVector3& hitNormalLocal,
   __const__ btVector3& hitPointLocal,
   btScalar hitFraction
   )
  :m_hitCollisionObject(hitCollisionObject),
  m_localShapeInfo(localShapeInfo),
  m_hitNormalLocal(hitNormalLocal),
  m_hitPointLocal(hitPointLocal),
  m_hitFraction(hitFraction)
  {
  }

  btCollisionObject* m_hitCollisionObject;
  LocalShapeInfo* m_localShapeInfo;
  btVector3 m_hitNormalLocal;
  btVector3 m_hitPointLocal;
  btScalar m_hitFraction;
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

  bool hasHit() __const__
  {
   return (m_closestHitFraction < btScalar(1.));
  }



  virtual bool needsCollision(btBroadphaseProxy* proxy0) __const__
  {
   bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
   collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   return collides;
  }

  virtual btScalar addSingleResult(LocalConvexResult& convexResult,bool normalInWorldSpace) = 0;
 };

 struct ClosestConvexResultCallback : public ConvexResultCallback
 {
  ClosestConvexResultCallback(__const__ btVector3& convexFromWorld,__const__ btVector3& convexToWorld)
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
   ;

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

  virtual bool needsCollision(btBroadphaseProxy* proxy0) __const__
  {
   bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
   collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
   return collides;
  }

  virtual btScalar addSingleResult(btManifoldPoint& cp, __const__ btCollisionObject* colObj0,int partId0,int index0,__const__ btCollisionObject* colObj1,int partId1,int index1) = 0;
 };



 int getNumCollisionObjects() __const__
 {
  return int(m_collisionObjects.size());
 }

 /// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
 /// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
 virtual void rayTest(__const__ btVector3& rayFromWorld, __const__ btVector3& rayToWorld, RayResultCallback& resultCallback) __const__;

 /// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
 /// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
 void convexSweepTest (__const__ btConvexShape* castShape, __const__ btTransform& from, __const__ btTransform& to, ConvexResultCallback& resultCallback, btScalar allowedCcdPenetration = btScalar(0.)) __const__;

 ///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
 ///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
 void contactTest(btCollisionObject* colObj, ContactResultCallback& resultCallback);

 ///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
 ///it reports one or more contact points (including the one with deepest penetration)
 void contactPairTest(btCollisionObject* colObjA, btCollisionObject* colObjB, ContactResultCallback& resultCallback);


 /// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
 /// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
 /// This allows more customization.
 static void rayTestSingle(__const__ btTransform& rayFromTrans,__const__ btTransform& rayToTrans,
       btCollisionObject* collisionObject,
       __const__ btCollisionShape* collisionShape,
       __const__ btTransform& colObjWorldTransform,
       RayResultCallback& resultCallback);

 /// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
 static void objectQuerySingle(__const__ btConvexShape* castShape, __const__ btTransform& rayFromTrans,__const__ btTransform& rayToTrans,
       btCollisionObject* collisionObject,
       __const__ btCollisionShape* collisionShape,
       __const__ btTransform& colObjWorldTransform,
       ConvexResultCallback& resultCallback, btScalar allowedPenetration);

 virtual void addCollisionObject(btCollisionObject* collisionObject,short int collisionFilterGroup=btBroadphaseProxy::DefaultFilter,short int collisionFilterMask=btBroadphaseProxy::AllFilter);

 btCollisionObjectArray& getCollisionObjectArray()
 {
  return m_collisionObjects;
 }

 __const__ btCollisionObjectArray& getCollisionObjectArray() __const__
 {
  return m_collisionObjects;
 }


 virtual void removeCollisionObject(btCollisionObject* collisionObject);

 virtual void performDiscreteCollisionDetection();

 btDispatcherInfo& getDispatchInfo()
 {
  return m_dispatchInfo;
 }

 __const__ btDispatcherInfo& getDispatchInfo() __const__
 {
  return m_dispatchInfo;
 }

 bool getForceUpdateAllAabbs() __const__
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

