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
 virtual void getAabb(__const__ btTransform& t,btVector3& aabbMin,btVector3& aabbMax) __const__ =0;

 virtual void getBoundingSphere(btVector3& center,btScalar& radius) __const__;

 ///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
 virtual btScalar getAngularMotionDisc() __const__;

 virtual btScalar getContactBreakingThreshold(btScalar defaultContactThresholdFactor) __const__;


 ///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
 ///result is conservative
 void calculateTemporalAabb(__const__ btTransform& curTrans,__const__ btVector3& linvel,__const__ btVector3& angvel,btScalar timeStep, btVector3& temporalAabbMin,btVector3& temporalAabbMax) __const__;



 inline bool isPolyhedral() __const__
 {
  return btBroadphaseProxy::isPolyhedral(getShapeType());
 }

 inline bool isConvex2d() __const__
 {
  return btBroadphaseProxy::isConvex2d(getShapeType());
 }

 inline bool isConvex() __const__
 {
  return btBroadphaseProxy::isConvex(getShapeType());
 }
 inline bool isNonMoving() __const__
 {
  return btBroadphaseProxy::isNonMoving(getShapeType());
 }
 inline bool isConcave() __const__
 {
  return btBroadphaseProxy::isConcave(getShapeType());
 }
 inline bool isCompound() __const__
 {
  return btBroadphaseProxy::isCompound(getShapeType());
 }

 inline bool isSoftBody() __const__
 {
  return btBroadphaseProxy::isSoftBody(getShapeType());
 }

 ///isInfinite is used to catch simulation error (aabb check)
 inline bool isInfinite() __const__
 {
  return btBroadphaseProxy::isInfinite(getShapeType());
 }


 virtual void setLocalScaling(__const__ btVector3& scaling) =0;
 virtual __const__ btVector3& getLocalScaling() __const__ =0;
 virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) __const__ = 0;


//debugging support
 virtual __const__ char* getName()__const__ =0 ;



 int getShapeType() __const__ { return m_shapeType; }
 virtual void setMargin(btScalar margin) = 0;
 virtual btScalar getMargin() __const__ = 0;


 ///optional user data pointer
 void setUserPointer(void* userPtr)
 {
  m_userPointer = userPtr;
 }

 void* getUserPointer() __const__
 {
  return m_userPointer;
 }

 virtual int calculateSerializeBufferSize() __const__;

 ///fills the dataBuffer and returns the struct name (and 0 on failure)
 virtual __const__ char* serialize(void* dataBuffer, btSerializer* serializer) __const__;

 virtual void serializeSingleShape(btSerializer* serializer) __const__;

};


