# 18 "/usr/local/include/G3D/Ray.h" 2 3

namespace G3D {

/**
 A 3D Ray.
 */
class Ray {
private:
    friend class Intersect;

 Vector3 m_origin;

 /** Unit length */
 Vector3 m_direction;

 /** 1.0 / direction */
 Vector3 m_invDirection;


 // The following are for the "ray slope" optimization from
 //  "Fast Ray / Axis-Aligned Bounding Box Overlap Tests using Ray Slopes" 
 //  by Martin Eisemann, Thorsten Grosch, Stefan M\FCller and Marcus Magnor
 //  Computer Graphics Lab, TU Braunschweig, Germany and
 //  University of Koblenz-Landau, Germany*/
 enum Classification {MMM, MMP, MPM, MPP, PMM, PMP, PPM, PPP, POO, MOO, OPO, OMO, OOP, OOM, OMM, OMP, OPM, OPP, MOM, MOP, POM, POP, MMO, MPO, PMO, PPO}; Classification classification;
 // ray slope
 float ibyj, jbyi, kbyj, jbyk, ibyk, kbyi;
 // Precomputed components
 float c_xy, c_xz, c_yx, c_yz, c_zx, c_zy;

public:

 void set(__const__ Vector3& origin, __const__ Vector3& direction);

 __inline__ __const__ Vector3& origin() __const__ {
  return m_origin;
 }

 /** Unit direction vector. */
 __inline__ __const__ Vector3& direction() __const__ {
  return m_direction;
 }

 /** Component-wise inverse of direction vector.  May have inf() components */
 __inline__ __const__ Vector3& invDirection() __const__ {
  return m_invDirection;
 }

 __inline__ Ray() {
  set(Vector3::zero(), Vector3::unitX());
 }

 __inline__ Ray(__const__ Vector3& origin, __const__ Vector3& direction) {
  set(origin, direction);
 }

 Ray(class BinaryInput& b);

 void serialize(class BinaryOutput& b) __const__;
 void deserialize(class BinaryInput& b);

    /**
     Creates a Ray from a origin and a (nonzero) unit direction.
     */
    static Ray fromOriginAndDirection(__const__ Vector3& point, __const__ Vector3& direction) {
        return Ray(point, direction);
    }

 /** Advances the origin along the direction by @a distance */
 __inline__ Ray bump(float distance) __const__ {
  return Ray(m_origin + m_direction * distance, m_direction);
 }

 /** Advances the origin along the @a bumpDirection by @a distance and returns the new ray*/
 __inline__ Ray bump(float distance, __const__ Vector3& bumpDirection) __const__ {
  return Ray(m_origin + bumpDirection * distance, m_direction);
 }

    /**
     Returns the closest point on the Ray to point.
     */
    Vector3 closestPoint(__const__ Vector3& point) __const__ {
        float t = m_direction.dot(point - m_origin);
        if (t < 0) {
            return m_origin;
        } else {
            return m_origin + m_direction * t;
        }
    }

    /**
     Returns the closest distance between point and the Ray
     */
    float distance(__const__ Vector3& point) __const__ {
        return (closestPoint(point) - point).magnitude();
    }

    /**
     Returns the point where the Ray and plane intersect.  If there
     is no intersection, returns a point at infinity.

      Planes are considered one-sided, so the ray will not intersect
      a plane where the normal faces in the traveling direction.
    */
    Vector3 intersection(__const__ class Plane& plane) __const__;

    /**
     Returns the distance until intersection with the sphere or the (solid) ball bounded by the sphere.
     Will be 0 if inside the sphere, inf if there is no intersection.

     The ray direction is <B>not</B> normalized.  If the ray direction
     has unit length, the distance from the origin to intersection
     is equal to the time.  If the direction does not have unit length,
     the distance = time * direction.length().

     See also the G3D::CollisionDetection "movingPoint" methods,
     which give more information about the intersection.

     \param solid If true, rays inside the sphere immediately intersect (good for collision detection).  If false, they hit the opposite side of the sphere (good for ray tracing).
     */
    float intersectionTime(__const__ class Sphere& sphere, bool solid = false) __const__;

    float intersectionTime(__const__ class Plane& plane) __const__;

    float intersectionTime(__const__ class Box& box) __const__;

    float intersectionTime(__const__ class AABox& box) __const__;

    /**
     The three extra arguments are the weights of vertices 0, 1, and 2
     at the intersection point; they are useful for texture mapping
     and interpolated normals.
     */
    float intersectionTime(
        __const__ Vector3& v0, __const__ Vector3& v1, __const__ Vector3& v2,
        __const__ Vector3& edge01, __const__ Vector3& edge02,
        double& w0, double& w1, double& w2) __const__;

     /**
     Ray-triangle intersection for a 1-sided triangle.  Fastest version.
       @cite http://www.acm.org/jgt/papers/MollerTrumbore97/
       http://www.graphics.cornell.edu/pubs/1997/MT97.html
     */
    __inline__ float intersectionTime(
        __const__ Vector3& vert0,
        __const__ Vector3& vert1,
        __const__ Vector3& vert2,
        __const__ Vector3& edge01,
        __const__ Vector3& edge02) __const__;


    __inline__ float intersectionTime(
        __const__ Vector3& vert0,
        __const__ Vector3& vert1,
        __const__ Vector3& vert2) __const__ {

        return intersectionTime(vert0, vert1, vert2, vert1 - vert0, vert2 - vert0);
    }


    __inline__ float intersectionTime(
        __const__ Vector3& vert0,
        __const__ Vector3& vert1,
        __const__ Vector3& vert2,
        double& w0,
        double& w1,
        double& w2) __const__ {

        return intersectionTime(vert0, vert1, vert2, vert1 - vert0, vert2 - vert0, w0, w1, w2);
    }

    /* One-sided triangle 
       */
    __inline__ float intersectionTime(__const__ Triangle& triangle) __const__ {
        return intersectionTime(
            triangle.vertex(0), triangle.vertex(1), triangle.vertex(2),
            triangle.edge01(), triangle.edge02());
    }

    __inline__ float intersectionTime(
        __const__ Triangle& triangle,
        double& w0,
        double& w1,
        double& w2) __const__ {
        return intersectionTime(triangle.vertex(0), triangle.vertex(1), triangle.vertex(2),
            triangle.edge01(), triangle.edge02(), w0, w1, w2);
    }

    /** Refracts about the normal
        using G3D::Vector3::refractionDirection
        and bumps the ray slightly from the newOrigin. */
    Ray refract(
        __const__ Vector3& newOrigin,
        __const__ Vector3& normal,
        float iInside,
        float iOutside) __const__;

    /** Reflects about the normal
        using G3D::Vector3::reflectionDirection
        and bumps the ray slightly from
        the newOrigin. */
    Ray reflect(
        __const__ Vector3& newOrigin,
        __const__ Vector3& normal) __const__;
};
# 238 "/usr/local/include/G3D/Ray.h" 3
__inline__ float Ray::intersectionTime(
    __const__ Vector3& vert0,
    __const__ Vector3& vert1,
    __const__ Vector3& vert2,
    __const__ Vector3& edge1,
    __const__ Vector3& edge2) __const__ {

    (void)vert1;
    (void)vert2;

    // Barycenteric coords
    float u, v;

    float tvec[3], pvec[3], qvec[3];

    // begin calculating determinant - also used to calculate U parameter
    pvec[0]=m_direction[1]*edge2[2]-m_direction[2]*edge2[1]; pvec[1]=m_direction[2]*edge2[0]-m_direction[0]*edge2[2]; pvec[2]=m_direction[0]*edge2[1]-m_direction[1]*edge2[0];;

    // if determinant is near zero, ray lies in plane of triangle
    __const__ float det = (edge1[0]*pvec[0]+edge1[1]*pvec[1]+edge1[2]*pvec[2]);

    if (det < 0.000001) {
        return finf();
    }

    // calculate distance from vert0 to ray origin
    tvec[0]=m_origin[0]-vert0[0]; tvec[1]=m_origin[1]-vert0[1]; tvec[2]=m_origin[2]-vert0[2];;

    // calculate U parameter and test bounds
    u = (tvec[0]*pvec[0]+tvec[1]*pvec[1]+tvec[2]*pvec[2]);
    if ((u < 0.0f) || (u > det)) {
        // Hit the plane outside the triangle
        return finf();
    }

    // prepare to test V parameter
    qvec[0]=tvec[1]*edge1[2]-tvec[2]*edge1[1]; qvec[1]=tvec[2]*edge1[0]-tvec[0]*edge1[2]; qvec[2]=tvec[0]*edge1[1]-tvec[1]*edge1[0];;

    // calculate V parameter and test bounds
    v = (m_direction[0]*qvec[0]+m_direction[1]*qvec[1]+m_direction[2]*qvec[2]);
    if ((v < 0.0f) || (u + v > det)) {
        // Hit the plane outside the triangle
        return finf();
    }


    // Case where we don't need correct (u, v):
    __const__ float t = (edge2[0]*qvec[0]+edge2[1]*qvec[1]+edge2[2]*qvec[2]);

    if (t >= 0.0f) {
        // Note that det must be positive
        return t / det;
    } else {
        // We had to travel backwards in time to intersect
        return finf();
    }
}


__inline__ float Ray::intersectionTime(
    __const__ Vector3& vert0,
    __const__ Vector3& vert1,
    __const__ Vector3& vert2,
    __const__ Vector3& edge1,
    __const__ Vector3& edge2,
    double& w0,
    double& w1,
    double& w2) __const__ {

    (void)vert1;
    (void)vert2;

    // Barycenteric coords
    float u, v;

    float tvec[3], pvec[3], qvec[3];

    // begin calculating determinant - also used to calculate U parameter
    pvec[0]=m_direction[1]*edge2[2]-m_direction[2]*edge2[1]; pvec[1]=m_direction[2]*edge2[0]-m_direction[0]*edge2[2]; pvec[2]=m_direction[0]*edge2[1]-m_direction[1]*edge2[0];;

    // if determinant is near zero, ray lies in plane of triangle
    __const__ float det = (edge1[0]*pvec[0]+edge1[1]*pvec[1]+edge1[2]*pvec[2]);

    if (det < 0.000001) {
        return finf();
    }

    // calculate distance from vert0 to ray origin
    tvec[0]=m_origin[0]-vert0[0]; tvec[1]=m_origin[1]-vert0[1]; tvec[2]=m_origin[2]-vert0[2];;

    // calculate U parameter and test bounds
    u = (tvec[0]*pvec[0]+tvec[1]*pvec[1]+tvec[2]*pvec[2]);
    if ((u < 0.0f) || (u > det)) {
        // Hit the plane outside the triangle
        return finf();
    }

    // prepare to test V parameter
    qvec[0]=tvec[1]*edge1[2]-tvec[2]*edge1[1]; qvec[1]=tvec[2]*edge1[0]-tvec[0]*edge1[2]; qvec[2]=tvec[0]*edge1[1]-tvec[1]*edge1[0];;

    // calculate V parameter and test bounds
    v = (m_direction[0]*qvec[0]+m_direction[1]*qvec[1]+m_direction[2]*qvec[2]);
    if ((v < 0.0f) || (u + v > det)) {
        // Hit the plane outside the triangle
        return finf();
    }

    float t = (edge2[0]*qvec[0]+edge2[1]*qvec[1]+edge2[2]*qvec[2]);

    if (t >= 0) {
        __const__ float inv_det = 1.0f / det;
        t *= inv_det;
        u *= inv_det;
        v *= inv_det;

        w0 = (1.0f - u - v);
        w1 = u;
        w2 = v;

        return t;
    } else {
        // We had to travel backwards in time to intersect
        return finf();
    }
}






}// namespace

