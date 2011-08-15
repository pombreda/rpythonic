# 10 "/usr/local/include/irrlicht/dimension2d.h" 2

namespace irr
{
namespace core
{
 template <class T>
 class vector2d;

 //! Specifies a 2 dimensional size.
 template <class T>
 class dimension2d
 {
  public:
   //! Default constructor for empty dimension
   dimension2d() : Width(0), Height(0) {}
   //! Constructor with width and height
   dimension2d(__const__ T& width, __const__ T& height)
    : Width(width), Height(height) {}

   dimension2d(__const__ vector2d<T>& other); // Defined in vector2d.h

   //! Use this constructor only where you are sure that the conversion is valid.
   template <class U>
   explicit dimension2d(__const__ dimension2d<U>& other) :
    Width((T)other.Width), Height((T)other.Height) { }

   template <class U>
   dimension2d<T>& operator=(__const__ dimension2d<U>& other)
   {
    Width = (T) other.Width;
    Height = (T) other.Height;
    return *this;
   }


   //! Equality operator
   bool operator==(__const__ dimension2d<T>& other) __const__
   {
    return core::equals(Width, other.Width) &&
      core::equals(Height, other.Height);
   }

   //! Inequality operator
   bool operator!=(__const__ dimension2d<T>& other) __const__
   {
    return ! (*this == other);
   }

   bool operator==(__const__ vector2d<T>& other) __const__; // Defined in vector2d.h

   bool operator!=(__const__ vector2d<T>& other) __const__
   {
    return !(*this == other);
   }

   //! Set to new values
   dimension2d<T>& set(__const__ T& width, __const__ T& height)
   {
    Width = width;
    Height = height;
    return *this;
   }

   //! Divide width and height by scalar
   dimension2d<T>& operator/=(__const__ T& scale)
   {
    Width /= scale;
    Height /= scale;
    return *this;
   }

   //! Divide width and height by scalar
   dimension2d<T> operator/(__const__ T& scale) __const__
   {
    return dimension2d<T>(Width/scale, Height/scale);
   }

   //! Multiply width and height by scalar
   dimension2d<T>& operator*=(__const__ T& scale)
   {
    Width *= scale;
    Height *= scale;
    return *this;
   }

   //! Multiply width and height by scalar
   dimension2d<T> operator*(__const__ T& scale) __const__
   {
    return dimension2d<T>(Width*scale, Height*scale);
   }

   //! Add another dimension to this one.
   dimension2d<T>& operator+=(__const__ dimension2d<T>& other)
   {
    Width += other.Width;
    Height += other.Height;
    return *this;
   }

   //! Subtract a dimension from this one
   dimension2d<T>& operator-=(__const__ dimension2d<T>& other)
   {
    Width -= other.Width;
    Height -= other.Height;
    return *this;
   }


   //! Add two dimensions
   dimension2d<T> operator+(__const__ dimension2d<T>& other) __const__
   {
    return dimension2d<T>(Width+other.Width, Height+other.Height);
   }

   //! Get area
   T getArea() __const__
   {
    return Width*Height;
   }

   //! Get the optimal size according to some properties
   /** This is a function often used for texture dimension

			calculations. The function returns the next larger or

			smaller dimension which is a power-of-two dimension

			(2^n,2^m) and/or square (Width=Height).

			\param requirePowerOfTwo Forces the result to use only

			powers of two as values.

			\param requireSquare Makes width==height in the result

			\param larger Choose whether the result is larger or

			smaller than the current dimension. If one dimension

			need not be changed it is kept with any value of larger.

			\param maxValue Maximum texturesize. if value > 0 size is

			clamped to maxValue

			\return The optimal dimension under the given

			constraints. */
# 145 "/usr/local/include/irrlicht/dimension2d.h"
   dimension2d<T> getOptimalSize(
     bool requirePowerOfTwo=true,
     bool requireSquare=false,
     bool larger=true,
     u32 maxValue = 0) __const__
   {
    u32 i=1;
    u32 j=1;
    if (requirePowerOfTwo)
    {
     while (i<(u32)Width)
      i<<=1;
     if (!larger && i!=1 && i!=(u32)Width)
      i>>=1;
     while (j<(u32)Height)
      j<<=1;
     if (!larger && j!=1 && j!=(u32)Height)
      j>>=1;
    }
    else
    {
     i=(u32)Width;
     j=(u32)Height;
    }

    if (requireSquare)
    {
     if ((larger && (i>j)) || (!larger && (i<j)))
      j=i;
     else
      i=j;
    }

    if ( maxValue > 0 && i > maxValue)
     i = maxValue;

    if ( maxValue > 0 && j > maxValue)
     j = maxValue;

    return dimension2d<T>((T)i,(T)j);
   }

   //! Get the interpolated dimension
   /** \param other Other dimension to interpolate with.

			\param d Value between 0.0f and 1.0f.

			\return Interpolated dimension. */
# 191 "/usr/local/include/irrlicht/dimension2d.h"
   dimension2d<T> getInterpolated(__const__ dimension2d<T>& other, f32 d) __const__
   {
    f32 inv = (1.0f - d);
    return dimension2d<T>( (T)(other.Width*inv + Width*d), (T)(other.Height*inv + Height*d));
   }


   //! Width of the dimension.
   T Width;
   //! Height of the dimension.
   T Height;
 };

 //! Typedef for an f32 dimension.
 typedef dimension2d<f32> dimension2df;
 //! Typedef for an unsigned integer dimension.
 typedef dimension2d<u32> dimension2du;

 //! Typedef for an integer dimension.
 /** There are few cases where negative dimensions make sense. Please consider using

		dimension2du instead. */
# 212 "/usr/local/include/irrlicht/dimension2d.h"
 typedef dimension2d<s32> dimension2di;


} // end namespace core
} // end namespace irr


namespace irr
{
namespace scene
{
 //! Simple implementation of the IMesh interface.
 struct SMesh : public IMesh
 {
  //! constructor
  SMesh()
  {



  }

  //! destructor
  virtual ~SMesh()
  {
   // drop buffers
   for (u32 i=0; i<MeshBuffers.size(); ++i)
    MeshBuffers[i]->drop();
  }

  //! returns amount of mesh buffers.
  virtual u32 getMeshBufferCount() __const__
  {
   return MeshBuffers.size();
  }

  //! returns pointer to a mesh buffer
  virtual IMeshBuffer* getMeshBuffer(u32 nr) __const__
  {
   return MeshBuffers[nr];
  }

  //! returns a meshbuffer which fits a material
  /** reverse search */
  virtual IMeshBuffer* getMeshBuffer( __const__ video::SMaterial & material) __const__
  {
   for (s32 i = (s32)MeshBuffers.size()-1; i >= 0; --i)
   {
    if ( material == MeshBuffers[i]->getMaterial())
     return MeshBuffers[i];
   }

   return 0;
  }

  //! returns an axis aligned bounding box
  virtual __const__ core::aabbox3d<f32>& getBoundingBox() __const__
  {
   return BoundingBox;
  }

  //! set user axis aligned bounding box
  virtual void setBoundingBox( __const__ core::aabbox3df& box)
  {
   BoundingBox = box;
  }

  //! recalculates the bounding box
  void recalculateBoundingBox()
  {
   if (MeshBuffers.size())
   {
    BoundingBox = MeshBuffers[0]->getBoundingBox();
    for (u32 i=1; i<MeshBuffers.size(); ++i)
     BoundingBox.addInternalBox(MeshBuffers[i]->getBoundingBox());
   }
   else
    BoundingBox.reset(0.0f, 0.0f, 0.0f);
  }

  //! adds a MeshBuffer
  void addMeshBuffer(IMeshBuffer* buf)
  {
   if (buf)
   {
    buf->grab();
    MeshBuffers.push_back(buf);
   }
  }

  //! sets a flag of all contained materials to a new value
  virtual void setMaterialFlag(video::E_MATERIAL_FLAG flag, bool newvalue)
  {
   for (u32 i=0; i<MeshBuffers.size(); ++i)
    MeshBuffers[i]->getMaterial().setFlag(flag, newvalue);
  }

  //! set the hardware mapping hint, for driver
  virtual void setHardwareMappingHint( E_HARDWARE_MAPPING newMappingHint, E_BUFFER_TYPE buffer=EBT_VERTEX_AND_INDEX )
  {
   for (u32 i=0; i<MeshBuffers.size(); ++i)
    MeshBuffers[i]->setHardwareMappingHint(newMappingHint, buffer);
  }

  //! flags the meshbuffer as changed, reloads hardware buffers
  virtual void setDirty(E_BUFFER_TYPE buffer=EBT_VERTEX_AND_INDEX)
  {
   for (u32 i=0; i<MeshBuffers.size(); ++i)
    MeshBuffers[i]->setDirty(buffer);
  }

  //! The meshbuffers of this mesh
  core::array<IMeshBuffer*> MeshBuffers;

  //! The bounding box of this mesh
  core::aabbox3d<f32> BoundingBox;
 };


} // end namespace scene
} // end namespace irr


# 181 "/usr/include/c++/4.5/i686-linux-gnu/bits/gthr-default.h" 3
/* On Solaris 2.6 up to 9, the libc exposes a POSIX threads interface even if
   -pthreads is not specified.  The functions are dummies and most return an
   error value.  However pthread_once returns 0 without invoking the routine
   it is passed so we cannot pretend that the interface is active if -pthreads
   is not specified.  On Solaris 2.5.1, the interface is not exposed at all so
   we need to play the usual game with weak symbols.  On Solaris 10 and up, a
   working interface is always exposed.  On FreeBSD 6 and later, libc also
   exposes a dummy POSIX threads interface, similar to what Solaris 2.6 up
   to 9 does.  FreeBSD >= 700014 even provides a pthread_cancel stub in libc,
   which means the alternate __gthread_active_p below cannot be used there.  */
# 237 "/usr/include/c++/4.5/i686-linux-gnu/bits/gthr-default.h" 3
static inline int
__gthread_active_p (void)
{
  static void *__const__ __gthread_active_ptr
    = __extension__ (void *) &__gthrw_pthread_cancel;
  return __gthread_active_ptr != 0;
}

# 648 "/usr/include/c++/4.5/i686-linux-gnu/bits/gthr-default.h" 3
static inline int
__gthread_create (__gthread_t *__threadid, void *(*__func) (void*),
    void *__args)
{
  return __gthrw_pthread_create (__threadid, __null, __func, __args);
}



  static inline void
  __atomic_add_single(_Atomic_word* __mem, int __val)
  { *__mem += __val; }

  static inline _Atomic_word
  __attribute__ ((__unused__))
  __exchange_and_add_dispatch(_Atomic_word* __mem, int __val)
  {

    if (__gthread_active_p())
      return __exchange_and_add(__mem, __val);
    else
      return __exchange_and_add_single(__mem, __val);



  }

