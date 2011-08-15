# 25 "/usr/local/include/G3D/Map2D.h" 2 3

namespace G3D {

namespace _internal {

/** The default compute type for a type is the type itself. */
template<typename Storage> class _GetComputeType {
public:
    typedef Storage Type;
};

} // _internal
} // G3D

// This weird syntax is needed to support VC6, which doesn't
// properly implement template overloading.
# 50 "/usr/local/include/G3D/Map2D.h" 3
namespace G3D { namespace _internal { template<> class _GetComputeType < float32 > { public: typedef float64 Type; }; } }


namespace G3D {

/**
  Map of values across a discrete 2D plane.  Can be thought of as a generic class for 2D images, 
  allowing flexibility as to pixel format and convenient methods.
  In fact, the "pixels" can be any values
  on a grid that can be sensibly interpolated--RGB colors, scalars, 4D vectors, and so on.

  Other "image" classes in G3D:

  G3D::GImage - Supports file formats, fast, Color3uint8 and Color4uint8 formats.  No interpolation.

  G3D::Texture::Ref - Represents image on the graphics card (not directly readable on the CPU).  Supports 2D, 3D, and a variety of interpolation methods, loads file formats.

  G3D::Image3 - A subclass of Map2D<Color3> that supports image loading and saving and conversion to Texture.

  G3D::Image4 - A subclass of Map2D<Color4> that supports image loading and saving and conversion to Texture.

  G3D::Image3uint8 - A subclass of Map2D<Color3uint8> that supports image loading and saving and conversion to Texture.

  G3D::Image4uint8 -  A subclass of Map2D<Color4uint8> that supports image loading and saving and conversion to Texture.

  There are two type parameters-- the first (@ Storage) is the type 
  used to store the "pixel" values efficiently and 
  the second (@a Compute) is
  the type operated on by computation.  The Compute::Compute(Storage&) constructor
  is used to convert between storage and computation types.
  @a Storage is often an integer version of @a Compute, for example 
  <code>Map2D<double, uint8></code>.  By default, the computation type is:

  <pre>
     Storage       Computation

     uint8          float32
     uint16         float32
     uint32         float64
     uint64         float64

     int8           float32
     int16          float32
     int32          float64
     int64          float64

     float32        float64
     float64        float64

     Vector2        Vector2
     Vector2int16   Vector2

     Vector3        Vector3
     Vector3int16   Vector3

     Vector4        Vector4

     Color3         Color3
     Color3uint8    Color3

     Color4         Color4
     Color4uint8    Color4
    </pre>
  Any other storage type defaults to itself as the computation type.

  The computation type can be any that 
  supports lerp, +, -, *, /, and an empty constructor.

  Assign value:

    <code>im->set(x, y, 7);</code> or 
    <code>im->get(x, y) = 7;</code>

  Read value:

    <code>int c = im(x, y);</code>

  Can also sample with nearest neighbor, bilinear, and bicubic
  interpolation.  
  
  Sampling follows OpenGL conventions, where 
  pixel values represent grid points and (0.5, 0.5) is half-way
  between two vertical and two horizontal grid points.  
  To draw an image of dimensions w x h with nearest neighbor
  sampling, render pixels from [0, 0] to [w - 1, h - 1].

  Under the WrapMode::CLAMP wrap mode, the value of bilinear interpolation
  becomes constant outside [1, w - 2] horizontally.  Nearest neighbor
  interpolation is constant outside [0, w - 1] and bicubic outside
  [3, w - 4].  The class does not offer quadratic interpolation because
  the interpolation filter could not center over a pixel.
  
  @author Morgan McGuire, http://graphics.cs.williams.edu
 */
template< typename Storage,
typename Compute = typename G3D::_internal::_GetComputeType<Storage>::Type>
class Map2D : public ReferenceCountedObject {

//
// It doesn't make sense to automatically convert from Compute back to Storage
// because the rounding rule (and scaling) is application dependent.
// Thus the interpolation methods all return type Compute.
//

public:

    typedef Storage StorageType;
    typedef Compute ComputeType;
    typedef Map2D<Storage, Compute> Type;
    typedef ReferenceCountedPointer<Map2D> Ref;

protected:

    Storage ZERO;

    /** Width, in pixels. */
    uint32 w;

    /** Height, in pixels. */
    uint32 h;

    WrapMode _wrapMode;

    /** 0 if no mutating method has been invoked 
        since the last call to setChanged(); */
    AtomicInt32 m_changed;

    Array<Storage> data;

    /** Handles the exceptional cases from get */
    __const__ Storage& slowGet(int x, int y, WrapMode wrap) {
        switch (wrap) {
        case WrapMode::CLAMP:
            return fastGet(iClamp(x, 0, w - 1), iClamp(y, 0, h - 1));

        case WrapMode::TILE:
            return fastGet(iWrap(x, w), iWrap(y, h));

        case WrapMode::ZERO:
            return ZERO;

        case WrapMode::ERROR:
            { if (!(((uint32)x < w) && ((uint32)y < h))) { G3D::_internal::_releaseInputGrab_(); if ((G3D::_internal::_failureHook != __null) && G3D::_internal::_failureHook("((uint32)x < w) && ((uint32)y < h)", format("Index out of bounds: (%d, %d), w = %d, h = %d", x, y, w, h),

 "/usr/local/include/G3D/Map2D.h"
# 217 "/usr/local/include/G3D/Map2D.h" 3
            ,

 219
# 217 "/usr/local/include/G3D/Map2D.h" 3
            , true)) { ::exit(-1); } G3D::_internal::_restoreInputGrab_(); } }

                            ;

            // intentionally fall through
        case WrapMode::IGNORE:
            // intentionally fall through
        default:
            {
                static Storage temp;
                return temp;
            }
        }
    }

public:

    /** Unsafe access to the underlying data structure with no wrapping support; requires that (x, y) is in bounds. */
    __inline__ __const__ Storage& fastGet(int x, int y) __const__ {
        do {} while (0);
        return data[x + y * w];
    }

    /** Unsafe access to the underlying data structure with no wrapping support; requires that (x, y) is in bounds. */
    __inline__ void fastSet(int x, int y, __const__ Storage& v) {
        do {} while (0);
        data[x + y * w] = v;
    }

protected:

    /** Given four control points and a value on the range [0, 1)
        evaluates the Catmull-rom spline between the times of the
        middle two control points */
    Compute bicubic(__const__ Compute* ctrl, double s) __const__ {

        // f = B * S * ctrl'

        // B matrix: Catmull-Rom spline basis
        static __const__ double B[4][4] = {
            { 0.0, -0.5, 1.0, -0.5},
            { 1.0, 0.0, -2.5, 1.5},
            { 0.0, 0.5, 2.0, -1.5},
            { 0.0, 0.0, -0.5, 0.5}};

        // S: Powers of the fraction
        double S[4];
        double s2 = s * s;
        S[0] = 1.0;
        S[1] = s;
        S[2] = s2;
        S[3] = s2 * s;

        Compute sum(ZERO);

        for (int c = 0; c < 4; ++c) {
            double coeff = 0.0;
            for (int power = 0; power < 4; ++power) {
                coeff += B[c][power] * S[power];
            }
            sum += ctrl[c] * coeff;
        }

        return sum;
    }


    Map2D(int w, int h, WrapMode wrap) : w(0), h(0), _wrapMode(wrap), m_changed(1) {
        ZERO = Storage(Compute(Storage()) * 0);
        resize(w, h);
    }

public:

    /**
     Although Map2D is not threadsafe (except for the setChanged() method),
     you can use this mutex to create your own threadsafe access to a Map2D.
     Not used by the default implementation.
    */
    GMutex mutex;

    static Ref create(int w = 0, int h = 0, WrapMode wrap = WrapMode::ERROR) {
        return new Map2D(w, h, wrap);
    }

    /** Resizes without clearing, leaving garbage.
      */
    void resize(uint32 newW, uint32 newH) {
        if ((newW != w) || (newH != h)) {
            w = newW;
            h = newH;
            data.resize(w * h);
            setChanged(true);
        }
    }

    /** 
     Returns true if this map has been written to since the last call to setChanged(false).
     This is useful if you are caching a texture map other value that must be recomputed
     whenever this changes.
     */
    bool changed() {
        return m_changed.value() != 0;
    }

    /** Set/unset the changed flag. */
    void setChanged(bool c) {
        m_changed = c ? 1 : 0;
    }

    /** Returns a pointer to the underlying row-major data. There is no padding at the end of the row.
        Be careful--this will be reallocated during a resize.  You should call setChanged(true) if you mutate the array.*/
    Storage* getCArray() {
        return data.getCArray();
    }


    __const__ Storage* getCArray() __const__ {
        return data.getCArray();
    }


    /** Row-major array.  You should call setChanged(true) if you mutate the array. */
    Array<Storage>& getArray() {
        return data;
    }


    __const__ Array<Storage>& getArray() __const__ {
        return data;
    }

    /** is (x, y) strictly within the image bounds, or will it trigger some kind of wrap mode */
    __inline__ bool inBounds(int x, int y) __const__ {
        return (((uint32)x < w) && ((uint32)y < h));
    }

    /** is (x, y) strictly within the image bounds, or will it trigger some kind of wrap mode */
    __inline__ bool inBounds(__const__ Vector2int16& v) __const__ {
        return inBounds(v.x, v.y);
    }

    /** Get the value at (x, y).
    
        Note that the type of image->get(x, y) is 
        the storage type, not the computation
        type.  If the constructor promoting Storage to Compute rescales values
        (as, for example Color3(Color3uint8&) does), this will not match the value
        returned by Map2D::nearest.
      */
    __inline__ __const__ Storage& get(int x, int y, WrapMode wrap) __const__ {
        if (((uint32)x < w) && ((uint32)y < h)) {
            return data[x + y * w];
        } else {
            // Remove the const to allow a slowGet on this object
            // (we're returning a const reference so this is ok)
            return const_cast<Type*>(this)->slowGet(x, y, wrap);
        }

            // gcc gives a useless warning that the above code might reach the end of the function;
            // we use this line to supress the warning.
            return ZERO;

    }

    __inline__ __const__ Storage& get(int x, int y) __const__ {
        return get(x, y, _wrapMode);
    }

    __inline__ __const__ Storage& get(__const__ Vector2int16& p) __const__ {
        return get(p.x, p.y, _wrapMode);
    }

    __inline__ __const__ Storage& get(__const__ Vector2int16& p, WrapMode wrap) __const__ {
        return get(p.x, p.y, wrap);
    }

    __inline__ Storage& get(int x, int y, WrapMode wrap) {
        return const_cast<Storage&>(const_cast<__const__ Type*>(this)->get(x, y, wrap));

            // gcc gives a useless warning that the above code might reach the end of the function;
            // we use this line to supress the warning.
            return ZERO;

    }

    __inline__ Storage& get(int x, int y) {
        return const_cast<Storage&>(const_cast<__const__ Type*>(this)->get(x, y));

            // gcc gives a useless warning that the above code might reach the end of the function;
            // we use this line to supress the warning.
            return ZERO;

    }

    __inline__ Storage& get(__const__ Vector2int16& p) {
        return get(p.x, p.y);
    }

    /** Sets the changed flag to true */
    __inline__ void set(__const__ Vector2int16& p, __const__ Storage& v) {
        set(p.x, p.y, v);
    }

    /** Sets the changed flag to true */
    void set(int x, int y, __const__ Storage& v, WrapMode wrap) {
        setChanged(true);
        if (((uint32)x < w) && ((uint32)y < h)) {
            // In bounds, wrapping isn't an issue.
            data[x + y * w] = v;
        } else {
            const_cast<Storage&>(slowGet(x, y, wrap)) = v;
        }
    }

    void set(int x, int y, __const__ Storage& v) {
        set(x, y, v, _wrapMode);
    }


    void setAll(__const__ Storage& v) {
        for(int i = 0; i < data.size(); ++i) {
            data[i] = v;
        }
        setChanged(true);
    }

    /** flips if @a flip is true*/
    void maybeFlipVertical(bool flip) {
        if (flip) {
            flipVertical();
        }
    }

 virtual void flipVertical() {
  int halfHeight = h/2;
  Storage* d = data.getCArray();
  for (int y = 0; y < halfHeight; ++y) {
   int o1 = y * w;
   int o2 = (h - y - 1) * w;
   for (int x = 0; x < (int)w; ++x) {
    int i1 = o1 + x;
    int i2 = o2 + x;
    Storage temp = d[i1];
    d[i1] = d[i2];
    d[i2] = temp;
   }
  }
        setChanged(true);
 }

 virtual void flipHorizontal() {
  int halfWidth = w / 2;
  Storage* d = data.getCArray();
  for (int x = 0; x < halfWidth; ++x) {
   for (int y = 0; y < (int)h; ++y) {
    int i1 = y * w + x;
    int i2 = y * w + (w - x - 1);
    Storage temp = d[i1];
    d[i1] = d[i2];
    d[i2] = temp;
   }
  }
        setChanged(true);
 }

    /**
     Crops this map so that it only contains pixels between (x, y) and (x + w - 1, y + h - 1) inclusive.
     */
    virtual void crop(int newX, int newY, int newW, int newH) {
        { if (!(newX + newW <= (int)w)) { G3D::_internal::_releaseInputGrab_(); if ((G3D::_internal::_failureHook != __null) && G3D::_internal::_failureHook("newX + newW <= (int)w", "Cannot grow when cropping", "/usr/local/include/G3D/Map2D.h", 487, true)) { ::exit(-1); } G3D::_internal::_restoreInputGrab_(); } };
        { if (!(newY + newH <= (int)h)) { G3D::_internal::_releaseInputGrab_(); if ((G3D::_internal::_failureHook != __null) && G3D::_internal::_failureHook("newY + newH <= (int)h", "Cannot grow when cropping", "/usr/local/include/G3D/Map2D.h", 488, true)) { ::exit(-1); } G3D::_internal::_restoreInputGrab_(); } };
        { if (!(newX >= 0 && newY >= 0)) { G3D::_internal::_releaseInputGrab_(); if ((G3D::_internal::_failureHook != __null) && G3D::_internal::_failureHook("newX >= 0 && newY >= 0", "Origin out of bounds.", "/usr/local/include/G3D/Map2D.h", 489, true)) { ::exit(-1); } G3D::_internal::_restoreInputGrab_(); } };

        // Always safe to copy towards the upper left, provided 
        // that we're iterating towards the lower right.  This lets us avoid
        // reallocating the underlying array.
        for (int y = 0; y < newH; ++y) {
            for (int x = 0; x < newW; ++x) {
                data[x + y * newW] = data[(x + newX) + (y + newY) * w];
            }
        }

        resize(newW, newH);
    }

    /** iRounds to the nearest x0 and y0. */
    virtual void crop(__const__ Rect2D& rect) {
        crop(iRound(rect.x0()), iRound(rect.y0()), iRound(rect.x1()) - iRound(rect.x0()), iRound(rect.y1()) - iRound(rect.y0()));
    }

    /** Returns the nearest neighbor.  Pixel values are considered
        to be at the upper left corner, so <code>image->nearest(x, y) == image(x, y)</code>
      */
    __inline__ Compute nearest(float x, float y, WrapMode wrap) __const__ {
        int ix = iRound(x);
        int iy = iRound(y);
        return Compute(get(ix, iy, wrap));
    }

    __inline__ Compute nearest(float x, float y) __const__ {
        return nearest(x, y, _wrapMode);
    }

    __inline__ Compute nearest(__const__ Vector2& p) __const__ {
        return nearest(p.x, p.y);
    }

    /** Returns the average value of all elements of the map */
    Compute average() __const__ {
        if ((w == 0) || (h == 0)) {
            return ZERO;
        }

        // To avoid overflows, compute the average of row averages

        Compute rowSum = ZERO;
        for (unsigned int y = 0; y < h; ++y) {
            Compute sum = ZERO;
            int offset = y * w;
            for (unsigned int x = 0; x < w; ++x) {
                sum += Compute(data[offset + x]);
            }
            rowSum += sum * (1.0f / w);
        }

        return rowSum * (1.0f / h);
    }

    /** 
      Needs to access elements from (floor(x), floor(y))
      to (floor(x) + 1, floor(y) + 1) and will use
      the wrap mode appropriately (possibly generating 
      out of bounds errors).

      Guaranteed to match nearest(x, y) at integers. */
    Compute bilinear(float x, float y, WrapMode wrap) __const__ {
        __const__ int i = iFloor(x);
        __const__ int j = iFloor(y);

        __const__ float fX = x - i;
        __const__ float fY = y - j;

        // Horizontal interpolation, first row
        __const__ Compute& t0 = get(i, j, wrap);
        __const__ Compute& t1 = get(i + 1, j, wrap);

        // Horizontal interpolation, second row
        __const__ Compute& t2 = get(i, j + 1, wrap);
        __const__ Compute& t3 = get(i + 1, j + 1, wrap);

        __const__ Compute& A = lerp(t0, t1, fX);
        __const__ Compute& B = lerp(t2, t3, fX);

        // Vertical interpolation
        return lerp(A, B, fY);
    }

    Compute bilinear(float x, float y) __const__ {
        return bilinear(x, y, _wrapMode);
    }

    __inline__ Compute bilinear(__const__ Vector2& p) __const__ {
        return bilinear(p.x, p.y, _wrapMode);
    }

    __inline__ Compute bilinear(__const__ Vector2& p, WrapMode wrap) __const__ {
        return bilinear(p.x, p.y, wrap);
    }

    /**
     Uses Catmull-Rom splines to interpolate between grid
     values.  Guaranteed to match nearest(x, y) at integers.
     */
    Compute bicubic(float x, float y, WrapMode wrap) __const__ {
        int i = iFloor(x);
        int j = iFloor(y);
        float fX = x - i;
        float fY = y - j;

        Compute vsample[4];
        for (int v = 0; v < 4; ++v) {

            // Horizontal interpolation
            Compute hsample[4];
            for (int u = 0; u < 4; ++u) {
                hsample[u] = Compute(get(i + u - 1, j + v - 1, wrap));
            }

            vsample[v] = bicubic(hsample, fX);
        }

        //  Vertical interpolation
        return bicubic(vsample, fY);
    }

    Compute bicubic(float x, float y) __const__ {
        return bicubic(x, y, _wrapMode);
    }

    __inline__ Compute bicubic(__const__ Vector2& p, WrapMode wrap) __const__ {
        return bicubic(p.x, p.y, wrap);
    }

    __inline__ Compute bicubic(__const__ Vector2& p) __const__ {
        return bicubic(p.x, p.y, _wrapMode);
    }

    /** Pixel width */
    __inline__ int32 width() __const__ {
        return (int32)w;
    }


    /** Pixel height */
    __inline__ int32 height() __const__ {
        return (int32)h;
    }


    /** Dimensions in pixels */
    Vector2int16 size() __const__ {
        return Vector2int16(w, h);
    }

    /** Rectangle from (0, 0) to (w, h) */
    Rect2D rect2DBounds() __const__ {
        return Rect2D::xywh(0, 0, w, h);
    }

    /** Number of bytes occupied by the image data and this structure */
    size_t sizeInMemory() __const__ {
        return data.size() * sizeof(Storage) + sizeof(*this);
    }


    WrapMode wrapMode() __const__ {
        return _wrapMode;
    }


    void setWrapMode(WrapMode m) {
        _wrapMode = m;
    }
};


typedef bool (*somecallback)(size_t size, bool recoverable);


class System {
public:
    /**
       @param size Size of memory that the system was trying to allocate

       @param recoverable If true, the system will attempt to allocate again
       if the callback returns true.  If false, malloc is going to return 
       NULL and this invocation is just to notify the application.

       @return Return true to force malloc to attempt allocation again if the
       error was recoverable.
     */
    typedef bool (*OutOfMemoryCallback)(size_t size, bool recoverable);
};

}

