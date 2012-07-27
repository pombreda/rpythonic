namespace Ogre {
# 73 "/usr/local/include/OGRE/OgreHardwareBuffer.h"
 class HardwareBuffer : public BufferAlloc
    {

     public:

      enum Usage
      {
                HBU_STATIC = 1,
                HBU_DYNAMIC = 2,
                HBU_WRITE_ONLY = 4,
# 105 "/usr/local/include/OGRE/OgreHardwareBuffer.h"
                HBU_DISCARDABLE = 8,
                HBU_STATIC_WRITE_ONLY = 5,
                HBU_DYNAMIC_WRITE_ONLY = 6,
                HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE = 14
      };

      enum LockOptions
      {
       HBL_NORMAL,
       HBL_DISCARD,
       HBL_READ_ONLY,
       HBL_NO_OVERWRITE
      };

  };

class HardwareBuffer::ExternalDefinedSubclass {
	public:
		void somefunction(void);
		void* method_with_unnamed_arg( int* );
}

}

namespace Ogre {
# 88 "/usr/local/include/OGRE/OgreMesh.h"
    struct MeshLodUsage;
    class LodStrategy;

    class Mesh: public Resource
    {

        enum WorldFragmentType {
            WFT_NONE,
            WFT_PLANE_BOUNDED_REGION,
            WFT_SINGLE_INTERSECTION,
            WFT_CUSTOM_GEOMETRY,
            WFT_RENDER_OPERATION
        };


        HardwareBuffer::Usage mVertexBufferUsage;
        HardwareBuffer::Usage mIndexBufferUsage;
        HardwareBuffer::Usage getVertexBufferUsage(void) __const__ { return mVertexBufferUsage; }

# 168 "/usr/local/include/OGRE/OgreSceneQuery.h"
        virtual void setWorldFragmentType(enum WorldFragmentType wft);

	int16 test_stdint_swap( uint8 myint );

	virtual Real getSquaredViewDepth(__const__ Camera* cam) __const__ = 0;

	/// unusal case: struct has a constructor
	/// Rule controlling whether technique is deemed supported based on GPU vendor
	struct GPUVendorRule
	{
		GPUVendor vendor;
		IncludeOrExclude includeOrExclude;
		GPUVendorRule()
		: vendor(GPU_UNKNOWN), includeOrExclude(EXCLUDE) {}
		GPUVendorRule(GPUVendor v, IncludeOrExclude ie)
		: vendor(v), includeOrExclude(ie) {}
	};

	void method_takes_arrays( int a[3], SomeObject b[12] );

	inline Real operator () ( __const__ size_t i ) __const__ {};
	inline Real operator [] ( __const__ size_t i ) __const__ {};
	inline Real operator == ( __const__ size_t i ) __const__ {};
	inline Real operator >= ( __const__ size_t i ) __const__ {};
	inline Real operator > ( __const__ size_t i ) __const__ {};
	inline Real operator >> ( __const__ size_t i ) __const__ {};


    };
}




