# 1 "/usr/include/OGRE/OgreScriptLoader.h" 1
# 36 "/usr/include/OGRE/OgreScriptLoader.h"
namespace Ogre {
# 53 "/usr/include/OGRE/OgreScriptLoader.h"
 class ScriptLoader
 {
 public:
  virtual ~ScriptLoader();
# 65 "/usr/include/OGRE/OgreScriptLoader.h"
  virtual const StringVector& getScriptPatterns(void) const = 0;






  virtual void parseScript(DataStreamPtr& stream, const String& groupName) = 0;
# 81 "/usr/include/OGRE/OgreScriptLoader.h"
  virtual Real getLoadingOrder(void) const = 0;

 };


}
# 41 "/usr/include/OGRE/OgreResourceManager.h" 2

namespace Ogre {
# 68 "/usr/include/OGRE/OgreResourceManager.h"
 class ResourceManager : public ScriptLoader, public ResourceAlloc
    {
    public:
 
        ResourceManager();
        virtual ~ResourceManager();
# 94 "/usr/include/OGRE/OgreResourceManager.h"
        virtual ResourcePtr create(const String& name, const String& group,
            bool isManual = false, ManualResourceLoader* loader = 0,
            const NameValuePairList* createParams = 0);

  typedef std::pair<ResourcePtr, bool> ResourceCreateOrRetrieveResult;
# 112 "/usr/include/OGRE/OgreResourceManager.h"
  virtual ResourceCreateOrRetrieveResult createOrRetrieve(const String& name,
   const String& group, bool isManual = false,
   ManualResourceLoader* loader = 0,
   const NameValuePairList* createParams = 0);
# 124 "/usr/include/OGRE/OgreResourceManager.h"
        virtual void setMemoryBudget( size_t bytes);



        virtual size_t getMemoryBudget(void) const;


  virtual size_t getMemoryUsage(void) const { return mMemoryUsage; }







  virtual void unload(const String& name);







  virtual void unload(ResourceHandle handle);
# 161 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void unloadAll(bool reloadableOnly = true);
# 174 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void reloadAll(bool reloadableOnly = true);
# 190 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void unloadUnreferencedResources(bool reloadableOnly = true);
# 205 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void reloadUnreferencedResources(bool reloadableOnly = true);
# 224 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void remove(ResourcePtr& r);
# 243 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void remove(const String& name);
# 262 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void remove(ResourceHandle handle);
# 277 "/usr/include/OGRE/OgreResourceManager.h"
        virtual void removeAll(void);



        virtual ResourcePtr getByName(const String& name);


        virtual ResourcePtr getByHandle(ResourceHandle handle);


  virtual bool resourceExists(const String& name)
  {
   return !getByName(name).isNull();
  }

  virtual bool resourceExists(ResourceHandle handle)
  {
   return !getByHandle(handle).isNull();
  }




  virtual void _notifyResourceTouched(Resource* res);




  virtual void _notifyResourceLoaded(Resource* res);




  virtual void _notifyResourceUnloaded(Resource* res);
# 325 "/usr/include/OGRE/OgreResourceManager.h"
  virtual ResourcePtr prepare(const String& name,
            const String& group, bool isManual = false,
   ManualResourceLoader* loader = 0, const NameValuePairList* loadParams = 0);
# 342 "/usr/include/OGRE/OgreResourceManager.h"
  virtual ResourcePtr load(const String& name,
            const String& group, bool isManual = false,
   ManualResourceLoader* loader = 0, const NameValuePairList* loadParams = 0);
# 361 "/usr/include/OGRE/OgreResourceManager.h"
  virtual const StringVector& getScriptPatterns(void) const { return mScriptPatterns; }
# 376 "/usr/include/OGRE/OgreResourceManager.h"
  virtual void parseScript(DataStreamPtr& stream, const String& groupName) {}







  virtual Real getLoadingOrder(void) const { return mLoadOrder; }


  const String& getResourceType(void) const { return mResourceType; }


        virtual void setVerbose(bool v) { mVerbose = v; }


        virtual bool getVerbose(void) { return mVerbose; }

    protected:


        ResourceHandle getNextHandle(void);
# 421 "/usr/include/OGRE/OgreResourceManager.h"
  virtual Resource* createImpl(const String& name, ResourceHandle handle,
   const String& group, bool isManual, ManualResourceLoader* loader,
            const NameValuePairList* createParams) = 0;

  virtual void addImpl( ResourcePtr& res );

  virtual void removeImpl( ResourcePtr& res );


  virtual void checkUsage(void);


    public:
  typedef ::std::tr1::unordered_map< String, ResourcePtr > ResourceMap;
  typedef std::map<ResourceHandle, ResourcePtr> ResourceHandleMap;
    protected:
        ResourceHandleMap mResourcesByHandle;
        ResourceMap mResources;
        ResourceHandle mNextHandle;
        size_t mMemoryBudget;
        size_t mMemoryUsage;

        bool mVerbose;




  StringVector mScriptPatterns;

  Real mLoadOrder;

  String mResourceType;

    public:
        typedef MapIterator<ResourceHandleMap> ResourceMapIterator;




        ResourceMapIterator getResourceIterator(void)
        {
            return ResourceMapIterator(mResourcesByHandle.begin(), mResourcesByHandle.end());
        }



    };

}
