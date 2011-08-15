namespace Ogre
{
# 436 "/usr/local/include/OGRE/OgrePrerequisites.h"
  typedef std::string _StringBase;





  typedef std::basic_stringstream<char,std::char_traits<char>,std::allocator<char> > _StringStreamBase;




 typedef _StringBase String;
 typedef _StringStreamBase StringStream;
 typedef StringStream stringstream;

}

# 37 "/usr/local/include/OGRE/OgreOverlayManager.h" 2

namespace Ogre {

 /** \addtogroup Core
	*  @{
	*/
 /** \addtogroup Overlays
	*  @{
	*/
 /** Manages Overlay objects, parsing them from .overlay files and
        storing a lookup library of them. Alo manages the creation of 
		OverlayContainers and OverlayElements, used for non-interactive 2D 
		elements such as HUDs.
    */
    class OverlayManager : public Singleton<OverlayManager>, public ScriptLoader, public OverlayAlloc
    {
    public:
        typedef map<String, Overlay*>::type OverlayMap;
  typedef map<String, OverlayElement*>::type ElementMap;
  typedef map<String, OverlayElementFactory*>::type FactoryMap;
    protected:
        OverlayMap mOverlayMap;
        StringVector mScriptPatterns;

        void parseNewElement( DataStreamPtr& chunk, String& elemType, String& elemName,
            bool isContainer, Overlay* pOverlay, bool isTemplate, String templateName = String(""), OverlayContainer* container = 0);
        void parseAttrib( __const__ String& line, Overlay* pOverlay);
        void parseElementAttrib( __const__ String& line, Overlay* pOverlay, OverlayElement* pElement );
        void skipToNextCloseBrace(DataStreamPtr& chunk);
        void skipToNextOpenBrace(DataStreamPtr& chunk);

        int mLastViewportWidth, mLastViewportHeight;
        bool mViewportDimensionsChanged;
        OrientationMode mLastViewportOrientationMode;

     bool parseChildren( DataStreamPtr& chunk, __const__ String& line,
            Overlay* pOverlay, bool isTemplate, OverlayContainer* parent = __null);

  FactoryMap mFactories;

  ElementMap mInstances;
  ElementMap mTemplates;

  typedef set<String>::type LoadedScripts;
  LoadedScripts mLoadedScripts;




  ElementMap& getElementMap(bool isTemplate);

  OverlayElement* createOverlayElementImpl(__const__ String& typeName, __const__ String& instanceName, ElementMap& elementMap);

  OverlayElement* getOverlayElementImpl(__const__ String& name, ElementMap& elementMap);

  bool hasOverlayElementImpl(__const__ String& name, ElementMap& elementMap);

  void destroyOverlayElementImpl(__const__ String& instanceName, ElementMap& elementMap);

  void destroyOverlayElementImpl(OverlayElement* pInstance, ElementMap& elementMap);

  void destroyAllOverlayElementsImpl(ElementMap& elementMap);

    public:
        OverlayManager();
        virtual ~OverlayManager();

        /// @copydoc ScriptLoader::getScriptPatterns
        __const__ StringVector& getScriptPatterns(void) __const__;
        /// @copydoc ScriptLoader::parseScript
        void parseScript(DataStreamPtr& stream, __const__ String& groupName);
        /// @copydoc ScriptLoader::getLoadingOrder
        Real getLoadingOrder(void) __const__;

        /** Create a new Overlay. */
        Overlay* create(__const__ String& name);
        /** Retrieve an Overlay by name 
        @returns A pointer to the Overlay, or 0 if not found
        */
        Overlay* getByName(__const__ String& name);
        /** Destroys an existing overlay by name */
        void destroy(__const__ String& name);
        /** Destroys an existing overlay */
        void destroy(Overlay* overlay);
        /** Destroys all existing overlays */
        void destroyAll(void);
        typedef MapIterator<OverlayMap> OverlayMapIterator;
        OverlayMapIterator getOverlayIterator(void);

        /** Internal method for queueing the visible overlays for rendering. */
        void _queueOverlaysForRendering(Camera* cam, RenderQueue* pQueue, Viewport *vp);

        /** Method for determining if the viewport has changed dimensions. 
        @remarks This is used by pixel-based OverlayElements to work out if they need to
            recalculate their sizes.
        */
        bool hasViewportChanged(void) __const__;

        /** Gets the height of the destination viewport in pixels. */
        int getViewportHeight(void) __const__;

        /** Gets the width of the destination viewport in pixels. */
        int getViewportWidth(void) __const__;
        Real getViewportAspectRatio(void) __const__;

        /** Gets the orientation mode of the destination viewport. */
        OrientationMode getViewportOrientationMode(void) __const__;

  /** Creates a new OverlayElement of the type requested.
		@remarks
		The type of element to create is passed in as a string because this
		allows plugins to register new types of component.
		@param typeName The type of element to create.
		@param instanceName The name to give the new instance.
		*/
  OverlayElement* createOverlayElement(__const__ String& typeName, __const__ String& instanceName, bool isTemplate = false);

  /** Gets a reference to an existing element. */
  OverlayElement* getOverlayElement(__const__ String& name, bool isTemplate = false);

  /** Tests if an element exists. */
  bool hasOverlayElement(__const__ String& name, bool isTemplate = false);

  /** Destroys a OverlayElement. 
		@remarks
		Make sure you're not still using this in an Overlay. If in
		doubt, let OGRE destroy elements on shutdown.
		*/
  void destroyOverlayElement(__const__ String& instanceName, bool isTemplate = false);

  /** Destroys a OverlayElement. 
		@remarks
		Make sure you're not still using this in an Overlay. If in
		doubt, let OGRE destroy elements on shutdown.
		*/
  void destroyOverlayElement(OverlayElement* pInstance, bool isTemplate = false);

  /** Destroys all the OverlayElement  created so far.
		@remarks
		Best to leave this to the engine to call internally, there
		should rarely be a need to call it yourself.
		*/
  void destroyAllOverlayElements(bool isTemplate = false);

  /** Registers a new OverlayElementFactory with this manager.
		@remarks
		Should be used by plugins or other apps wishing to provide
		a new OverlayElement subclass.
		*/
  void addOverlayElementFactory(OverlayElementFactory* elemFactory);

  /** Get const access to the list of registered OverlayElement factories. */
  __const__ FactoryMap& getOverlayElementFactoryMap() __const__ {
   return mFactories;
  }

  OverlayElement* createOverlayElementFromTemplate(__const__ String& templateName, __const__ String& typeName, __const__ String& instanceName, bool isTemplate = false);
  /**
		*  @remarks
		*  Creates a new OverlayElement object from the specified template name.  The new
		*  object's name, and all of it's children, will be instanceName/orignalName.
		*/
  OverlayElement* cloneOverlayElementFromTemplate(__const__ String& templateName, __const__ String& instanceName);

  OverlayElement* createOverlayElementFromFactory(__const__ String& typeName, __const__ String& instanceName);

  typedef MapIterator<ElementMap> TemplateIterator;
  /** Returns an iterator over all templates in this manager.*/
  TemplateIterator getTemplateIterator ()
  {
   return TemplateIterator (mTemplates.begin (), mTemplates.end ()) ;
  }
  /* Returns whether the Element with the given name is a Template */
  bool isTemplate (String strName) __const__ {
   return (mTemplates.find (strName) != mTemplates.end()) ;
  }


        /** Override standard Singleton retrieval.
        @remarks
        Why do we do this? Well, it's because the Singleton
        implementation is in a .h file, which means it gets compiled
        into anybody who includes it. This is needed for the
        Singleton template to work, but we actually only want it
        compiled into the implementation of the class based on the
        Singleton, not all of them. If we don't change this, we get
        link errors when trying to use the Singleton-based class from
        an outside dll.
        @par
        This method just delegates to the template version anyway,
        but the implementation stays in this single compilation unit,
        preventing link errors.
        */
        static OverlayManager& getSingleton(void);
        /** Override standard Singleton retrieval.
        @remarks
        Why do we do this? Well, it's because the Singleton
        implementation is in a .h file, which means it gets compiled
        into anybody who includes it. This is needed for the
        Singleton template to work, but we actually only want it
        compiled into the implementation of the class based on the
        Singleton, not all of them. If we don't change this, we get
        link errors when trying to use the Singleton-based class from
        an outside dll.
        @par
        This method just delegates to the template version anyway,
        but the implementation stays in this single compilation unit,
        preventing link errors.
        */
        static OverlayManager* getSingletonPtr(void);
    };


 /** @} */
 /** @} */

}

