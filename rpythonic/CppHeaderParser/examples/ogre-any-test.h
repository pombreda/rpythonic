namespace Ogre
{
# 436 "/usr/local/include/OGRE/OgrePrerequisites.h"
  typedef std::string _StringBase;





  typedef std::basic_stringstream<char,std::char_traits<char>,std::allocator<char> > _StringStreamBase;




 typedef _StringBase String;
 typedef _StringStreamBase StringStream;
 typedef StringStream stringstream;

}

# 44 "/usr/local/include/OGRE/OgreAny.h" 2


namespace Ogre
{
 /** \addtogroup Core
	*  @{
	*/
 /** \addtogroup General
	*  @{
	*/
 /** Variant type that can hold Any other type.
	*/
 class Any
    {
    public: // constructors

        Any()
          : mContent(0)
        {
        }

        template<typename ValueType>
        explicit Any(__const__ ValueType & value)
          : mContent(new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(holder<ValueType>))) holder<ValueType>(value))
        {
        }

        Any(__const__ Any & other)
          : mContent(other.mContent ? other.mContent->clone() : 0)
        {
        }

        virtual ~Any()
        {
            destroy();
        }

    public: // modifiers

        Any& swap(Any & rhs)
        {
            std::swap(mContent, rhs.mContent);
            return *this;
        }

        template<typename ValueType>
        Any& operator=(__const__ ValueType & rhs)
        {
            Any(rhs).swap(*this);
            return *this;
        }

        Any & operator=(__const__ Any & rhs)
        {
            Any(rhs).swap(*this);
            return *this;
        }

    public: // queries

        bool isEmpty() __const__
        {
            return !mContent;
        }

        __const__ std::type_info& getType() __const__
        {
            return mContent ? mContent->getType() : typeid(void);
        }

  inline friend std::ostream& operator <<
   ( std::ostream& o, __const__ Any& v )
  {
   if (v.mContent)
    v.mContent->writeToStream(o);
   return o;
  }

  void destroy()
  {
   if(mContent){(mContent)->~placeholder(); ::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::deallocateBytes((void*)mContent);};
   mContent = __null;
  }

    protected: // types

  class placeholder
        {
        public: // structors

            virtual ~placeholder()
            {
            }

        public: // queries

            virtual __const__ std::type_info& getType() __const__ = 0;

            virtual placeholder * clone() __const__ = 0;

   virtual void writeToStream(std::ostream& o) = 0;

        };

        template<typename ValueType>
        class holder : public placeholder
        {
        public: // structors

            holder(__const__ ValueType & value)
              : held(value)
            {
            }

        public: // queries

            virtual __const__ std::type_info & getType() __const__
            {
                return typeid(ValueType);
            }

            virtual placeholder * clone() __const__
            {
                return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(holder))) holder(held);
            }

   virtual void writeToStream(std::ostream& o)
   {
    o << held;
   }


        public: // representation

            ValueType held;

        };



    protected: // representation
        placeholder * mContent;

        template<typename ValueType>
        friend ValueType * any_cast(Any *);


    public:

     template<typename ValueType>
     ValueType operator()() __const__
     {
   if (!mContent)
   {
    throw Ogre::ExceptionFactory::create( Ogre::ExceptionCodeType<Exception::ERR_INVALIDPARAMS>(), "Bad cast from uninitialised Any", "Any::operator()",

 "/usr/local/include/OGRE/OgreAny.h"
# 198 "/usr/local/include/OGRE/OgreAny.h"
    ,

 200
# 198 "/usr/local/include/OGRE/OgreAny.h"
    );

                       ;
   }
   else if(getType() == typeid(ValueType))
   {
              return static_cast<Any::holder<ValueType> *>(mContent)->held;
   }
   else
   {
    StringUtil::StrStreamType str;
    str << "Bad cast from type '" << getType().name() << "' "
     << "to '" << typeid(ValueType).name() << "'";
    throw Ogre::ExceptionFactory::create( Ogre::ExceptionCodeType<Exception::ERR_INVALIDPARAMS>(), str.str(), "Any::operator()",

 "/usr/local/include/OGRE/OgreAny.h"
# 211 "/usr/local/include/OGRE/OgreAny.h"
    ,

 213
# 211 "/usr/local/include/OGRE/OgreAny.h"
    );

                       ;
   }
  }



    };      // END CLASS: Any
///////////////////////////////////////////////////////////////////////////



 /** Specialised Any class which has built in arithmetic operators, but can 
		hold only types which support operator +,-,* and / .
	*/
 class AnyNumeric : public Any
 {
 public:
  AnyNumeric()
  : Any()
  {
  }

  template<typename ValueType>
  AnyNumeric(__const__ ValueType & value)

  {
   mContent = new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder<ValueType>))) numholder<ValueType>(value);
  }

  AnyNumeric(__const__ AnyNumeric & other)
            : Any()
  {
   mContent = other.mContent ? other.mContent->clone() : 0;
  }

 protected:
  class numplaceholder : public Any::placeholder
  {
  public: // structors

   ~numplaceholder()
   {
   }
   virtual placeholder* add(placeholder* rhs) = 0;
   virtual placeholder* subtract(placeholder* rhs) = 0;
   virtual placeholder* multiply(placeholder* rhs) = 0;
   virtual placeholder* multiply(Real factor) = 0;
   virtual placeholder* divide(placeholder* rhs) = 0;
  };

  template<typename ValueType>
  class numholder : public numplaceholder
  {
  public: // structors

   numholder(__const__ ValueType & value)
    : held(value)
   {
   }

  public: // queries

   virtual __const__ std::type_info & getType() __const__
   {
    return typeid(ValueType);
   }

   virtual placeholder * clone() __const__
   {
    return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder))) numholder(held);
   }

   virtual placeholder* add(placeholder* rhs)
   {
    return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder))) numholder(held + static_cast<numholder*>(rhs)->held);
   }
   virtual placeholder* subtract(placeholder* rhs)
   {
    return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder))) numholder(held - static_cast<numholder*>(rhs)->held);
   }
   virtual placeholder* multiply(placeholder* rhs)
   {
    return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder))) numholder(held * static_cast<numholder*>(rhs)->held);
   }
   virtual placeholder* multiply(Real factor)
   {
    return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder))) numholder(held * factor);
   }
   virtual placeholder* divide(placeholder* rhs)
   {
    return new (::Ogre::CategorisedAllocPolicy<MEMCATEGORY_GENERAL>::allocateBytes(sizeof(numholder))) numholder(held / static_cast<numholder*>(rhs)->held);
   }
   virtual void writeToStream(std::ostream& o)
   {
    o << held;
   }

  public: // representation

   ValueType held;

  };

  /// Construct from holder
  AnyNumeric(placeholder* pholder)
  {
   mContent = pholder;
  }

 public:
  AnyNumeric & operator=(__const__ AnyNumeric & rhs)
  {
   AnyNumeric(rhs).swap(*this);
   return *this;
  }
  AnyNumeric operator+(__const__ AnyNumeric& rhs) __const__
  {
   return AnyNumeric(
    static_cast<numplaceholder*>(mContent)->add(rhs.mContent));
  }
  AnyNumeric operator-(__const__ AnyNumeric& rhs) __const__
  {
   return AnyNumeric(
    static_cast<numplaceholder*>(mContent)->subtract(rhs.mContent));
  }
  AnyNumeric operator*(__const__ AnyNumeric& rhs) __const__
  {
   return AnyNumeric(
    static_cast<numplaceholder*>(mContent)->multiply(rhs.mContent));
  }
  AnyNumeric operator*(Real factor) __const__
  {
   return AnyNumeric(
    static_cast<numplaceholder*>(mContent)->multiply(factor));
  }
  AnyNumeric operator/(__const__ AnyNumeric& rhs) __const__
  {
   return AnyNumeric(
    static_cast<numplaceholder*>(mContent)->divide(rhs.mContent));
  }
  AnyNumeric& operator+=(__const__ AnyNumeric& rhs)
  {
   *this = AnyNumeric(
    static_cast<numplaceholder*>(mContent)->add(rhs.mContent));
   return *this;
  }
  AnyNumeric& operator-=(__const__ AnyNumeric& rhs)
  {
   *this = AnyNumeric(
    static_cast<numplaceholder*>(mContent)->subtract(rhs.mContent));
   return *this;
  }
  AnyNumeric& operator*=(__const__ AnyNumeric& rhs)
  {
   *this = AnyNumeric(
    static_cast<numplaceholder*>(mContent)->multiply(rhs.mContent));
   return *this;
  }
  AnyNumeric& operator/=(__const__ AnyNumeric& rhs)
  {
   *this = AnyNumeric(
    static_cast<numplaceholder*>(mContent)->divide(rhs.mContent));
   return *this;
  }




 };     // END CLASS: AnyNumeric


    template<typename ValueType>
    ValueType * any_cast(Any * operand)
    {
        return operand && operand->getType() == typeid(ValueType)
                    ? &static_cast<Any::holder<ValueType> *>(operand->mContent)->held
                    : 0;
    }

    template<typename ValueType>
    __const__ ValueType * any_cast(__const__ Any * operand)
    {
        return any_cast<ValueType>(const_cast<Any *>(operand));
    }

    template<typename ValueType>
    ValueType any_cast(__const__ Any & operand)
    {
        __const__ ValueType * result = any_cast<ValueType>(&operand);
        if(!result)
  {
   StringUtil::StrStreamType str;
   str << "Bad cast from type '" << operand.getType().name() << "' "
    << "to '" << typeid(ValueType).name() << "'";
   throw Ogre::ExceptionFactory::create( Ogre::ExceptionCodeType<Exception::ERR_INVALIDPARAMS>(), str.str(), "Ogre::any_cast",

 "/usr/local/include/OGRE/OgreAny.h"
# 405 "/usr/local/include/OGRE/OgreAny.h"
   ,

 407
# 405 "/usr/local/include/OGRE/OgreAny.h"
   );

                     ;
  }
        return *result;
    }
 /** @} */
 /** @} */

void mystringfunction( String );

}

