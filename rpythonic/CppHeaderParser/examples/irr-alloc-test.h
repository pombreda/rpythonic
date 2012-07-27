# 12 "/usr/local/include/irrlicht/irrAllocator.h" 2

namespace irr
{
namespace core
{


//! Very simple allocator implementation, containers using it can be used across dll boundaries
template<typename T>
class irrAllocator
{
public:

 //! Destructor
 virtual ~irrAllocator() {}

 //! Allocate memory for an array of objects
 T* allocate(size_t cnt)
 {
  return (T*)internal_new(cnt* sizeof(T));
 }

 //! Deallocate memory for an array of objects
 void deallocate(T* ptr)
 {
  internal_delete(ptr);
 }

 //! Construct an element
 void construct(T* ptr, __const__ T&e)
 {
  new ((void*)ptr) T(e);
 }

 //! Destruct an element
 void destruct(T* ptr)
 {
  ptr->~T();
 }

protected:

 virtual void* internal_new(size_t cnt)
 {
  return operator new(cnt);
 }

 virtual void internal_delete(void* ptr)
 {
  operator delete(ptr);
 }

};


