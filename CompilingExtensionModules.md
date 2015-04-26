#caching CPython extension modules.

# Introduction #

It is easier to migrate code to RPython in stages, and keeping the rest of your code regular Python.  Two decorators let your mark functions and objects to be compiled and cached on demand.  You can set the cache location at the top of your script.  If there is no cached module the first time the script runs, RPythonic will look for GCC and compile a new module and load it at runtime.  If GCC is not found, the program can continue to run as long as your RPython code is not using RFFI.

If you need to distributing pre-compiled modules to clients without GCC.  You can just copy the module from your local cache and include it with your script.  Note that the cached module can be used with all Python versions because it is loaded by ctypes.


# Overview #
  * import rpythonic
  * decorate functions and objects
  * call rpythonic.cache()

## Example1 - Functions ##
![http://4.bp.blogspot.com/-kclx9Uhcbzs/TfHzFFsMhjI/AAAAAAAAAHk/fkTk1UKhCo4/s1600/Screenshot-10.png](http://4.bp.blogspot.com/-kclx9Uhcbzs/TfHzFFsMhjI/AAAAAAAAAHk/fkTk1UKhCo4/s1600/Screenshot-10.png)

```
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
################################
rpy = rpythonic.RPython()

@rpy.bind()			# declare arg types is optional if,
def add( a=1, b=1000 ):		# keyword defaults are given
	return a+b

@rpy.bind(a=float, b=float)
def sub( a, b ):
	return a-b

rpy.cache('test1', refresh=1)	# only compiles if cache is dirty
```

## Example2 - Shared Objects ##
![http://1.bp.blogspot.com/-2FjJyK3v9YM/TfHzFaVMvEI/AAAAAAAAAHs/rXxehIq9T9I/s1600/Screenshot-11.png](http://1.bp.blogspot.com/-2FjJyK3v9YM/TfHzFaVMvEI/AAAAAAAAAHs/rXxehIq9T9I/s1600/Screenshot-11.png)

```
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
rpy = rpythonic.RPython()

@rpy.object
class MyRpyObject(object):
	def __init__(self, x=.0, y=.0, z=.0):
		self.x = x; self.y = y; self.z = z
	def add( self, x=.0,y=.0,z=.0 ): self.x += x; self.y += y; self.z += z
	def sum( self ): return self.x + self.y + self.z
	def show( self ): print '<%s %s %s>' %(self.x, self.y, self.z)

rpy.cache('test2', refresh=1)	

o = MyRpyObject()
print o.x
o.add( 1,2,3 )
print o.x
o.show()

lock = thread.allocate_lock()

GO = True
def ctypes_thread():
	print( 'thread start' )
	while GO:
		lock.acquire()
		o.add( 0.1, 0.2, 0.3 )
		lock.release()
	print( 'thread exit' )


thread.start_new_thread( ctypes_thread, () )

for i in range(10000*10):
	# getting values from _pointer.contents is thread safe even without locks
	print o.x, o.y, o.z

	lock.acquire()
	o.show()						# not thread safe
	lock.release()

```