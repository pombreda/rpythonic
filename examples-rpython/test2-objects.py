#!/usr/bin/python
import os, sys, time
sys.path.append('..')
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
################################
rpy = rpythonic.RPython('test2')

@rpy.object
class MyRpyObject(object):
	def __init__(self, x=.0, y=.0, z=.0):
		self.x = x; self.y = y; self.z = z
	def add( self, x=.0,y=.0,z=.0 ): self.x += x; self.y += y; self.z += z
	def sum( self ): return self.x + self.y + self.z
	def show( self ): print( '<%s %s %s>' %(self.x, self.y, self.z))

rpy.cache(refresh=1)	

o = MyRpyObject()
print( o.x )
o.add( 1.1, 2.2, 3.3 )
print( o.x )
o.show()

if '--thread-test' in sys.argv:
	import thread
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

	GO = False
	print('----------------test exit----------------')
	os.abort()

	# test proves garbage collectors thread safe from python locks:
	#		. ref (reference counting)
	#		. boehm
	## unsafe: hybrid


