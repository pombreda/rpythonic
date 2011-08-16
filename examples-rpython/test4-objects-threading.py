#!/usr/bin/python
import os, sys, thread, time, math
sys.path.append('..')
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
################################
rpy = rpythonic.RPython('threadsafety_test')

@rpy.object
class MyRpyObject(object):
	def __init__(self):
		self._thread1 = []
		self._thread2 = []
	def append1(self, value=.0 ):
		self._thread1.append( math.sqrt(value) )
	def append2(self, value=.0 ):
		self._thread2.append( math.sqrt(value) )

rpy.cache(refresh=1)	

o = MyRpyObject()

lock = thread.allocate_lock()

FINISHED = 0
def thread1( values ):
	global FINISHED
	print( 'thread1 start' )
	while values:
		v = values.pop()
		#lock.acquire()
		o.append1( v )
		#lock.release()
	print( 'thread1 exit' )
	FINISHED += 1

def thread2( values ):
	global FINISHED
	print( 'thread2 start' )
	while values:
		v = values.pop()
		#lock.acquire()
		o.append2( v )
		#lock.release()
	print( 'thread2 exit' )
	FINISHED += 1

start = time.time()
thread.start_new_thread( thread1, ( [.8]*500000, ) )
thread.start_new_thread( thread2, ( [.4]*500000, ) )

while FINISHED != 2:
	time.sleep(0.5)

#print( o._thread1 )
#print( o._thread2 )

print('time', time.time()-start )
print('----------------test exit----------------')

# test proves garbage collectors thread safe from python locks:
#		. ref (reference counting)
#		. boehm
## unsafe: hybrid


