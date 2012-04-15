#!/usr/bin/python
print('THIS TEST IS DEPRECATED')
assert 0

import os, sys, thread, time
sys.path.append('..')
import rpythonic
rpythonic.set_cache( '../cache' )
rpythonic.set_pypy_root( '../../pypy' )
################################
rpy = rpythonic.RPython('stackles_test')


class _RPYSINGLETON(object):
	def __init__(self):
		self.instances = []
RPYSINGLETON = _RPYSINGLETON()

@rpy.object
class Shared(object):
	def __init__(self):
		self.value = 0
		self.increment = 1
		self.flag = False
		RPYSINGLETON.instances.append( self )
	def get_value(self): return self.value
	def reset_value(self): self.value = 0
	#@rpy.threadsafe()
	def show(self): print 'hi from rpy', self.value
	#@rpy.threadsafe()
	def set_increment(self, v=1):
		#rpy.acquire_lock()
		time.sleep(0.1)
		print 'hi from rpy - set increment', v
		self.increment = v
		#rpy.release_lock()

	def set_bool( self, b=True ):
		# "If this code appears on more than one thread, It could happen that bool_var is sucked into the processor and set to false but before the variable is returned to memory, 
		# a context switch could occur where the bool_var is set to true by another thread. Then this thread thaws and the false is put in memory where the other thread expected true. 
		# That is a race condition and a crash is not long in coming." 
		#http://bytes.com/topic/c/answers/649495-boolean-variable-thread-safety-c
		self.flag = b

class MyThunk( rpy.AbstractThunk ):
	def __init__( self, func, coros, arg ):
		self.func = func
		self.coros = coros
		self.arg = arg
	def call(self):
		#rpy.acquire_lock()
		self.func( self.coros, self.arg )
		#rpy.release_lock()

def producer( coros, lst ):
	print( '	entering producer' )
	active = True
	while active:
		rpy.acquire_lock()
		print('	producer')
		if RPYSINGLETON.instances:
			inc = RPYSINGLETON.instances[0].increment
			lst.append( inc )
			print( '	producer using increment: %s' %inc )
		else: lst.append( 10 )
		if active:
			coros['consumer1'].switch()
			#lst.append( 'b' )
			#coros['consumer2'].switch()
		#if RPYSINGLETON.instances and RPYSINGLETON.instances[0].value >= 100: active = False
		rpy.release_lock()
	print('	producer exit')

def consumer( coros, lst ):
	print( '	entering consumer' )
	active = True
	while active:
		if not len(lst): coros['main'].switch()
		batch = [ lst.pop(-1) ]
		while len(lst) > 3: batch.append( lst.pop(-1) )
		# do something with the batch
		print('	consumer' )
		for item in batch:
			#print( '	rpy-batch', item )
			if RPYSINGLETON.instances:
				#if RPYSINGLETON.instances[0].value < 100:
				RPYSINGLETON.instances[0].value += item
				#else: active = False
			else: print( '	WAITING for shared' )
		coros['main'].switch()
	print('	consumer exit')

@rpy.stackless
def stackless_thread():
	print('rpython stackless entrypoint')
	coro_main = rpythonic.RPython.Coroutine.getcurrent()
	coro1 = rpythonic.RPython.Coroutine()
	coro2 = rpythonic.RPython.Coroutine()
	coros = { 'main':coro_main, 'consumer1':coro1, 'consumer2':coro2 }
	print( 'coros created')

	exchange = []
	thunk0 = MyThunk( producer, coros, exchange )
	coro_main.bind( thunk0 )
	thunk1 = MyThunk( consumer, coros, exchange )
	coro1.bind( thunk1 )
	thunk2 = MyThunk( consumer, coros, exchange )
	coro2.bind( thunk2 )

	print('starting test...')
	coro_main.switch()	# must switch into main first
	print( 'rpython stackless test done' )


mod = rpy.cache(refresh=0)

############### testing ##############
s = Shared()
lock = rpy.Lock()
lock.release()
#s.set_increment( 1 )
#mod.pypy_g_slp_entry_point(None)

#lock.release()
#lock = thread.allocate()
print( 'starting thread...' )
#stackless_thread(None)
#thread.start_new_thread( stackless_thread, (None,) )
#lock = rpy.Lock()
#lock.release()
time.sleep(1)
import random
raise
print( 'python thread--------->' )
i = 0
#while s.value < 99:
while i < 30:
	#print( 'MAIN THREAD: s.value=', s.value )
	print( 'MAIN THREAD: s.value=' )
	i += 1
	#lock.acquire()
	#s.show()		# not threadsafe to only lock on the python side
	s.set_increment( int(random.uniform(0,5)) )
	#lock.release()
	#s.set_bool( True )
	#lock.release()
	#time.sleep(0.0001)

