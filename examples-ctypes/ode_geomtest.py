#!/usr/bin/python
import os,sys, time, ctypes
from random import *

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
ode = rpythonic.module( 'ode' )
assert ode

print( 'init ode')
ode.InitODE()
print( 'ode init ok' )

world = ode.WorldCreate()

space = ode.SimpleSpaceCreate()
print( 'space', space )

g = ode.CreateSphere( space, 1.0 )
test = 'hello world'
#g.SetData( ctypes.py_object(test) )	# segfaults at 370, gc?
_ptr_ = ctypes.pointer( ctypes.py_object(test) )	# MUST hold the pointer!
g.SetData( _ptr_ )
#del _ptr_		# segfaults at 500

for i in range(1000):
	print( i )
	ptr = ctypes.cast( g.GetData(), ctypes.POINTER(ctypes.py_object) )
	#print( ptr.contents.value.value )
	#assert ptr.contents.value.value is test
	print( ptr.contents.value )
	assert ptr.contents.value is test

ode.CloseODE()
print( 'geom Get/SetData test complete' )


