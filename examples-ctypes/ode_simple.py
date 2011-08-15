#!/usr/bin/python
## passed rpythonic 0.3.6 - may 19th ##
import os,sys, time, ctypes
from random import *

if '..' not in sys.path: sys.path.append( '..' )
import rpythonic
ode = rpythonic.module( 'ode' )
assert ode


print( 'init ode')
ode.InitODE()
print( 'ode init ok' )

## why are these segfaulting?
#print('ode config - ODE_single_precision:%s' %ode.CheckConfiguration('ODE_single_precision'))
#print('ode config - ODE_double_precision:%s' %ode.CheckConfiguration('ODE_double_precision'))
#print('ode config - ODE_EXT_trimesh:%s' %ode.CheckConfiguration('ODE_EXT_trimesh'))
#print('ode config - ODE_EXT_gimpact:%s' %ode.CheckConfiguration('ODE_EXT_gimpact'))
#print('ode config - ODE_EXT_gyroscopic:%s' %ode.CheckConfiguration('ODE_EXT_gyroscopic'))


world = ode.WorldCreate()
print( world )
ode.WorldSetQuickStepNumIterations(world, 16)
ode.WorldSetGravity( world, 0, 0, 0 )

bodies = []
for i in range(1):
	body = ode.BodyCreate( world ); bodies.append( body )
	ode.BodySetPosition( body, 0.99, 0.77, 0.1 )
	print('creating mass')
	mass = ode.Mass()	# no function to create a mass?
	print( 'mass before', mass.mass )
	ode.MassSetSphere( mass, 0.1, 0.5)		# density, radius
	print( 'mass after', mass.mass )

	#dMassAdjust(ptrmass,2.0)

vec4 = ctypes.c_double*4
vecp = ctypes.POINTER( vec4 )

for i in range(5):
	ode.WorldQuickStep( world, 1.1 )
	#ode.WorldStep( world, 0.01 )
	vec = vec4()

	print( 'frame', i )
	for b in bodies:
		ode.BodyAddForce(b, 0.1,.0,.0)
		#v = dBodyGetPosition(b)		# only remains valid while struct unchanged
		#v = ctypes.cast( v, vec3p )
		#x,y,z = v.contents
		#print(x,y,z)
		ode.BodyCopyPosition( b, vec )
		x,y,z,w = vec
		print( x,y,z )


ode.CloseODE()
print( 'simple ode test complete' )


