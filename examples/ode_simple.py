#!/usr/bin/python
## updated nov 27 2011 ##
import os,sys, time, ctypes
from random import *
import ode

## why are these segfaulting?
#print('ode config - ODE_single_precision:%s' %ode.CheckConfiguration('ODE_single_precision'))
#print('ode config - ODE_double_precision:%s' %ode.CheckConfiguration('ODE_double_precision'))
#print('ode config - ODE_EXT_trimesh:%s' %ode.CheckConfiguration('ODE_EXT_trimesh'))
#print('ode config - ODE_EXT_gimpact:%s' %ode.CheckConfiguration('ODE_EXT_gimpact'))
#print('ode config - ODE_EXT_gyroscopic:%s' %ode.CheckConfiguration('ODE_EXT_gyroscopic'))


print( 'init ode')
ode.InitODE()
print( 'ode init ok' )

world = ode.WorldCreate()
print( world )
#ode.WorldSetQuickStepNumIterations(world, 16)
ode.WorldSetGravity( world, 0, 0, 0 )

print('default linear damping', world.GetLinearDamping())
print('default angular damping', world.GetAngularDamping())

print('default contact max correcting vel', world.GetContactMaxCorrectingVel())
print('default quick step iterations', world.GetQuickStepNumIterations())
print('default ERP', world.GetERP() )
print('default CFM', world.GetCFM() )


bodies = []
for i in range(1):
	body = ode.BodyCreate( world )
	bodies.append( body )
	ode.BodySetPosition( body, 0.99, 0.77, 0.1 )
	mass = ode.Mass()		# no function to create a mass?
	ode.MassSetSphere( mass, 0.1, 0.5)		# density, radius
	ode.BodySetMass( body, mass )


for i in range(5):
	ode.WorldQuickStep( world, 1.1 )
	#ode.WorldStep( world, 0.01 )
	vec = ode.Vector4()

	print( 'frame', i )
	for b in bodies:
		ode.BodyAddForce(b, 0.1,.0,.0)
		ode.BodyCopyPosition( b, vec )
		x,y,z,w = vec
		print( x,y,z )


ode.CloseODE()
print( 'simple ode test complete' )


