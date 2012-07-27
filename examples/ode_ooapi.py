#!/usr/bin/python
import os,sys, time, ctypes
from random import *
import ode
#if '..' not in sys.path: sys.path.append( '..' )
#import rpythonic
#ode = rpythonic.module( 'ode' )
#assert ode

print( 'init ode')
ode.InitODE()
print( 'ode init ok' )

world = ode.WorldCreate()
print( dir(world) )

try: print( 'angular damping default', world.GetAngularDamping() )
except: print( 'angular damping missing on windows?' )

print( 'quickstep default', world.GetQuickStepNumIterations() )
world.SetQuickStepNumIterations(24)
world.SetGravity( 0, 0, -.9 )

space = ode.SimpleSpaceCreate()
print( 'space', space )

bodies = []
masses = []
for i in range(20):
	body = ode.BodyCreate( world )
	print( body )
	bodies.append( body )
	body.SetPosition( random(), random(), random() )
	mass = ode.Mass()	# no function to create a mass?
	print( mass )
	mass.SetSphere( 0.1+random(), 0.5)		# density, radius
	print( 'setting mass' )
	body.SetMass( mass )
	masses.append( mass )

print('starting sim')
start = time.time()
for i in range(500):
	world.QuickStep( 1.1 )
	print( 'frame', i )
	for b in bodies:
		b.AddForce(0.1, .0, .0)
		x,y,z = b.GetPosition()
		print(x,y,z)
print('time', time.time()-start)

ode.CloseODE()
print( 'object oriented API ode test complete' )


