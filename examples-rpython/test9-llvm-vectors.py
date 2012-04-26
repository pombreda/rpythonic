#!/usr/bin/python
import os, sys, time
sys.path.append('..')
import rpythonic
################################
rpy = rpythonic.RPython( 'test9', backend='llvm' )

@rpy.vector( type='float32', length=4 )
class Vector(object):
	def __init__(self, x=.0, y=.0, z=.0):
		self.x = x
		self.y = y
		self.z = z

	def __getitem__(self, index):
		r = .0
		if index == 0: r = self.x
		elif index == 1: r = self.y
		elif index == 2: r = self.z
		return r

	def __setitem__(self, index, value):
		if index == 0: self.x = value
		if index == 1: self.y = value
		if index == 2: self.z = value

	def __add__( self, other ):
		x = self.x + other.x
		y = self.y + other.y
		z = self.z + other.z
		return Vector( x,y,z )

#@rpy.llvm_hints( sizeof={'int':64} )
@rpy.bind(x1=float, y1=float, z1=float, x2=float, y2=float, z2=float)
def test(x1, y1, z1, x2, y2, z2):
	a = Vector(x1, y1, z1)
	b = Vector(x2, y2, z2)
	i = 0
	c = 0.0
	while i < 16000000:
		v = a + b
		c += v[0] + v[1] + v[2]
		i += 1
	#assert isinstance(a,Vector)
	#c = a[0] + a[1] + a[2]
	return c


rpy.cache( refresh=1 )
print('CACHED: starting llvm test...')
############### testing ##############
for i in range(5):
	start = time.time()
	a = test( 0.3, 0.3, 0.3,  0.3, 0.3, 0.3)
	print('end of llvm benchmark:', time.time()-start)
	print('test result:', a)


