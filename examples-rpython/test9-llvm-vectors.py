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
	#h = 11
	i = 0
	c = 0.0
	#while i < 100000*100000:
	while i < 1410065408:
		v = a + b
		#a[ 0 ] += b[0]
		#a[ 1 ] += b[1]
		#a[ 2 ] += b[2]
		#w = a[0] + a[1] + a[2]
		#c += w
		#b[2] += w
		c += v[0]	# + v[1] + v[2]
		#x = h * 2
		#i += x
		i += 1
	#c = a[0] + a[1] + a[2] + b[0] + b[1] + b[2]

	return c


rpy.cache( refresh=1 )
print('CACHED: starting llvm test...')
############### testing ##############
for i in range(10):
	start = time.time()
	a = test( 0.56, 0.0000066, 0.000033,  0.04, 0.0000022, 0.00000011)
	print('end of llvm benchmark:', time.time()-start)
	print('test result:', a)

#pypy 846 039 265 vs rpyllvm 16 777 216.0



