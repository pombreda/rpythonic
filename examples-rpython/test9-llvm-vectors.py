#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
################################
rpy = rpythonic.RPython( 'test9', backend='llvm' )

class Vector(object):
	def __init__(self, x=0, y=0, z=0):
		self.x = x
		self.y = y
		self.z = z

	def __getitem__(self, index):
		r = 0
		if index == 0: r = self.x
		elif index == 1: r = self.y
		elif index == 2: r = self.z
		return r

	def __setitem__(self, index, value):
		if index == 0: self.x = value
		if index == 1: self.y = value
		if index == 2: self.z = value


@rpy.bind()
def test():
	a = Vector(10,20,30)
	return a[1]

rpy.cache( refresh=1 )
print('CACHED: starting test...')
############### testing ##############
print( 'test result:', test() )

