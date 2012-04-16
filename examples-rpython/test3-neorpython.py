#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
################################
rpy = rpythonic.RPython( 'test3' )


class R(object):
	def __init__(self):
		self.values = []
	def append( self, value ):
		self.values.append( value )
	def __getitem__(self, index): return self.values[ index ]
	def __setitem__(self, index, value): self.values[ index ] = value

@rpy.bind()
def test_neo_rpy( arg=1 ):
	r = R()
	r.append( arg )
	r.append( 100 )
	r[0] = r[0] + r[1]
	#r.append( _sub_test(r) )
	return r[0]

def _sub_test( r ):
	assert isinstance(r, R)
	a = r[1]
	return a

rpy.cache( refresh=1)

print( test_neo_rpy( 1 ) )


