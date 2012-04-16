#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
################################
rpy = rpythonic.RPython( 'test1' )

@rpy.bind()					# declare arg types is optional if,
def add( a=1, b=1000 ):			# keyword defaults are given
	print('rpy.add %s %s' %(a,b))
	return a+b

@rpy.bind(a=float, b=float)
def sub( a, b ):
	print('rpy.sub %s %s' %(a,b))
	return a-b


#rpythonic.set_cache( '/tmp' )					# defaults to /home/user/.rpythonic
rpy.cache( refresh=1)					# only compiles if cache is dirty

############### testing ##############

print( add( 100, 9 ))
print( add( 1, 99 ))
print( sub( 1.0, .8 ))


