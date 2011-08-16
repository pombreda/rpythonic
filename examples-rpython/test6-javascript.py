#!/usr/bin/python
import os, sys
sys.path.append('..')
import rpythonic
rpythonic.set_pypy_root( '../../pypy' )
################################
rpy = rpythonic.RPython( 'jstest', platform='javascript' )
rprint = rpy.get_rprint()           # normal print won't work

def add( a=1, b=1000 ):			# keyword defaults are given
	return a+b

def sub( a, b ):
	return a-b

@rpy.standalone
def main():
	rprint( 'hello javascript world' )
	x = add( 1, 2 )
	y = 0
	for i in range(10):
		y = sub( x+y, 100 )
		rprint( y )
	rprint( 'done' )

js = rpy.compile()
print( js )

js.test()

open( '_test6_.js', 'wb' ).write( js.get_source() )

print( 'javascript test complete' )


